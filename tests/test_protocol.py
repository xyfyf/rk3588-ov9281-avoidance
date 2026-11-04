import copy
import time
from pymavlink.dialects.v20 import common as mav
from omniavoid.config import live_transmit_errors
from omniavoid.protocol import Snapshot, SnapshotStore, Publisher, obstacle_message
from omniavoid.sectors import UNKNOWN


def decode(payload):
    parser = mav.MAVLink(None)
    return parser.parse_buffer(payload) or []


def test_actual_mavlink2_roundtrip_and_acquisition_timestamp():
    stamp = 1_795_000_000_123_456_000
    snapshot = Snapshot(100, stamp, tuple([123] + [UNKNOWN] * 71))
    message = obstacle_message(snapshot, 6)
    wire = message.pack(mav.MAVLink(None, srcSystem=191, srcComponent=196))
    assert wire[0] == 0xFD
    result = decode(wire)[0]
    assert result.get_msgId() == 330
    assert result.frame == mav.MAV_FRAME_BODY_FRD
    assert result.time_usec == stamp // 1000
    assert result.angle_offset == 0 and result.increment_f == 5
    assert result.distances[0] == 123 and result.distances[1] == UNKNOWN
    assert result.min_distance == 0 and result.max_distance == 600


def test_unknown_payload_never_uses_no_obstacle_sentinel():
    msg = obstacle_message(None, 6)
    assert msg.distances == [UNKNOWN] * 72
    assert 601 not in msg.distances


def test_saturated_contact_is_not_discarded_by_ardupilot():
    # Tested custom ArduPilot discards wire distance 0, but accepts 1 cm.
    snapshot = Snapshot(100, 1000, tuple([0, 1, 123] + [UNKNOWN] * 69))
    message = obstacle_message(snapshot, 6)
    result = decode(message.pack(mav.MAVLink(None)))[0]
    assert result.distances[:3] == [1, 1, 123]
    assert result.distances[3:] == [UNKNOWN] * 69
    assert snapshot.distances[0] == 0


def test_snapshot_age_is_based_on_capture_not_publish():
    store = SnapshotStore()
    value = Snapshot(1_000_000_000, 1_000_000_000, (100,) * 72)
    store.update(value)
    assert store.read(1_050_000_000, 100)[0] is value
    assert store.read(1_100_000_001, 100) == (None, "stale")
    assert store.read(999_999_999, 100) == (None, "stale")


class MemoryTransport:
    def __init__(self):
        self.messages = []
    def write(self, payload):
        self.messages.extend(decode(payload))
        return len(payload)
    def close(self):
        pass


def test_sender_independently_expires_ranges_when_processor_stops(cfg):
    cfg = copy.deepcopy(cfg)
    cfg["timing"].update(max_age_ms=25, publish_hz=100)
    store, transport = SnapshotStore(), MemoryTransport()
    store.update(Snapshot(time.monotonic_ns(), time.time_ns(), (100,) * 72))
    sender = Publisher(cfg, store, transport)
    sender.start()
    try:
        time.sleep(0.09)
    finally:
        sender.close()
    messages = [m for m in transport.messages if m.get_type() == "OBSTACLE_DISTANCE"]
    assert messages[0].distances == [100] * 72
    assert messages[-2].distances == [UNKNOWN] * 72
    assert all(m.get_type() in ("OBSTACLE_DISTANCE", "HEARTBEAT", "STATUSTEXT") for m in transport.messages)


def test_live_gate_cannot_transmit_replay_or_synthetic(cfg):
    cfg["calibration"].update(validated=True, body_alignment_verified=True, occlusion_masks_verified=True, synthetic=False)
    cfg["capture"].update(hardware_sync_verified=True, exposure_timestamp_verified=True)
    assert live_transmit_errors(cfg, "v4l2") == []
    assert live_transmit_errors(cfg, "replay")
    cfg["calibration"]["synthetic"] = True
    assert live_transmit_errors(cfg, "v4l2")
