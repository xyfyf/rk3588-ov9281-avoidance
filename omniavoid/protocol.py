from __future__ import annotations

import threading
from dataclasses import dataclass
import time
import socket
import numpy as np
from .sectors import UNKNOWN


@dataclass(frozen=True)
class Snapshot:
    capture_monotonic_ns: int
    capture_epoch_ns: int
    distances: tuple
    processing_ms: float = 0.0


class SnapshotStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.value = None
        self.failure = "waiting_for_frames"

    def update(self, value):
        if len(value.distances) != 72 or any(not 0 <= int(d) <= UNKNOWN for d in value.distances):
            raise ValueError("Expected exactly 72 valid uint16 distances")
        with self.lock:
            self.value, self.failure = value, ""

    def invalidate(self, reason):
        with self.lock:
            self.value, self.failure = None, str(reason)

    def read(self, now_ns, max_age_ms):
        with self.lock:
            value, failure = self.value, self.failure
        if value is None:
            return None, failure
        age = now_ns - value.capture_monotonic_ns
        if age < 0 or age > int(max_age_ms * 1e6):
            return None, "stale"
        return value, "ok" if any(d != UNKNOWN for d in value.distances) else "no_valid_depth"


def obstacle_message(snapshot, max_range_m, angle_offset_deg=0.0):
    # Import v2 explicitly: extension fields frame/angle_offset must be retained.
    from pymavlink.dialects.v20 import common as mav
    # Tested ArduPilot build drops 0 entirely. Encode saturated contact at the
    # smallest accepted positive value; preserve internal 0 and unknown sectors.
    distances = [1 if d == 0 else d for d in snapshot.distances] if snapshot else [UNKNOWN] * 72
    timestamp = snapshot.capture_epoch_ns // 1000 if snapshot else time.time_ns() // 1000
    return mav.MAVLink_obstacle_distance_message(
        time_usec=timestamp, sensor_type=mav.MAV_DISTANCE_SENSOR_UNKNOWN,
        distances=distances, increment=5,
        # No sub-centimetre accuracy is claimed by this wire representation.
        min_distance=0, max_distance=int(max_range_m * 100), increment_f=5.0,
        angle_offset=float(angle_offset_deg), frame=mav.MAV_FRAME_BODY_FRD)


class Transport:
    def __init__(self, endpoint, baud=460800):
        self.serial = None
        self.socket = None
        if endpoint.startswith("udpout:"):
            host, port = endpoint[len("udpout:"):].rsplit(":", 1)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(0.1)
            self.address = (host, int(port))
        elif endpoint.startswith("serial:"):
            import serial
            self.serial = serial.Serial(endpoint[len("serial:"):], baudrate=baud,
                                        timeout=0.1, write_timeout=0.1)
        else:
            raise ValueError("Use udpout:HOST:PORT or serial:/dev/ttySx")

    def write(self, payload):
        if self.serial:
            sent = self.serial.write(payload)
            if sent != len(payload):
                raise IOError("Partial serial write")
            return sent
        return self.socket.sendto(payload, self.address)

    def close(self):
        if self.serial:
            self.serial.close()
        if self.socket:
            self.socket.close()


class Publisher:
    """Independent sender so a blocked image processor cannot resend stale ranges."""
    def __init__(self, cfg, store, transport):
        from pymavlink.dialects.v20 import common as mav
        self.cfg, self.store, self.transport, self.constants = cfg, store, transport, mav
        self.encoder = mav.MAVLink(transport, srcSystem=cfg["mavlink"]["system_id"],
                                  srcComponent=cfg["mavlink"]["component_id"])
        self.stop_event = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._loop, name="mavlink-publisher", daemon=True)

    def start(self):
        self.thread.start()

    def _loop(self):
        cfg, mav = self.cfg, self.constants
        interval = 1 / cfg["timing"]["publish_hz"]
        next_tick, last_heartbeat, previous = time.monotonic(), 0.0, None
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                snapshot, health = self.store.read(time.monotonic_ns(), cfg["timing"]["max_age_ms"])
                self.encoder.send(obstacle_message(snapshot, cfg["range"]["max_m"], cfg["mavlink"]["angle_offset_deg"]))
                if now - last_heartbeat >= 1:
                    self.encoder.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID,
                                               0, 0, mav.MAV_STATE_ACTIVE if health == "ok" else mav.MAV_STATE_CRITICAL)
                    last_heartbeat = now
                if health != previous:
                    self.encoder.statustext_send(mav.MAV_SEVERITY_INFO if health == "ok" else mav.MAV_SEVERITY_WARNING,
                                                 ("omniavoid: " + health).encode("ascii", "replace")[:50])
                    previous = health
                next_tick += interval
                if next_tick < time.monotonic():
                    next_tick = time.monotonic()
                self.stop_event.wait(max(0, next_tick - time.monotonic()))
        except Exception as exc:
            self.error = exc
            self.stop_event.set()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        try:
            self.encoder.send(obstacle_message(None, self.cfg["range"]["max_m"], self.cfg["mavlink"]["angle_offset_deg"]))
        finally:
            self.transport.close()
