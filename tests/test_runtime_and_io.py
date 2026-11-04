import copy
import json
import socket
import cv2
import numpy as np
import pytest
from pymavlink.dialects.v20 import common as mav
from omniavoid.cli import main
from omniavoid.config import validate_config
from omniavoid.dataset import generate_demo, read_dataset
from omniavoid.geometry import Camera
from omniavoid.protocol import Snapshot, Transport, obstacle_message
from omniavoid.stereo import StereoPair


def test_udp_transport_emits_parseable_mavlink2_to_real_local_socket():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(1)
    transport = Transport(f"udpout:127.0.0.1:{receiver.getsockname()[1]}")
    try:
        encoder = mav.MAVLink(transport, srcSystem=191, srcComponent=196)
        encoder.send(obstacle_message(Snapshot(1, 1_000_000, (123,) * 72), 6))
        packet, _ = receiver.recvfrom(4096)
        decoded = mav.MAVLink(None).parse_buffer(packet)[0]
        assert decoded.frame == mav.MAV_FRAME_BODY_FRD
        assert decoded.distances == [123] * 72
    finally:
        transport.close()
        receiver.close()


def test_real_mask_removes_obstructed_regions(cfg, tmp_path):
    d = copy.deepcopy(cfg["cameras"][0])
    path = tmp_path / "black_mask.png"
    cv2.imwrite(str(path), np.zeros((d["height"], d["width"]), np.uint8))
    d["valid_mask_path"] = str(path)
    pair = StereoPair(Camera.from_dict(d), Camera.from_dict(cfg["cameras"][1]), cfg["stereo"])
    assert not pair.maps[0][2].any()


def test_invalid_mask_does_not_silently_enable_unmasked_capture(cfg):
    d = copy.deepcopy(cfg["cameras"][0])
    d["valid_mask_path"] = "this-mask-does-not-exist.png"
    with pytest.raises(ValueError, match="valid_mask_path"):
        StereoPair(Camera.from_dict(d), Camera.from_dict(cfg["cameras"][1]), cfg["stereo"])


def test_configuration_rejects_invalid_search_and_nonfinite_values(cfg):
    invalid = copy.deepcopy(cfg)
    invalid["stereo"]["num_disparities"] = 15
    with pytest.raises(ValueError):
        validate_config(invalid)
    invalid = copy.deepcopy(cfg)
    invalid["timing"]["max_age_ms"] = float("nan")
    with pytest.raises(ValueError):
        validate_config(invalid)


def test_replay_cli_cannot_be_given_transmit_flag():
    with pytest.raises(SystemExit) as error:
        main(["replay", "--config", "a", "--manifest", "b", "--output", "c", "--transmit"])
    assert error.value.code == 2


def test_dataset_images_and_synthetic_label_round_trip(cfg, tmp_path):
    _, manifest = generate_demo(cfg, tmp_path)
    row, images = next(read_dataset(manifest))
    assert row["synthetic"] is True
    assert set(images) == {"front", "right", "back", "left"}
    data = json.loads(manifest.read_text())
    data["frames"].append(data["frames"][0])
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="timestamps"):
        list(read_dataset(manifest))
