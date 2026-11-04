import cv2
import numpy as np
import pytest
from omniavoid.geometry import Camera
from omniavoid.stereo import StereoPair, StereoPipeline
from omniavoid.dataset import render_camera
from omniavoid.runtime import process_images


def test_constant_disparity_recovers_metric_depth(cfg):
    cameras = [Camera.from_dict(c) for c in cfg["cameras"]]
    pair = StereoPair(cameras[0], cameras[1], cfg["stereo"])
    rng = np.random.default_rng(3)
    image_l = rng.integers(0, 256, size=(pair.height, pair.width), dtype=np.uint8)
    image_l = cv2.GaussianBlur(image_l, (3, 3), 0.6)
    disparity = 12
    image_r = np.zeros_like(image_l)
    image_r[:, :-disparity] = image_l[:, disparity:]
    points, _ = pair.cloud_from_rectified(image_l, image_r)
    virtual = (points - cameras[0].t) @ pair.R
    assert len(points) > 1000
    assert abs(np.median(virtual[:, 2]) - pair.f * pair.baseline / disparity) < 0.02


def test_featureless_frames_produce_unknown(cfg):
    pipeline = StereoPipeline(cfg)
    try:
        images = {c["name"]: np.full((c["height"], c["width"]), 127, np.uint8) for c in cfg["cameras"]}
        result, points, _ = process_images(pipeline, cfg, images)
        assert len(points) == 0
        assert result["valid_sectors"] == 0
        assert result["distances_cm"] == [65535] * 72
    finally:
        pipeline.close()


def test_no_implicit_image_resize_or_bit_depth_conversion(cfg):
    a, b = [Camera.from_dict(d) for d in cfg["cameras"][:2]]
    pair = StereoPair(a, b, cfg["stereo"])
    with pytest.raises(ValueError, match="no implicit resizing"):
        pair.rectify({a.name: np.zeros((a.height, a.width), np.uint16)})


def test_rendered_four_camera_scene_detects_front_box_and_room(cfg):
    images = {d["name"]: render_camera(Camera.from_dict(d)) for d in cfg["cameras"]}
    pipeline = StereoPipeline(cfg)
    try:
        result, _, _ = process_images(pipeline, cfg, images)
        # Box front face is x=1.4m; conservative range must be near and not far wall.
        assert 90 <= result["distances_cm"][0] <= 145
        assert result["valid_sectors"] >= 60
        assert result["distances_cm"][18] > 150
        assert result["distances_cm"][36] > 200
    finally:
        pipeline.close()
