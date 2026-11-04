import copy
import numpy as np
import pytest
from omniavoid.geometry import Camera, project, unproject, rectified_basis


def test_full_angle_projection_retains_rear_hemisphere(cfg):
    camera = Camera.from_dict(cfg["cameras"][0])
    theta = np.deg2rad(np.array([-98, -90, -45, 0, 45, 90, 98]))
    rays = np.column_stack((np.sin(theta), np.zeros(len(theta)), np.cos(theta)))
    pixels, valid = project(camera, rays)
    assert valid.all()
    reconstructed, supported = unproject(camera, pixels)
    assert supported.all()
    assert np.allclose(rays, reconstructed, atol=1e-9)
    assert pixels[0, 0] < pixels[1, 0] < camera.cx < pixels[-1, 0]


def test_non_monotonic_lens_rejected(cfg):
    d = copy.deepcopy(cfg["cameras"][0])
    d["distortion"][0] = -1
    with pytest.raises(ValueError, match="non-monotonic"):
        Camera.from_dict(d)


@pytest.mark.parametrize("pair_index", range(4))
def test_virtual_rectification_epipolar_and_metric_scale(cfg, pair_index):
    cameras = {d["name"]: Camera.from_dict(d) for d in cfg["cameras"]}
    a, b = [cameras[n] for n in cfg["pairs"][pair_index]]
    R, baseline = rectified_basis(a, b)
    pvirtual = np.array([[0.1, 0.05, 2], [-0.2, -0.1, 3]])
    pbody = pvirtual @ R.T + a.t
    left, right = (pbody - a.t) @ R, (pbody - b.t) @ R
    f = 140.0
    ul, ur = f * left[:, 0] / left[:, 2], f * right[:, 0] / right[:, 2]
    assert np.allclose(left[:, 1] / left[:, 2], right[:, 1] / right[:, 2])
    assert np.allclose(f * baseline / (ul - ur), pvirtual[:, 2])
    assert np.all(ul - ur > 0)
    assert np.isclose(np.linalg.det(R), 1)


def test_camera_mask_excludes_rim_outside_calibrated_support(cfg):
    camera = Camera.from_dict(cfg["cameras"][0])
    theta = np.deg2rad(99)
    _, valid = project(camera, np.array([[np.sin(theta), 0, np.cos(theta)]]), guard_deg=3)
    assert not valid[0]
