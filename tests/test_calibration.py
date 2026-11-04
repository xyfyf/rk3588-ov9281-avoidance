import copy
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from omniavoid.calibration import make_board, write_board, detect_grid, fit_intrinsics, fit_extrinsics
from omniavoid.geometry import Camera, nominal_pose, project_kb4


def test_printable_board_has_unambiguous_metric_marker_ids(tmp_path):
    spec = write_board(tmp_path)
    image = cv2.imread(str(tmp_path / "aruco_grid.png"), cv2.IMREAD_GRAYSCALE)
    _, dictionary, lookup = make_board()
    objects, pixels = detect_grid(image, dictionary, lookup)
    assert len(objects) == 35 * 4
    assert spec["print_width_mm"] == 275
    assert np.isclose(spec["print_height_mm"], 200)
    assert len(pixels) == len(objects)


def test_intrinsic_solver_recovers_synthetic_projection(cfg):
    truth = copy.deepcopy(cfg["cameras"][0])
    truth["distortion"] = [0.012, -0.001, 0.0001, 0]
    camera = Camera.from_dict(truth)
    xx, yy = np.meshgrid(np.arange(6) * 0.035, np.arange(5) * 0.035)
    objects = np.column_stack((xx.ravel(), yy.ravel(), np.zeros(xx.size)))
    observations = []
    rng = np.random.default_rng(42)
    for i in range(16):
        r = Rotation.from_rotvec(rng.uniform(-0.45, 0.45, 3)).as_matrix()
        centre = np.array([rng.uniform(-0.65, 0.65), rng.uniform(-0.4, 0.4), rng.uniform(0.45, 0.9)])
        t = centre - r @ objects.mean(axis=0)
        p = objects @ r.T + t
        pixels, _ = project_kb4(p, truth["intrinsics"], truth["distortion"])
        observations.append({"group": i, "objects": objects, "pixels": pixels})
    seed = copy.deepcopy(truth)
    seed["intrinsics"][0] *= 1.025
    seed["intrinsics"][1] *= 0.975
    seed["distortion"] = [0, 0, 0, 0]
    fitted, poses, report = fit_intrinsics(seed, observations)
    assert len(poses) >= 12
    assert report["rms_px"] < 0.1
    assert abs(fitted["intrinsics"][0] - truth["intrinsics"][0]) / camera.fx < 0.02


def test_bundle_adjustment_recovers_adjacent_camera_baselines(cfg):
    cameras = [copy.deepcopy(d) for d in cfg["cameras"]]
    xx, yy = np.meshgrid(np.arange(4) * 0.03, np.arange(3) * 0.03)
    objects = np.column_stack((xx.ravel(), yy.ravel(), np.zeros(xx.size)))
    poses = {d["name"]: {} for d in cameras}
    observations = []
    for pair in range(4):
        for sample in range(3):
            group = pair * 3 + sample
            rbb, forward = nominal_pose(45 + 90 * pair + sample * 2, 1)
            tbb = forward * (1 + 0.1 * sample) - rbb @ objects.mean(axis=0)
            for j in (pair, (pair + 1) % 4):
                c = Camera.from_dict(cameras[j])
                rcb, tcb = c.R.T @ rbb, c.R.T @ (tbb - c.t)
                pose = np.r_[Rotation.from_matrix(rcb).as_rotvec(), tcb]
                poses[c.name][group] = pose
                pc = objects @ rcb.T + tcb
                pixels, _ = project_kb4(pc, cameras[j]["intrinsics"], cameras[j]["distortion"])
                observations.append({"group": group, "camera": c.name, "objects": objects, "pixels": pixels})
    fitted, report = fit_extrinsics(cameras, observations, poses)
    assert report["rms_px"] < 1e-5
    for a, b in zip(fitted, cameras):
        assert np.allclose(a["t_body_from_camera_m"], b["t_body_from_camera_m"], atol=1e-5)
        assert np.allclose(a["R_body_from_camera"], b["R_body_from_camera"], atol=1e-5)
