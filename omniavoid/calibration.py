"""Identified ArUco grid observations + full-angle KB4 bundle adjustment.

Output is a candidate calibration. Physical scale, body alignment, outer-FOV
support, synchronization, and obstacle ranging must be independently validated.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.sparse import lil_matrix
from .geometry import Camera, project_kb4, radial_derivative, unproject

BOARD = {"columns": 7, "rows": 5, "marker_length_m": 0.03,
         "separation_m": 0.0075, "dictionary": "DICT_4X4_100"}


def make_board(spec=None):
    spec = spec or BOARD
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("ArUco requires OpenCV contrib (Ubuntu python3-opencv or opencv-contrib-python-headless)")
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, spec["dictionary"]))
    args = ((spec["columns"], spec["rows"]), spec["marker_length_m"], spec["separation_m"], dictionary)
    if hasattr(aruco, "GridBoard"):
        board = aruco.GridBoard(*args)
        ids, objects = board.getIds().ravel(), board.getObjPoints()
    else:
        board = aruco.GridBoard_create(spec["columns"], spec["rows"], *args[1:])
        ids, objects = board.ids.ravel(), board.objPoints
    return board, dictionary, {int(i): np.asarray(p, dtype=float) for i, p in zip(ids, objects)}


def write_board(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    board, _, _ = make_board()
    width = BOARD["columns"] * BOARD["marker_length_m"] + (BOARD["columns"] - 1) * BOARD["separation_m"]
    height = BOARD["rows"] * BOARD["marker_length_m"] + (BOARD["rows"] - 1) * BOARD["separation_m"]
    pixels_per_m, margin_m = 10000, 0.01
    dimensions = (round((width + 2 * margin_m) * pixels_per_m), round((height + 2 * margin_m) * pixels_per_m))
    margin = round(margin_m * pixels_per_m)
    image = board.generateImage(dimensions, marginSize=margin) if hasattr(board, "generateImage") else board.draw(dimensions, marginSize=margin)
    if not cv2.imwrite(str(output / "aruco_grid.png"), image):
        raise IOError("Could not write board PNG")
    spec = {**BOARD, "print_width_mm": (width + 2 * margin_m) * 1000,
            "print_height_mm": (height + 2 * margin_m) * 1000,
            "note": "Print at these exact dimensions, disable fit-to-page, and measure several markers with a ruler."}
    (output / "board.json").write_text(json.dumps(spec, indent=2), encoding="utf-8")
    return spec


def detect_grid(image, dictionary, object_lookup):
    aruco = cv2.aruco
    parameters = aruco.DetectorParameters() if hasattr(aruco, "DetectorParameters") else aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    if hasattr(aruco, "ArucoDetector"):
        corners, ids, _ = aruco.ArucoDetector(dictionary, parameters).detectMarkers(image)
    else:
        corners, ids, _ = aruco.detectMarkers(image, dictionary, parameters=parameters)
    objects, pixels = [], []
    if ids is not None:
        for points, marker_id in zip(corners, ids.ravel()):
            if int(marker_id) in object_lookup:
                objects.extend(object_lookup[int(marker_id)])
                pixels.extend(np.asarray(points).reshape(4, 2))
    return np.asarray(objects, dtype=float).reshape(-1, 3), np.asarray(pixels, dtype=float).reshape(-1, 2)


def seed_pose(camera, objects, pixels):
    rays, valid = unproject(camera, pixels)
    valid &= rays[:, 2] > 0.15
    if valid.sum() < 8:
        raise ValueError("Board view has insufficient forward-facing corners to initialize; include more central views")
    normalized = rays[valid, :2] / rays[valid, 2:]
    ok, r, t = cv2.solvePnP(objects[valid], normalized, np.eye(3), np.zeros(4), flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError("Could not initialize board pose")
    return np.r_[r.ravel(), t.ravel()]


def fit_intrinsics(camera_dict, observations, min_views=12):
    camera = Camera.from_dict(camera_dict)
    seeded = []
    for observation in observations:
        try:
            seeded.append((observation, seed_pose(camera, observation["objects"], observation["pixels"])))
        except (ValueError, cv2.error):
            continue
    if len(seeded) < min_views:
        raise ValueError(f"{camera.name}: need >= {min_views} usable board poses, got {len(seeded)}")
    initial = np.r_[[camera.fx, camera.fy, camera.cx, camera.cy], camera.distortion,
                    np.concatenate([pose for _, pose in seeded])]
    theta_samples = np.linspace(0, np.deg2rad(camera.max_theta_deg), 32)

    def residual(x):
        result = []
        for i, (observation, _) in enumerate(seeded):
            pose = x[8 + 6 * i:14 + 6 * i]
            p = observation["objects"] @ Rotation.from_rotvec(pose[:3]).as_matrix().T + pose[3:]
            uv, _ = project_kb4(p, x[:4], x[4:8])
            result.extend((uv - observation["pixels"]).ravel())
        result.extend(100 * np.minimum(0, radial_derivative(theta_samples, x[4:8]) - 0.1))
        return np.asarray(result)

    rows = sum(len(o["pixels"]) * 2 for o, _ in seeded) + len(theta_samples)
    sparse = lil_matrix((rows, len(initial)), dtype=int)
    offset = 0
    for i, (observation, _) in enumerate(seeded):
        n = len(observation["pixels"]) * 2
        sparse[offset:offset+n, :8] = 1
        sparse[offset:offset+n, 8 + i * 6:14 + i * 6] = 1
        offset += n
    sparse[offset:, 4:8] = 1
    lower = np.r_[[0.1 * camera.width, 0.1 * camera.height, 0.1 * camera.width, 0.1 * camera.height], [-0.5] * 4, [-np.inf] * (len(initial) - 8)]
    upper = np.r_[[3 * camera.width, 3 * camera.height, 0.9 * camera.width, 0.9 * camera.height], [0.5] * 4, [np.inf] * (len(initial) - 8)]
    fit = least_squares(residual, initial, bounds=(lower, upper), jac_sparsity=sparse.tocsr(),
                        loss="soft_l1", f_scale=1.0, x_scale="jac", max_nfev=600, ftol=1e-9)
    if not fit.success:
        raise ValueError(f"{camera.name}: intrinsic solver did not converge: {fit.message}")
    result = copy.deepcopy(camera_dict)
    result["intrinsics"], result["distortion"] = fit.x[:4].tolist(), fit.x[4:8].tolist()
    poses, errors, theta_max = {}, [], 0.0
    for i, (observation, _) in enumerate(seeded):
        pose = fit.x[8 + i * 6:14 + i * 6]
        p = observation["objects"] @ Rotation.from_rotvec(pose[:3]).as_matrix().T + pose[3:]
        uv, theta = project_kb4(p, fit.x[:4], fit.x[4:8])
        error = np.linalg.norm(uv - observation["pixels"], axis=1)
        if np.median(error) > 2.0:
            continue
        poses[observation["group"]] = pose
        errors.extend(error)
        theta_max = max(theta_max, float(np.rad2deg(theta.max())))
    if len(poses) < min_views:
        raise ValueError(f"{camera.name}: too many bad board observations after fit")
    # Never grant the nominal lens rim as calibrated if no board corners reached it.
    result["max_theta_deg"] = min(camera.max_theta_deg, theta_max)
    Camera.from_dict(result)
    report = {"views": len(poses), "rms_px": float(np.sqrt(np.mean(np.square(errors)))),
              "p95_px": float(np.percentile(errors, 95)), "observed_max_theta_deg": theta_max,
              "note": "Scalar angular extent alone does not prove uniform rim coverage; inspect every azimuth."}
    return result, poses, report


def pose_matrix(pose):
    return Rotation.from_rotvec(pose[:3]).as_matrix(), np.asarray(pose[3:])


def fit_extrinsics(cameras, observations, poses):
    """Fix first camera as body anchor, solve other cameras and board poses jointly."""
    names = [c["name"] for c in cameras]
    root = names[0]
    root_camera = Camera.from_dict(cameras[0])
    body = {root: (root_camera.R, root_camera.t)}
    board = {}
    # Shared, uniquely identified board observations connect the rig graph.
    while True:
        changed = False
        for name in names:
            for group, pose in poses[name].items():
                rcb, tcb = pose_matrix(pose)
                if name in body and group not in board:
                    rbc, tbc = body[name]
                    board[group] = (rbc @ rcb, rbc @ tcb + tbc)
                    changed = True
                if group in board and name not in body:
                    rbb, tbb = board[group]
                    rbc = rbb @ rcb.T
                    body[name] = (rbc, tbb - rbc @ tcb)
                    changed = True
        if not changed:
            break
    if len(body) != len(names):
        raise ValueError("Disconnected camera graph: capture static board poses simultaneously visible in each adjacent pair")
    groups = sorted(board)
    camera_offsets = {n: i * 6 for i, n in enumerate(names[1:])}
    board_offsets = {g: 6 * (len(names) - 1 + i) for i, g in enumerate(groups)}
    values = [np.r_[Rotation.from_matrix(body[n][0]).as_rotvec(), body[n][1]] for n in names[1:]]
    values += [np.r_[Rotation.from_matrix(board[g][0]).as_rotvec(), board[g][1]] for g in groups]
    initial = np.concatenate(values)
    intrinsics = {c["name"]: c for c in cameras}
    used = [o for o in observations if o["group"] in board and o["group"] in poses[o["camera"]]]

    def unpack(x):
        b = {root: (root_camera.R, root_camera.t)}
        b.update({n: pose_matrix(x[i:i+6]) for n, i in camera_offsets.items()})
        return b, {g: pose_matrix(x[i:i+6]) for g, i in board_offsets.items()}

    def residual(x):
        b, boards = unpack(x)
        result = []
        for o in used:
            rbc, tbc = b[o["camera"]]
            rbb, tbb = boards[o["group"]]
            pbody = o["objects"] @ rbb.T + tbb
            pcam = (pbody - tbc) @ rbc
            c = intrinsics[o["camera"]]
            uv, _ = project_kb4(pcam, c["intrinsics"], c["distortion"])
            result.extend((uv - o["pixels"]).ravel())
        return np.asarray(result)

    sparse = lil_matrix((sum(len(o["pixels"]) * 2 for o in used), len(initial)), dtype=int)
    row = 0
    for o in used:
        n = len(o["pixels"]) * 2
        if o["camera"] != root:
            i = camera_offsets[o["camera"]]
            sparse[row:row+n, i:i+6] = 1
        i = board_offsets[o["group"]]
        sparse[row:row+n, i:i+6] = 1
        row += n
    fit = least_squares(residual, initial, jac_sparsity=sparse.tocsr(), loss="soft_l1",
                        f_scale=1, x_scale="jac", max_nfev=600, ftol=1e-9)
    if not fit.success:
        raise ValueError("Extrinsic solver did not converge: " + fit.message)
    fitted, _ = unpack(fit.x)
    result = copy.deepcopy(cameras)
    for c in result:
        c["R_body_from_camera"], c["t_body_from_camera_m"] = [v.tolist() for v in fitted[c["name"]]]
    error = residual(fit.x).reshape(-1, 2)
    return result, {"rms_px": float(np.sqrt(np.mean(np.sum(error * error, axis=1)))),
                    "groups": len(groups), "observations": len(used)}


def calibrate_dataset(cfg, manifest_path, output, min_views=12):
    manifest_path = Path(manifest_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    spec = manifest.get("board", BOARD)
    _, dictionary, lookup = make_board(spec)
    observations = []
    sizes = {d["name"]: (d["height"], d["width"]) for d in cfg["cameras"]}
    for index, group in enumerate(manifest["frames"]):
        for name, path in group["images"].items():
            image = cv2.imread(str(manifest_path.parent / path), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != sizes[name]:
                raise ValueError(f"Invalid calibration image: {path}")
            objects, pixels = detect_grid(image, dictionary, lookup)
            if len(objects) >= 12:
                observations.append({"group": index, "camera": name, "objects": objects, "pixels": pixels})
    fitted, poses, reports = [], {}, {}
    for camera in cfg["cameras"]:
        c, p, r = fit_intrinsics(camera, [o for o in observations if o["camera"] == camera["name"]], min_views)
        fitted.append(c)
        poses[camera["name"]], reports[camera["name"]] = p, r
    fitted, extrinsic_report = fit_extrinsics(fitted, observations, poses)
    result = copy.deepcopy(cfg)
    result["cameras"] = fitted
    result["calibration"] = {"validated": False, "synthetic": False, "body_alignment_verified": False, "occlusion_masks_verified": False,
                             "notes": "Candidate only. First camera body pose is a user-supplied anchor, not an IMU calibration.",
                             "board": spec, "report": {"intrinsics": reports, "extrinsics": extrinsic_report}}
    Path(output).write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result["calibration"]["report"]
