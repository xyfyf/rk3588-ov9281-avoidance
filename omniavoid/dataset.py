from __future__ import annotations

import copy
import json
from pathlib import Path
import cv2
import numpy as np
from .geometry import Camera, unproject


def demo_config(cfg, width=640, height=400):
    cfg = copy.deepcopy(cfg)
    cfg["calibration"] = {"validated": False, "synthetic": True, "body_alignment_verified": False, "occlusion_masks_verified": False,
                          "notes": "Synthetic equidistant cameras; no physical camera calibration"}
    for c in cfg["cameras"]:
        sx, sy = width / c["width"], height / c["height"]
        fx, fy, cx, cy = c["intrinsics"]
        c["intrinsics"] = [fx * sx, fy * sy, (cx + 0.5) * sx - 0.5, (cy + 0.5) * sy - 0.5]
        c["width"], c["height"] = width, height
    return cfg


def scene_surfaces():
    # axis, coordinate, bounds on remaining axes; all values are metres in FRD.
    surfaces = []
    bounds = [(-3.5, 3.0), (-2.8, 2.5), (-1.8, 1.8)]
    for axis in range(3):
        for coordinate in bounds[axis]:
            surfaces.append((axis, coordinate, [bounds[i] for i in range(3) if i != axis]))
    obstacle = [(1.4, 1.8), (-0.35, 0.35), (-0.6, 0.6)]
    for axis in range(3):
        for coordinate in obstacle[axis]:
            surfaces.append((axis, coordinate, [obstacle[i] for i in range(3) if i != axis]))
    return surfaces


def intersect_scene(origin, directions):
    distance = np.full(directions.shape[:-1], np.inf)
    surface_index = np.full(distance.shape, -1, dtype=int)
    for index, (axis, value, bounds) in enumerate(scene_surfaces()):
        denominator = directions[..., axis]
        safe = np.abs(denominator) > 1e-10
        t = np.divide(value - origin[axis], denominator, out=np.full_like(distance, np.inf), where=safe)
        other_axes = [i for i in range(3) if i != axis]
        valid = safe & (t > 0)
        for other_axis, (lo, hi) in zip(other_axes, bounds):
            coordinate = origin[other_axis] + np.where(safe, t, 0) * directions[..., other_axis]
            valid &= (coordinate >= lo) & (coordinate <= hi)
        update = valid & (t < distance)
        distance[update], surface_index[update] = t[update], index
    return distance, surface_index


def render_camera(camera):
    u, v = np.meshgrid(np.arange(camera.width), np.arange(camera.height))
    rays, valid = unproject(camera, np.stack((u, v), axis=-1))
    directions = rays @ camera.R.T
    distance, surface_index = intersect_scene(camera.t, directions)
    hit = camera.t + np.where(np.isfinite(distance), distance, 0)[..., None] * directions
    image = np.zeros((camera.height, camera.width), dtype=np.uint8)
    for index, (axis, _, _) in enumerate(scene_surfaces()):
        mask = (surface_index == index) & valid
        if not mask.any():
            continue
        axes = [i for i in range(3) if i != axis]
        rng = np.random.default_rng(1000 + index)
        texture = rng.integers(25, 230, size=(512, 512), dtype=np.uint8)
        texture = cv2.GaussianBlur(texture, (3, 3), 0.5)
        # Stable world-space texture, shared by every camera.
        mx = np.remainder(hit[..., axes[0]] * 100 + 256, 510).astype(np.float32)
        my = np.remainder(hit[..., axes[1]] * 100 + 256, 510).astype(np.float32)
        material = cv2.remap(texture, mx, my, cv2.INTER_LINEAR)
        image[mask] = material[mask]
    return image


def generate_demo(cfg, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    cfg = demo_config(cfg)
    (output / "rig.synthetic.json").write_text(json.dumps(cfg, indent=2), encoding="utf-8")
    images = {}
    for d in cfg["cameras"]:
        camera = Camera.from_dict(d)
        filename = camera.name + ".png"
        if not cv2.imwrite(str(output / filename), render_camera(camera)):
            raise IOError("Could not write demo image")
        images[camera.name] = filename
    manifest = {"schema_version": 1, "synthetic": True,
                "description": "Rendered static textured room and front box; not real camera footage",
                "frames": [{"sequence": 0, "timestamp_ns": 0, "skew_ns": 0, "images": images}]}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    angles = np.deg2rad(np.arange(72) * 5)
    rays = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(72)))
    distances, _ = intersect_scene(np.zeros(3), rays)
    truth = {"distance_m": distances.tolist(), "description": "Ray distance at exact sector centres. Sector minimum can be closer near box boundaries."}
    (output / "ground_truth.json").write_text(json.dumps(truth, indent=2), encoding="utf-8")
    return cfg, output / "manifest.json"


def read_dataset(path):
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    previous = -1
    for index, row in enumerate(data["frames"]):
        row = {**row, "synthetic": bool(data.get("synthetic", False))}
        stamp = int(row["timestamp_ns"])
        if stamp <= previous:
            raise ValueError("Dataset timestamps must increase strictly")
        previous = stamp
        images = {name: cv2.imread(str(path.parent / filename), cv2.IMREAD_GRAYSCALE)
                  for name, filename in row["images"].items()}
        if any(image is None for image in images.values()):
            raise ValueError(f"Dataset contains unreadable images at frame {index}")
        yield row, images
