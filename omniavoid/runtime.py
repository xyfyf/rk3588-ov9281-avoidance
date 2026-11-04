from __future__ import annotations

import json
from pathlib import Path
import time
import cv2
import numpy as np
from .config import live_transmit_errors
from .dataset import read_dataset
from .geometry import Camera, project
from .protocol import Publisher, Snapshot, SnapshotStore, Transport
from .report import write_report
from .sectors import UNKNOWN, aggregate
from .stereo import StereoPipeline


def process_images(pipeline, cfg, images):
    if set(images) != {c["name"] for c in cfg["cameras"]}:
        raise ValueError("Every frame group must contain exactly the four configured cameras")
    start = time.perf_counter()
    points, sigmas, stereo_ms = pipeline.compute(images)
    distances, counts = aggregate(points, sigmas, cfg["range"], cfg["mavlink"]["angle_offset_deg"])
    result = {"distances_cm": distances.astype(int).tolist(), "support_counts": counts.tolist(),
              "valid_sectors": int(np.sum(distances != UNKNOWN)), "point_count": len(points),
              "processing_ms": (time.perf_counter() - start) * 1000, "stereo_ms": stereo_ms,
              "frame": "BODY_FRD", "increment_deg": 5, "angle_offset_deg": cfg["mavlink"]["angle_offset_deg"],
              "unknown_value": UNKNOWN, "free_space_claimed": False}
    return result, points, sigmas


def replay(cfg, manifest, output, limit=0, html_path=None, cloud_path=None):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    pipeline, first = StereoPipeline(cfg), None
    try:
        with output.open("w", encoding="utf-8") as stream:
            for index, (row, images) in enumerate(read_dataset(manifest)):
                if limit and index >= limit:
                    break
                if row.get("skew_ns", 0) > cfg["timing"]["max_skew_ms"] * 1e6:
                    result = {"distances_cm": [UNKNOWN] * 72, "support_counts": [0] * 72,
                              "valid_sectors": 0, "point_count": 0, "processing_ms": 0, "health": "excessive_skew"}
                    points, sigmas = np.empty((0, 3)), np.empty(0)
                else:
                    result, points, sigmas = process_images(pipeline, cfg, images)
                    result["health"] = "ok" if result["valid_sectors"] else "no_valid_depth"
                result.update({"sequence": row.get("sequence", index), "dataset_timestamp_ns": row["timestamp_ns"], "source": "offline_replay", "synthetic": row["synthetic"]})
                stream.write(json.dumps(result) + "\n")
                if first is None:
                    first = result
                    if cloud_path:
                        np.savez_compressed(cloud_path, points_body_m=points, range_sigma_m=sigmas)
                    if html_path:
                        write_report(html_path, result)
    finally:
        pipeline.close()
    if first is None:
        raise ValueError("Dataset is empty")
    return first


def run_live(cfg, output, transmit=False, duration_s=0):
    from .v4l2 import CaptureRig
    if transmit:
        errors = live_transmit_errors(cfg, "v4l2")
        if errors:
            raise ValueError("Transmission blocked:\n" + "\n".join(errors))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    store = SnapshotStore()
    rig = pipeline = publisher = None
    try:
        pipeline = StereoPipeline(cfg)
        if transmit:
            publisher = Publisher(cfg, store, Transport(cfg["mavlink"]["endpoint"], cfg["mavlink"]["baud"]))
            publisher.start()
        rig = CaptureRig(cfg)
        start, last_status, sequence = time.monotonic(), 0.0, 0
        with output.open("w", encoding="utf-8") as stream:
            while duration_s <= 0 or time.monotonic() - start < duration_s:
                if publisher and publisher.error:
                    raise RuntimeError(f"MAVLink publisher failed: {publisher.error}")
                group = rig.next()
                if group:
                    capture = min(f.monotonic_ns for f in group.values())
                    epoch = min(f.epoch_ns for f in group.values())
                    result, _, _ = process_images(pipeline, cfg, {n: f.image for n, f in group.items()})
                    age_ms = (time.monotonic_ns() - capture) / 1e6
                    if age_ms > cfg["timing"]["max_age_ms"] or age_ms < 0:
                        store.invalidate("processing_deadline_missed")
                        result["distances_cm"], result["valid_sectors"] = [UNKNOWN] * 72, 0
                        health = "processing_deadline_missed"
                    else:
                        store.update(Snapshot(capture, epoch, tuple(result["distances_cm"]), result["processing_ms"]))
                        health = "ok" if result["valid_sectors"] else "no_valid_depth"
                    result.update({"sequence": sequence, "capture_epoch_ns": epoch, "age_ms": age_ms,
                                   "skew_ms": (max(f.monotonic_ns for f in group.values()) - capture) / 1e6,
                                   "camera_sequences": {n: f.sequence for n, f in group.items()}, "health": health,
                                   "camera_timestamp_flags": {n: f.timestamp_flags for n, f in group.items()},
                                   "transmitting": transmit, "source": "live_v4l2"})
                    stream.write(json.dumps(result) + "\n")
                    stream.flush()
                    sequence += 1
                if time.monotonic() - last_status > 1:
                    _, health = store.read(time.monotonic_ns(), cfg["timing"]["max_age_ms"])
                    print(json.dumps({"health": health, "processed_groups": sequence,
                                      "rejected_frames": rig.sync.rejected, "discarded_frames": rig.sync.discarded}), flush=True)
                    last_status = time.monotonic()
    finally:
        store.invalidate("shutdown")
        if publisher:
            publisher.close()
        if rig:
            rig.close()
        if pipeline:
            pipeline.close()


def record(cfg, directory, samples=100, interval_s=0.3, timeout_s=120):
    from .v4l2 import CaptureRig
    directory = Path(directory)
    if directory.exists() and any(directory.iterdir()):
        raise ValueError("Recording directory must be empty")
    directory.mkdir(parents=True, exist_ok=True)
    manifest = {"schema_version": 1, "synthetic": False, "frames": [],
                "hardware_sync_verified": cfg["capture"]["hardware_sync_verified"],
                "exposure_timestamp_verified": cfg["capture"]["exposure_timestamp_verified"]}
    rig, begin, last = CaptureRig(cfg), time.monotonic(), 0.0
    try:
        while len(manifest["frames"]) < samples:
            if time.monotonic() - begin > timeout_s:
                raise TimeoutError("Recording deadline reached; inspect capture nodes and synchronization")
            group = rig.next()
            if not group or time.monotonic() - last < interval_s:
                continue
            sequence = len(manifest["frames"])
            images = {}
            for name, frame in group.items():
                filename = f"{sequence:05d}_{name}.png"
                if not cv2.imwrite(str(directory / filename), frame.image):
                    raise IOError("Could not save capture")
                images[name] = filename
            timestamps = [f.monotonic_ns for f in group.values()]
            manifest["frames"].append({"sequence": sequence, "timestamp_ns": min(timestamps),
                                       "skew_ns": max(timestamps) - min(timestamps), "images": images,
                                       "camera_sequences": {n: f.sequence for n, f in group.items()},
                                       "camera_timestamp_flags": {n: f.timestamp_flags for n, f in group.items()}})
            last = time.monotonic()
            print(f"recorded {sequence + 1}/{samples}", flush=True)
    finally:
        rig.close()
        (directory / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return directory / "manifest.json"


def geometric_coverage(cfg, distance_m=3.0):
    cameras = {d["name"]: Camera.from_dict(d) for d in cfg["cameras"]}
    angles = np.deg2rad(np.arange(72) * 5 + cfg["mavlink"]["angle_offset_deg"])
    points = distance_m * np.column_stack((np.cos(angles), np.sin(angles), np.zeros(72)))
    seen = {}
    for name, c in cameras.items():
        uv, seen[name] = project(c, (points - c.t) @ c.R, cfg["stereo"]["rim_guard_deg"])
        if c.valid_mask_path:
            mask = cv2.imread(c.valid_mask_path, cv2.IMREAD_GRAYSCALE)
            if mask is None or mask.shape != (c.height, c.width):
                raise ValueError("Invalid valid_mask_path")
            ix = np.clip(np.rint(uv[:, 0]).astype(int), 0, c.width - 1)
            iy = np.clip(np.rint(uv[:, 1]).astype(int), 0, c.height - 1)
            seen[name] &= mask[iy, ix] >= 254
    paired = np.zeros(72, dtype=bool)
    for a, b in cfg["pairs"]:
        paired |= seen[a] & seen[b]
    return {"distance_m": distance_m, "geometric_paired_sectors": int(paired.sum()),
            "uncovered_sector_centres_deg": ((np.flatnonzero(~paired) * 5 + cfg["mavlink"]["angle_offset_deg"]) % 360).tolist(),
            "note": "Optical overlap only, ignores stereo search borders, textures, uncertainty, occlusion and detection reliability."}
