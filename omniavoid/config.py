from __future__ import annotations

import json
from pathlib import Path
import numpy as np
from .geometry import Camera


def load_config(path):
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_config(cfg)
    return cfg


def validate_config(cfg):
    if cfg.get("schema_version") != 1:
        raise ValueError("Expected schema_version=1")
    cameras = [Camera.from_dict(c) for c in cfg["cameras"]]
    names = [c.name for c in cameras]
    if len(names) != 4 or len(set(names)) != 4:
        raise ValueError("This baseline requires exactly four uniquely named cameras")
    for p in cfg["pairs"]:
        if p[0] not in names or p[1] not in names or p[0] == p[1]:
            raise ValueError("Invalid camera pair")
    st, rg, tm = cfg["stereo"], cfg["range"], cfg["timing"]
    if st["num_disparities"] < 16 or st["num_disparities"] % 16:
        raise ValueError("num_disparities must be positive and divisible by 16")
    if st["width"] <= st["num_disparities"] + 2 * st["block_size"]:
        raise ValueError("Virtual image too narrow for selected disparity range")
    if not 0 < st["horizontal_fov_deg"] < 170:
        raise ValueError("Virtual pinhole FOV must be below 170 degrees")
    if st["block_size"] < 3 or st["block_size"] % 2 != 1:
        raise ValueError("block_size must be odd and >= 3")
    if not 0 <= rg["min_m"] < rg["max_m"] <= 650:
        raise ValueError("Invalid range limits")
    if not rg["body_z_min_m"] < rg["body_z_max_m"]:
        raise ValueError("Invalid body z slab")
    if not 0 < tm["max_skew_ms"] < tm["max_age_ms"] < 2000:
        raise ValueError("Invalid timing limits")
    if not 0 < tm["publish_hz"] <= 100 or not 0 < rg["min_support"] <= 1000:
        raise ValueError("Invalid publishing rate or sample support")
    if not 0 <= cfg["mavlink"]["angle_offset_deg"] < 360:
        raise ValueError("angle_offset_deg must be within [0,360)")
    def finite(item):
        if isinstance(item, dict):
            return all(finite(v) for v in item.values())
        if isinstance(item, list):
            return all(finite(v) for v in item)
        return not isinstance(item, (float, int)) or np.isfinite(item)
    if not finite(cfg):
        raise ValueError("Non-finite configuration value")
    return cfg


def live_transmit_errors(cfg, source_kind):
    errors = []
    if source_kind != "v4l2":
        errors.append("Only live V4L2 input may transmit; synthetic/replay transmission is blocked")
    for key in ("validated", "body_alignment_verified", "occlusion_masks_verified"):
        if cfg.get("calibration", {}).get(key) is not True:
            errors.append(f"calibration.{key} must be verified on the physical rig")
    for key in ("hardware_sync_verified", "exposure_timestamp_verified"):
        if cfg["capture"].get(key) is not True:
            errors.append(f"capture.{key} must be verified on the physical rig")
    if cfg.get("calibration", {}).get("synthetic", True):
        errors.append("Synthetic/example calibration cannot transmit")
    return errors
