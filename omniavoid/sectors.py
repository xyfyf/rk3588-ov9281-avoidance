from __future__ import annotations

import numpy as np

UNKNOWN = 65535
SECTORS = 72
INCREMENT_DEG = 5


def sector_indices(points, angle_offset_deg=0.0):
    angles = np.rad2deg(np.arctan2(points[:, 1], points[:, 0]))
    # Index 0 is centred on angle_offset; positive rotation is clockwise in FRD.
    return np.floor(((angles - angle_offset_deg + 2.5) % 360) / 5).astype(np.int32)


def aggregate(points, sigmas, cfg, angle_offset_deg=0.0):
    distances = np.full(SECTORS, UNKNOWN, dtype=np.uint16)
    counts = np.zeros(SECTORS, dtype=np.int32)
    points, sigmas = np.asarray(points), np.asarray(sigmas)
    if len(points) == 0:
        return distances, counts
    radius = np.hypot(points[:, 0], points[:, 1])
    keep = (np.isfinite(points).all(axis=1) & np.isfinite(sigmas) & (sigmas >= 0) &
            (radius >= cfg["min_m"]) & (radius <= cfg["max_m"]) &
            (points[:, 2] >= cfg["body_z_min_m"]) & (points[:, 2] <= cfg["body_z_max_m"]))
    indices = sector_indices(points[keep], angle_offset_deg)
    radius, sigmas = radius[keep], sigmas[keep]
    for sector in range(SECTORS):
        where = np.flatnonzero(indices == sector)
        counts[sector] = len(where)
        if len(where) < cfg["min_support"]:
            continue
        order = where[np.argsort(radius[where])]
        n = cfg["min_support"]
        # Require a small radial group, then choose its nearest conservative bound.
        # This can reject very thin obstacles; it is not a detection guarantee.
        span = radius[order[n - 1:]] - radius[order[:len(order) - n + 1]]
        supported = np.flatnonzero(span <= cfg["support_span_m"])
        if len(supported) == 0:
            continue
        # An unconfirmed closer return could be a thin foreground object.
        # Do not replace it with a confident-looking background distance.
        if (cfg.get("reject_unconfirmed_foreground", True) and supported[0] > 0 and
                radius[order[supported[0]]] - radius[order[0]] > cfg["support_span_m"]):
            continue
        cluster = order[supported[0]:supported[0] + n]
        lower_bound = np.min(radius[cluster] - cfg["uncertainty_sigma"] * sigmas[cluster])
        # Zero means contact. Never wrap negative ranges into uint16.
        cm = max(0, int(np.floor(lower_bound * 100)))
        distances[sector] = min(cm, int(cfg["max_m"] * 100))
    inflation = cfg.get("angular_inflation_radius_m", 0.0)
    if inflation > 0:
        original = distances.copy()
        for i in np.flatnonzero(original != UNKNOWN):
            half_angle = np.rad2deg(np.arctan2(inflation, max(float(original[i]) / 100, 0.01)))
            neighbours = min(36, int(np.ceil(half_angle / 5)))
            for offset in range(-neighbours, neighbours + 1):
                j = (i + offset) % SECTORS
                distances[j] = min(distances[j], original[i])
    return distances, counts


def advisory_speed_mps(distance_m, delay_s, deceleration_mps2, margin_m):
    """Static-obstacle stopping estimate, not a controller or flight speed rating."""
    if not np.isfinite([distance_m, delay_s, deceleration_mps2, margin_m]).all():
        return 0.0
    if delay_s < 0 or deceleration_mps2 <= 0 or margin_m < 0 or distance_m <= margin_m:
        return 0.0
    at = deceleration_mps2 * delay_s
    return float(np.sqrt(at * at + 2 * deceleration_mps2 * (distance_m - margin_m)) - at)
