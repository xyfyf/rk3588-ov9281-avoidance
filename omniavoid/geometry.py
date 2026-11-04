"""Full-angle KB4 fisheye projection; angles above 90 degrees retain their sign.

Body: x forward, y right, z down. Camera: x right, y down, z optical forward.
All poses are body_from_camera; translations and reconstructed points are metres.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class Camera:
    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: np.ndarray
    max_theta_deg: float
    R: np.ndarray
    t: np.ndarray
    valid_mask_path: str | None = None

    @classmethod
    def from_dict(cls, d):
        cam = cls(d["name"], int(d["width"]), int(d["height"]),
                  *map(float, d["intrinsics"]),
                  np.asarray(d["distortion"], dtype=float), float(d["max_theta_deg"]),
                  np.asarray(d["R_body_from_camera"], dtype=float),
                  np.asarray(d["t_body_from_camera_m"], dtype=float), d.get("valid_mask_path"))
        if cam.width < 16 or cam.height < 16 or min(cam.fx, cam.fy) <= 0:
            raise ValueError("Invalid camera dimensions or focal length")
        if cam.distortion.shape != (4,) or cam.R.shape != (3, 3) or cam.t.shape != (3,):
            raise ValueError("KB4 requires four coefficients and a 3x3 rotation")
        if not np.allclose(cam.R.T @ cam.R, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(cam.R), 1, atol=1e-5):
            raise ValueError("Camera rotation must be a proper orthonormal rotation")
        values = [cam.fx, cam.fy, cam.cx, cam.cy, cam.max_theta_deg, *cam.distortion, *cam.R.ravel(), *cam.t]
        if not np.isfinite(values).all() or not 0 < cam.max_theta_deg < 175:
            raise ValueError("Invalid camera parameters")
        theta = np.linspace(0, np.deg2rad(cam.max_theta_deg), 256)
        if np.any(radial_derivative(theta, cam.distortion) < 0.05):
            raise ValueError("KB4 radial function is non-monotonic or ill-conditioned")
        return cam


def radial(theta, k):
    t2 = theta * theta
    return theta * (1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))))


def radial_derivative(theta, k):
    t2 = theta * theta
    return 1 + t2 * (3 * k[0] + t2 * (5 * k[1] + t2 * (7 * k[2] + t2 * 9 * k[3])))


def project_kb4(points, intrinsics, distortion):
    """Project rays or 3D camera points, including z < 0 (theta > pi/2)."""
    points = np.asarray(points, dtype=float)
    fx, fy, cx, cy = intrinsics
    xy = np.linalg.norm(points[..., :2], axis=-1)
    theta = np.arctan2(xy, points[..., 2])
    scale = radial(theta, distortion) / np.maximum(xy, 1e-12)
    return np.stack((cx + fx * points[..., 0] * scale,
                     cy + fy * points[..., 1] * scale), axis=-1), theta


def project(camera: Camera, rays, guard_deg=0.0):
    uv, theta = project_kb4(rays, (camera.fx, camera.fy, camera.cx, camera.cy), camera.distortion)
    valid = ((theta <= np.deg2rad(camera.max_theta_deg - guard_deg)) &
             (uv[..., 0] >= 1) & (uv[..., 0] < camera.width - 2) &
             (uv[..., 1] >= 1) & (uv[..., 1] < camera.height - 2) &
             (np.linalg.norm(rays, axis=-1) > 1e-10))
    return uv, valid


def unproject(camera: Camera, pixels):
    uv = (np.asarray(pixels, dtype=float) - [camera.cx, camera.cy]) / [camera.fx, camera.fy]
    radius = np.linalg.norm(uv, axis=-1)
    lo, hi = np.zeros_like(radius), np.full_like(radius, np.deg2rad(camera.max_theta_deg))
    valid = radius <= radial(hi, camera.distortion)
    # Bisection is bounded, deterministic, and robust near the fisheye rim.
    for _ in range(40):
        mid = (lo + hi) / 2
        lower = radial(mid, camera.distortion) < radius
        lo = np.where(lower, mid, lo)
        hi = np.where(lower, hi, mid)
    theta = (lo + hi) / 2
    xy = uv * (np.sin(theta) / np.maximum(radius, 1e-12))[..., None]
    rays = np.concatenate((xy, np.cos(theta)[..., None]), axis=-1)
    return rays, valid


def nominal_pose(yaw_deg, radius_m):
    a = np.deg2rad(yaw_deg)
    forward = np.array([np.cos(a), np.sin(a), 0.0])
    right = np.array([-np.sin(a), np.cos(a), 0.0])
    return np.column_stack((right, [0.0, 0.0, 1.0], forward)), radius_m * forward


def rectified_basis(left: Camera, right: Camera):
    baseline = right.t - left.t
    distance = float(np.linalg.norm(baseline))
    if distance < 0.01:
        raise ValueError("Stereo baseline must be >= 1 cm")
    x = baseline / distance
    forward = left.R[:, 2] + right.R[:, 2]
    z = forward - np.dot(forward, x) * x
    if np.linalg.norm(z) < 0.1:
        raise ValueError("Degenerate virtual camera direction; use adjacent cameras")
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    return np.column_stack((x, y, z)), distance
