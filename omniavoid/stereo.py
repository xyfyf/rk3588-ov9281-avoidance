from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
import cv2
import numpy as np
from .geometry import Camera, project, rectified_basis


class StereoPair:
    def __init__(self, left: Camera, right: Camera, cfg):
        self.left, self.right, self.cfg = left, right, cfg
        self.R, self.baseline = rectified_basis(left, right)
        self.width, self.height = cfg["width"], cfg["height"]
        self.f = (self.width - 1) / (2 * np.tan(np.deg2rad(cfg["horizontal_fov_deg"]) / 2))
        self.cx, self.cy = (self.width - 1) / 2, (self.height - 1) / 2
        self.u, self.v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        rays = np.stack(((self.u - self.cx) / self.f, (self.v - self.cy) / self.f,
                         np.ones_like(self.u)), axis=-1)
        self.maps = []
        for camera in (left, right):
            uv, valid = project(camera, rays @ self.R.T @ camera.R, cfg["rim_guard_deg"])
            if camera.valid_mask_path:
                mask = cv2.imread(camera.valid_mask_path, cv2.IMREAD_GRAYSCALE)
                if mask is None or mask.shape != (camera.height, camera.width):
                    raise ValueError(f"{camera.name}: missing or wrong-size valid_mask_path; paths are relative to working directory")
                sampled = cv2.remap(mask, uv[..., 0].astype(np.float32), uv[..., 1].astype(np.float32),
                                    cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
                valid &= sampled >= 254
            uv[~valid] = -1
            kernel = np.ones((cfg["block_size"] + 2, cfg["block_size"] + 2), np.uint8)
            valid = cv2.erode(valid.astype(np.uint8), kernel, borderType=cv2.BORDER_CONSTANT,
                              borderValue=0).astype(bool)
            self.maps.append((uv[..., 0].astype(np.float32), uv[..., 1].astype(np.float32), valid))
        common = dict(numDisparities=cfg["num_disparities"], blockSize=cfg["block_size"],
                      P1=8 * cfg["block_size"] ** 2, P2=32 * cfg["block_size"] ** 2,
                      disp12MaxDiff=-1, preFilterCap=31, uniquenessRatio=cfg["uniqueness_ratio"],
                      speckleWindowSize=cfg["speckle_window_size"], speckleRange=cfg["speckle_range"],
                      mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.right_min = -cfg["num_disparities"] + 1
        self.matcher_l = cv2.StereoSGBM_create(minDisparity=0, **common)
        self.matcher_r = cv2.StereoSGBM_create(minDisparity=self.right_min, **common)

    def rectify(self, images):
        out = []
        for camera, (mx, my, _) in zip((self.left, self.right), self.maps):
            image = images[camera.name]
            if image.shape != (camera.height, camera.width) or image.dtype != np.uint8:
                raise ValueError(f"{camera.name}: expected uint8 grayscale {camera.width}x{camera.height}; no implicit resizing")
            out.append(cv2.remap(image, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT))
        return out

    def texture_mask(self, image):
        a = image.astype(np.float32)
        mean = cv2.boxFilter(a, -1, (7, 7))
        var = cv2.boxFilter(a * a, -1, (7, 7)) - mean * mean
        return var >= self.cfg["texture_std_min"] ** 2

    def cloud_from_rectified(self, image_l, image_r):
        dl = self.matcher_l.compute(image_l, image_r).astype(np.float32) / 16.0
        dr = self.matcher_r.compute(image_r, image_l).astype(np.float32) / 16.0
        valid_l = self.maps[0][2] & self.texture_mask(image_l) & (dl > 0.5) & (dl < self.cfg["num_disparities"] - 1)
        valid_r = self.maps[1][2] & self.texture_mask(image_r) & (dr < -0.5) & (dr > self.right_min)
        points, sigmas = [], []
        stride = max(1, int(self.cfg["point_stride"]))
        sample_mask = (self.u % stride == 0) & (self.v % stride == 0)
        # Both reference images are reconstructed; this reduces unilateral border loss.
        for disparity, other, valid, other_valid, sign, camera in (
                (dl, dr, valid_l, valid_r, 1, self.left),
                (dr, dl, valid_r, valid_l, -1, self.right)):
            match_u = np.rint(self.u - disparity).astype(np.int32)
            inside = (match_u >= 0) & (match_u < self.width)
            ix = np.clip(match_u, 0, self.width - 1)
            consistent = (np.abs(disparity + other[self.v, ix]) <= self.cfg["lr_max_diff_px"])
            absolute = sign * disparity
            relative_sigma = self.cfg["disparity_sigma_px"] / np.maximum(absolute, 0.01)
            keep = (valid & inside & other_valid[self.v, ix] & consistent & sample_mask &
                    (relative_sigma <= self.cfg["max_relative_sigma"]))
            y, x = np.nonzero(keep)
            d = absolute[y, x]
            z = self.f * self.baseline / d
            p = np.column_stack(((x - self.cx) * z / self.f, (y - self.cy) * z / self.f, z))
            sigma = np.linalg.norm(p, axis=1) * self.cfg["disparity_sigma_px"] / d
            points.append(p @ self.R.T + camera.t)
            sigmas.append(sigma)
        return np.concatenate(points), np.concatenate(sigmas)

    def compute(self, images):
        return self.cloud_from_rectified(*self.rectify(images))


class StereoPipeline:
    def __init__(self, cfg):
        self.cfg = cfg
        cv2.setNumThreads(cfg["stereo"]["opencv_threads"])
        cameras = {d["name"]: Camera.from_dict(d) for d in cfg["cameras"]}
        self.pairs = [StereoPair(cameras[a], cameras[b], cfg["stereo"]) for a, b in cfg["pairs"]]
        workers = cfg["stereo"].get("pair_workers", 1)
        self.executor = ThreadPoolExecutor(max_workers=workers) if workers > 1 else None

    def compute(self, images):
        start = time.perf_counter()
        clouds = list(self.executor.map(lambda p: p.compute(images), self.pairs)) if self.executor else [p.compute(images) for p in self.pairs]
        return np.concatenate([p for p, _ in clouds]), np.concatenate([s for _, s in clouds]), (time.perf_counter() - start) * 1000

    def close(self):
        if self.executor:
            self.executor.shutdown(wait=True)
