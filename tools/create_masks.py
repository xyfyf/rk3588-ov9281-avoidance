"""Create lens-domain mask templates. Paint every real obstruction black."""
import argparse
from pathlib import Path
import cv2
import numpy as np
from omniavoid.config import load_config
from omniavoid.geometry import Camera, unproject


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", default="masks")
    args = parser.parse_args()
    cfg = load_config(args.config)
    directory = Path(args.output)
    directory.mkdir(parents=True, exist_ok=True)
    for d in cfg["cameras"]:
        camera = Camera.from_dict(d)
        u, v = np.meshgrid(np.arange(camera.width), np.arange(camera.height))
        rays, valid = unproject(camera, np.stack((u, v), axis=-1))
        theta = np.rad2deg(np.arccos(np.clip(rays[..., 2], -1, 1)))
        valid &= theta <= camera.max_theta_deg - cfg["stereo"]["rim_guard_deg"]
        path = directory / (camera.name + ".png")
        if path.exists():
            raise ValueError("Refusing to overwrite an existing edited mask: " + str(path))
        if not cv2.imwrite(str(path), valid.astype(np.uint8) * 255):
            raise IOError("Could not save mask")
    print("Templates only. Paint mount/airframe/propeller/poor-quality pixels black; set each camera.valid_mask_path relative to project root.")


if __name__ == "__main__":
    main()
