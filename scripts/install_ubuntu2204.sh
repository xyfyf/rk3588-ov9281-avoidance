#!/usr/bin/env bash
set -euo pipefail
package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$package_root"
echo 'Installing user-space dependencies. This does not install camera drivers or change flight-controller parameters.'
sudo apt-get update
sudo apt-get install -y python3 python3-dev python3-venv python3-pip python3-numpy python3-scipy \
  python3-opencv python3-lxml python3-serial build-essential v4l-utils
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install 'pip>=22,<26' 'setuptools>=61,<76' wheel \
  'pymavlink>=2.4.41,<3' 'pyserial>=3.5,<4' 'pytest>=7,<9'
# OpenCV/NumPy/SciPy come from Ubuntu apt; do not shadow the BSP's OpenCV with pip wheels.
.venv/bin/python -m pip install --no-deps --no-build-isolation -e .
.venv/bin/python -c 'import cv2, numpy, scipy; assert hasattr(cv2, "aruco"); print("OpenCV", cv2.__version__, "NumPy", numpy.__version__, "SciPy", scipy.__version__)'
cc -std=c11 -Wall -Wextra -O2 tools/v4l2_abi_probe.c -o .venv/v4l2_abi_probe
.venv/v4l2_abi_probe
.venv/bin/python -m pytest -q
echo 'Ready. Run: .venv/bin/python -m omniavoid demo --output data/synthetic_demo'
