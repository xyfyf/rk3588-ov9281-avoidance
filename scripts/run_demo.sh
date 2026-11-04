#!/usr/bin/env bash
set -euo pipefail
package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$package_root"
.venv/bin/python -m omniavoid demo --output data/synthetic_demo
.venv/bin/python -m omniavoid benchmark --config data/synthetic_demo/rig.synthetic.json \
  --manifest data/synthetic_demo/manifest.json --iterations 20 --output data/synthetic_demo/benchmark.json
