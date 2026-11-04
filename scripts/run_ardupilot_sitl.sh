#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHON="${PYTHON:-python3}"
SITL_BINARY="${SITL_BINARY:-/home/eft/ardupilot/build/sitl/bin/arducopter}"
ARDUPILOT_REPO="${ARDUPILOT_REPO:-/home/eft/ardupilot}"
OUT="${1:-examples/ardupilot_sitl_$(date +%Y%m%d_%H%M%S)}"
# Starts a local owned SITL subprocess; existing output EEPROM is never reused.
for SCENARIO in reception avoid_off avoid_on avoid_dropout visual_on; do
  "$PYTHON" tools/ardupilot_sitl_test.py --binary "$SITL_BINARY" --repo "$ARDUPILOT_REPO" \
    --scenario "$SCENARIO" --output "$OUT/${SCENARIO}_01"
done
"$PYTHON" tools/sitl_report.py --root "$OUT" --output "$OUT/sitl.html"
printf 'Report: %s/sitl.html\n' "$OUT"
