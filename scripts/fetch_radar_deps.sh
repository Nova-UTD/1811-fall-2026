#!/usr/bin/env bash
# Download Smart Access binaries required by the smartmicro radar driver.
# The git submodule only has the ROS packages; the vendor libs are fetched
# separately (license prompt) and are gitignored inside the submodule.
#
# Run once after: git submodule update --init --recursive
# Safe to re-run: skips if libs are already present.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RADAR_DIR="$ROOT/ros2_ws/src/smartmicro_ros2_radars"
MARKER="$RADAR_DIR/umrr_ros2_driver/smartmicro/lib-linux-x86_64-gcc_9"

if [ ! -d "$RADAR_DIR" ] || [ ! -f "$RADAR_DIR/smart_extract.sh" ]; then
  echo "smartmicro_ros2_radars submodule missing."
  echo "From the repo root run:"
  echo "  git submodule update --init --recursive"
  exit 1
fi

if [ -d "$MARKER" ]; then
  echo "Smart Access libs already present under umrr_ros2_driver/smartmicro/ — skipping."
  exit 0
fi

echo "Fetching Smart Access Automotive binaries (accepts Apache 2.0 license)..."
cd "$RADAR_DIR"
# smart_extract.sh prompts yes/no for the Apache 2.0 license text it prints.
printf 'yes\n' | ./smart_extract.sh

if [ ! -d "$MARKER" ]; then
  echo "Extract finished but expected libs were not found at:"
  echo "  $MARKER"
  echo "Check network access to smartmicro.com and re-run."
  exit 1
fi

echo "Smart Access libs ready."
