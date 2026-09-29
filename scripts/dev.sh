#!/usr/bin/env bash
# Open a shell in the 1811 container -- run once per terminal you need.
# Starts the container if it isn't up, and builds the workspace the first time.
set -euo pipefail
cd "$(dirname "$0")/.."

docker compose up -d dev
# flock: a second terminal opened mid-build waits instead of building twice.
exec docker compose exec dev bash -ic \
  "flock /tmp/1811-build.lock bash -c '[ -f install/setup.bash ] || colcon build --symlink-install'; exec bash"
