#!/usr/bin/env bash
# Open a shell in the 1811 container -- run once per terminal you need.
# Starts the container if it isn't up, and builds the workspace the first time.
set -euo pipefail
cd "$(dirname "$0")/.."

# docker-compose.yml mounts ~/.ssh (read-only); make sure it exists, owned by
# you, rather than letting Docker create it as root.
mkdir -p "$HOME/.ssh" && chmod 700 "$HOME/.ssh"

docker compose up -d dev
# flock: a second terminal opened mid-build waits instead of building twice.
if (( $# )); then
  # Source the overlay after the first build as well: .bashrc runs before it.
  # Positional arguments keep launch options intact without shell evaluation.
  exec docker compose exec dev bash -ic '
    set -e
    flock /tmp/1811-build.lock bash -c "[ -f install/setup.bash ] || colcon build --symlink-install"
    source install/setup.bash
    exec "$@"
  ' bash "$@"
fi

exec docker compose exec dev bash -ic \
  "flock /tmp/1811-build.lock bash -c '[ -f install/setup.bash ] || colcon build --symlink-install'; exec bash"
