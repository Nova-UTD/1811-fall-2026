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
  if [[ ${NOVA_1811_FULL_STACK_BUILD:-0} == 1 ]]; then
    # The vehicle shortcut rebuilds incrementally, including on a fresh checkout.
    exec docker compose exec dev bash -ic '
      set -e
      echo "[g] Building the vehicle workspace (incremental)..."
      flock /tmp/1811-build.lock colcon build --symlink-install --packages-ignore umrr_ros2_driver umrr_ros2_msgs smart_rviz_plugin
      source install/setup.bash
      exec "$@"
    ' bash "$@"
  fi
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
