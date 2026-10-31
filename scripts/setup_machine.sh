#!/usr/bin/env bash
# Run once after cloning the repo, on the Karbon or any laptop standing in
# for it (native Ubuntu or WSL).
#
# This script does NOT install ROS 2 — ROS 2 Humble now lives inside the
# Docker image (see ../Dockerfile), not on the host. This script only:
#   1. Confirms/installs Docker on the host.
#   2. Adds you to the 'docker' group if needed (Linux/WSL native Docker
#      Engine only — not needed if you're using Docker Desktop's WSL
#      integration, which handles this for you).
#   3. Builds the vehicle_1811 image via docker compose.
#   4. Prints the next steps (device passthrough on WSL, running the
#      container).
set -euo pipefail

echo "== 1811 machine setup =="

# ---- Detect environment -----------------------------------------------
IS_WSL=false
if grep -qi microsoft /proc/version 2>/dev/null; then
  IS_WSL=true
fi

if [ "$IS_WSL" = true ]; then
  echo "Detected: WSL (Windows Subsystem for Linux)."
  echo "This script assumes Docker Desktop is installed on the Windows side"
  echo "with WSL integration enabled for this distro."
  echo "If 'docker --version' fails below, check:"
  echo "  Docker Desktop -> Settings -> Resources -> WSL Integration"
  echo "  (toggle this distro on, then Apply & Restart)"
else
  echo "Detected: native Linux (Karbon or Linux laptop)."
fi

# ---- Check Docker is available -----------------------------------------
if ! command -v docker >/dev/null 2>&1; then
  if [ "$IS_WSL" = true ]; then
    echo "Docker not found in this WSL distro."
    echo "Install Docker Desktop for Windows on the host, enable WSL"
    echo "integration for this distro, then re-run this script."
    exit 1
  fi

  echo "Docker not found. Installing Docker Engine (native Linux)..."
  # Official Docker install steps for Ubuntu (apt-based).
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
    -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc
  echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y \
    docker-ce docker-ce-cli containerd.io \
    docker-buildx-plugin docker-compose-plugin
else
  echo "Docker found: $(docker --version)"
fi

# ---- Docker group (native Linux Docker Engine only) --------------------
if ! [ "$IS_WSL" = true ] || command -v docker >/dev/null 2>&1 && ! docker info >/dev/null 2>&1; then
  if ! groups "$USER" | grep -q '\bdocker\b'; then
    echo "Adding $USER to the 'docker' group (needed to run docker without sudo)."
    sudo usermod -aG docker "$USER"
    echo "You must log out and back in (or close/reopen your terminal) for"
    echo "this to take effect, then re-run this script."
    exit 0
  fi
fi

# ---- Sanity check: can we actually talk to the Docker daemon? ----------
if ! docker info >/dev/null 2>&1; then
  echo "Docker CLI is present but can't reach the Docker daemon."
  if [ "$IS_WSL" = true ]; then
    echo "Make sure Docker Desktop is running on Windows, and that WSL"
    echo "integration is enabled for this distro, then try again."
  else
    echo "Make sure the Docker service is running: sudo systemctl start docker"
  fi
  exit 1
fi

echo "Docker is installed and reachable."

# ---- (Optional / fallback) host-level dialout group ---------------------
# Not required for the container itself (it runs privileged and can access
# /dev regardless), but useful if you ever want to run something against
# the serial port directly on the host, outside the container.
if ! groups "$USER" | grep -q '\bdialout\b'; then
  echo "Adding $USER to the 'dialout' group on the host (optional, for"
  echo "native/non-container serial access if you ever need it)."
  sudo usermod -aG dialout "$USER"
fi

# ---- Repo root -----------------------------------------------------------
cd "$(dirname "$0")/.."   # repo root, assuming this script lives in scripts/

# ---- Smartmicro radar vendor libs (gitignored inside the submodule) ------
# The submodule itself is pulled by `git submodule update --init --recursive`.
# Without this extract step, umrr_ros2_driver fails to link.
if [ -f scripts/fetch_radar_deps.sh ]; then
  bash scripts/fetch_radar_deps.sh
fi

# ---- Build the image -----------------------------------------------------
echo "Building the vehicle_1811 Docker image (this may take a while the"
echo "first time)..."
docker compose build

cat <<'EOF'

== Setup complete ==

Next steps:

  ./scripts/dev.sh

This drops you into the container with ROS 2 Humble and the workspace
built and sourced (the first run builds it). See README.md for:
  - rebuilding the workspace (`rebuild`) after code changes
  - running teleop (keyboard / gamepad)
  - WSL-specific USB passthrough (usbipd) if you're on a Windows laptop
  - display forwarding for the teleop pygame window

If you just got added to a new group (docker/dialout) above, log out and
back in (or restart your terminal) before continuing.
EOF