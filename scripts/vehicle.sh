#!/usr/bin/env bash
# Vehicle development, Jetson, and setup commands.
# Run with --help for the available subcommands.
# Container: dev, full-stack. Jetson: cameras, setup-ssh.
# Network: setup-link-jetson, setup-link-karbon, check-link, survey-link.
# Host setup: setup-machine, fetch-radar.

command_dev() {
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
    # The full stack can also start from a service or redirected terminal.
    # Compose must not allocate a TTY when its stdin is not a terminal.
    local -a tty_option=()
    if [[ ! -t 0 ]]; then tty_option=(-T); fi
    # The vehicle shortcut rebuilds incrementally, including on a fresh checkout.
    exec docker compose exec "${tty_option[@]}" dev bash -ic '
      set -e
      # Record the checkout being built so the hub can verify it before updating.
      NOVA_1811_STACK_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || true)
      NOVA_1811_STACK_BRANCH=$(git symbolic-ref --quiet --short HEAD 2>/dev/null || true)
      NOVA_1811_STACK_REVISION=$(git rev-parse --verify HEAD 2>/dev/null || true)
      NOVA_1811_STACK_CLEAN=0
      if [[ -n "$NOVA_1811_STACK_ROOT" && -n "$NOVA_1811_STACK_BRANCH" && -n "$NOVA_1811_STACK_REVISION" ]] &&
         stack_changes=$(git status --porcelain --untracked-files=all --ignore-submodules=none 2>/dev/null) &&
         [[ -z "$stack_changes" ]]; then
        NOVA_1811_STACK_CLEAN=1
      fi
      echo "[g] Building the vehicle workspace (incremental)..."
      flock /tmp/1811-build.lock colcon build --symlink-install --packages-ignore umrr_ros2_driver umrr_ros2_msgs smart_rviz_plugin
      source install/setup.bash
      if [[ "$(git rev-parse --show-toplevel 2>/dev/null || true)" != "$NOVA_1811_STACK_ROOT" ||
            "$(git symbolic-ref --quiet --short HEAD 2>/dev/null || true)" != "$NOVA_1811_STACK_BRANCH" ||
            "$(git rev-parse --verify HEAD 2>/dev/null || true)" != "$NOVA_1811_STACK_REVISION" ]] ||
         ! stack_changes=$(git status --porcelain --untracked-files=all --ignore-submodules=none 2>/dev/null) ||
         [[ -n "$stack_changes" ]]; then
        NOVA_1811_STACK_CLEAN=0
      fi
      export NOVA_1811_STACK_ROOT NOVA_1811_STACK_BRANCH NOVA_1811_STACK_REVISION NOVA_1811_STACK_CLEAN
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
}

command_full_stack() {
# Run inside the container. Extra arguments are passed to the ROS launch file.
set -euo pipefail

args=()
explicit_lidar=false
explicit_port=false
supervisor_token=
for arg in "$@"; do
  case "$arg" in
    lidar_ip:=*) explicit_lidar=true ;;
    port:=*) explicit_port=true ;;
    hub_supervisor_token:=*) supervisor_token=${arg#hub_supervisor_token:=} ;;
  esac
done

if [[ "$explicit_lidar" == false ]]; then
  args+=(lidar_ip:=169.254.148.80)
fi

if [[ "$explicit_port" == false ]]; then
  # Match the documented Arduino by-id path, never an unrelated serial device.
  shopt -s nullglob nocaseglob
  ports=(/dev/serial/by-id/*Arduino*)
  if (( ${#ports[@]} > 1 )); then
    printf 'Multiple Arduino devices found; choose one with port:=<path>:\n' >&2
    printf '  %s\n' "${ports[@]}" >&2
    exit 1
  elif (( ${#ports[@]} == 0 )); then
    printf 'No Arduino found in /dev/serial/by-id/; the serial bridge will not start.\n' >&2
    args+=(port:=/dev/serial/by-id/)
  else
    args+=("port:=${ports[0]}")
  fi
fi

# The host supervisor needs the exact running launch and build snapshot. It
# uses these only after an operator requests an update from this hub session.
if [[ "$supervisor_token" =~ ^run\.[[:alnum:]]{6,}$ ]]; then
  supervisor_dir="${NOVA_1811_STACK_ROOT:-/vehicle_1811}/.nova-ui-startup/$supervisor_token"
  if [[ -d "$supervisor_dir" ]]; then
    printf '%s\n' "$$" > "$supervisor_dir/launch_pid"
    printf '%s\n' "${NOVA_1811_STACK_BRANCH:-}" > "$supervisor_dir/branch"
    printf '%s\n' "${NOVA_1811_STACK_REVISION:-}" > "$supervisor_dir/revision"
    printf '%s\n' "${NOVA_1811_STACK_CLEAN:-0}" > "$supervisor_dir/clean"
  fi
fi
exec ros2 launch obc_bringup bringup.launch.py "${args[@]}" "$@"
}

command_cameras() {
# Start the four ZED X cameras (and optionally the panorama) on the Jetson.
#
# Normally you don't run this yourself: the Karbon's bringup starts it over SSH
# (bringup.launch.py cameras:=true, panorama:=true). It can also be run by hand
# on the Jetson:
#   bash ~/1811-fall-2026/scripts/vehicle.sh cameras --prepare --registration --depth-cameras=all
#
# Shows only the important lines; full logs go to ~/.ros/jetson_cameras/.
# Stops everything cleanly on Ctrl-C, or when the SSH connection from the
# Karbon closes -- so a camera is never left "in use" for the next start.
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
PANORAMA=false
REGISTRATION=false
HUB_CALIBRATION=false
RUN_CAMERAS=true
PREPARE=false
CAMERA_ARGS=()          # default: all four
DEPTH_ARGS=()           # default: depth disabled
GROUND_ARGS=()
REGISTRATION_ARGS=()
for arg in "$@"; do
    case $arg in
        --panorama) PANORAMA=true ;;
        --registration|--corners) REGISTRATION=true ;;  # --corners supports existing scripts.
        --hub-calibration) HUB_CALIBRATION=true ;;
        --no-cameras) RUN_CAMERAS=false ;;
        --prepare) PREPARE=true ;;
        --cameras=*) CAMERA_ARGS=("cameras:=${arg#--cameras=}") ;;
        --depth-cameras=*) DEPTH_ARGS=("depth_cameras:=${arg#--depth-cameras=}") ;;
        *) echo "usage: $0 cameras [--prepare] [--panorama] [--registration] [--hub-calibration] [--no-cameras] [--cameras=front,rear] [--depth-cameras=front]"; exit 2 ;;
    esac
done
if $HUB_CALIBRATION; then
    REGISTRATION_ARGS=('interactive_calibration:=true')
fi

say() { echo "[jetson] $*"; }

# Over `ssh -tt` the terminal would turn every \n into \r\n, and echo the Ctrl-C
# the Karbon types to stop us; keep the Karbon's bringup output clean. Restored
# on exit, for when this runs by hand in a terminal.
TTY_SAVED=$(stty -g 2>/dev/null)
stty -onlcr -echo 2>/dev/null

# A non-interactive SSH command doesn't read ~/.bashrc, so set up the same ROS
# environment it would: Humble, the ZED workspace, this repo, and the settings
# that match the Karbon (see docs/camera_link_setup.md).
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO/config/fastdds_link.xml"
set +u
source /opt/ros/humble/setup.bash
source "${ZED_WS:-$HOME/zed_ws}/install/local_setup.bash"
source "$REPO/ros2_ws/install/local_setup.bash"
set -u

LOG_DIR="$HOME/.ros/jetson_cameras"
mkdir -p "$LOG_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
PIDS=()
LOGS=()

if $PREPARE; then
    BUILD_LOG="$LOG_DIR/prepare_build_$STAMP.log"
    say "building Jetson camera and registration packages (log: $BUILD_LOG)..."
    if ! (cd "$REPO/ros2_ws" && colcon build --symlink-install --packages-select \
        jetson_bringup camera_panorama lidar_perception >"$BUILD_LOG" 2>&1); then
        say "[ERROR] Jetson package build failed; see $BUILD_LOG"
        exit 1
    fi
    set +u
    source "$REPO/ros2_ws/install/local_setup.bash"
    set -u
    say "Jetson packages ready."
fi

# Each launch gets its own process group (setsid), so the last-resort kill in
# stop() also gets the ZED containers it started. env --default-signal: a
# script's background jobs start with Ctrl-C ignored, and launch must hear it.
start() {  # start <label> <log name> <launch args...>
    local label=$1 log="$LOG_DIR/$2_$STAMP.log"
    shift 2
    if pgrep -f "ros2 launch $1 $2" >/dev/null; then
        say "$label already running (started elsewhere) -- leaving it alone."
        return
    fi
    setsid env --default-signal=INT ros2 launch "$@" >"$log" 2>&1 &
    PIDS+=($!)
    LOGS+=("$log")
    ln -sf "$log" "$LOG_DIR/$(basename "${log%_"$STAMP".log}")_latest.log"
    say "starting $label (log: $log)"
}

if $REGISTRATION; then
    if ! python3 -c 'import importlib.util as u; import sys; sys.exit(any(u.find_spec(n) is None for n in ("numpy", "open3d", "rclpy", "sensor_msgs_py", "geometry_msgs", "std_msgs", "tf2_ros")))'; then
        say "[ERROR] LiDAR-camera registration needs Open3D, NumPy, rclpy, sensor_msgs_py, geometry_msgs, std_msgs and tf2_ros on the Jetson."
        REGISTRATION=false
    elif ! cmp -s "$REPO/ros2_ws/src/lidar_perception/lidar_camera_registration.py" \
             "$REPO/ros2_ws/install/lidar_perception/lib/lidar_perception/lidar_camera_registration.py" ||
         ! cmp -s "$REPO/ros2_ws/src/lidar_perception/interactive_calibration.py" \
             "$REPO/ros2_ws/install/lidar_perception/lib/lidar_perception/interactive_calibration.py" ||
         ! cmp -s "$REPO/ros2_ws/src/lidar_perception/launch/lidar_camera_registration.launch.py" \
             "$REPO/ros2_ws/install/lidar_perception/share/lidar_perception/launch/lidar_camera_registration.launch.py" ||
         ! cmp -s "$REPO/ros2_ws/src/lidar_perception/config/registration.yaml" \
             "$REPO/ros2_ws/install/lidar_perception/share/lidar_perception/config/registration.yaml"; then
        BUILD_LOG="$LOG_DIR/registration_build_$STAMP.log"
        say "building camera_panorama and lidar_perception on Jetson (log: $BUILD_LOG)..."
        if (cd "$REPO/ros2_ws" && colcon build --symlink-install --packages-select camera_panorama lidar_perception >"$BUILD_LOG" 2>&1); then
            set +u
            source "$REPO/ros2_ws/install/local_setup.bash"
            set -u
        else
            say "[ERROR] lidar_perception build failed; see $BUILD_LOG"
            REGISTRATION=false
        fi
    fi
fi

if $REGISTRATION && $RUN_CAMERAS && [ ${#DEPTH_ARGS[@]} -gt 0 ] &&
   [[ "${DEPTH_ARGS[0]}" != 'depth_cameras:=none' ]] &&
   [[ "${DEPTH_ARGS[0]}" != 'depth_cameras:=false' ]]; then
    # Follow the same switch as the registration node, so disabling the filter
    # also makes the extra wrapper build and camera tracking unnecessary.
    if ! ground_filter=$(python3 - "$REPO/ros2_ws/src/lidar_perception/config/registration.yaml" <<'PY'
import sys
import yaml
try:
    with open(sys.argv[1], encoding='utf-8') as stream:
        value = yaml.safe_load(stream)['lidar_camera_registration']['ros__parameters'].get(
            'ground_filter_enabled', True)
    if not isinstance(value, bool):
        raise ValueError('ground_filter_enabled must be true or false')
    print(str(value).lower())
except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
    print(f'[jetson] [ERROR] Cannot read registration ground filter setting: {exc}', file=sys.stderr)
    sys.exit(1)
PY
    ); then
        say "[ERROR] Registration configuration unavailable; registration will not start. Cameras remain available."
        REGISTRATION=false
    elif [[ "$ground_filter" == true ]]; then
        # The stock wrapper owns the cameras; publish each SDK floor plane from
        # an isolated patched component for the same point-cloud capture.
        if overlay_setup=$(python3 "$REPO/scripts/prepare_zed_floor_overlay.py"); then
            set +u
            if source "$overlay_setup"; then
                GROUND_ARGS=('ground_detection:=true')
            else
                say "[ERROR] Cannot source ZED floor-plane wrapper; registration will not start. Cameras remain available."
                REGISTRATION=false
            fi
            set -u
        else
            say "[ERROR] ZED floor-plane wrapper unavailable; registration will not start. Cameras remain available."
            REGISTRATION=false
        fi
    fi
fi

if $RUN_CAMERAS; then
    start "cameras" cameras jetson_bringup zed_cameras.launch.py "${CAMERA_ARGS[@]}" "${DEPTH_ARGS[@]}" "${GROUND_ARGS[@]}"
fi
if $PANORAMA; then
    start "panorama" panorama camera_panorama panorama.launch.py
fi
if $REGISTRATION; then
    # An older checkout may still have the previous launch running under its old name.
    if pgrep -f '[r]os2 launch lidar_perception corner_detector[.]launch[.]py' >/dev/null; then
        say "[ERROR] The old corner_detector launch is still running; stop it before registration."
    else
        start "lidar_camera_registration" lidar_camera_registration lidar_perception lidar_camera_registration.launch.py "${REGISTRATION_ARGS[@]}"
    fi
fi

if [ ${#PIDS[@]} -eq 0 ]; then
    say "nothing to start."
    [ -n "$TTY_SAVED" ] && stty "$TTY_SAVED" 2>/dev/null
    exit 0
fi

# Only the lines that matter: each camera coming up, the panorama status, errors.
# --pid: tail quits when this script does.
tail -n +1 -F --pid=$$ "${LOGS[@]}" 2>/dev/null | grep --line-buffered -E \
    "=== zed_[a-z]+ started ===|Serial Number ->|Panorama ready|LiDAR and camera registration started|LiDAR ground|ZED ground|Floor detection|Calibrated ZED cameras|Skipped ZED clouds|Too few LiDAR points after filtering|Waiting for (a fresh LiDAR|ZED clouds|vehicle robot_description)|URDF update timed out|now ready|Starting without|\[ERROR\]|\[FATAL\]|CAMERA NOT|not detected|failed" &
FILTER=$!

stopped=false
stop() {
    $stopped && return
    stopped=true
    trap '' INT TERM HUP                 # a second Ctrl-C mustn't cut this short
    kill "$FILTER" 2>/dev/null           # shutdown noise isn't news; the logs keep it
    say "stopping..."
    # Ctrl-C to each launch only; it passes it on to its nodes once each.
    for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
    for _ in $(seq 30); do               # up to 15 s for the cameras to close
        alive=false
        for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=true; done
        $alive || break
        sleep 0.5
    done
    for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; done
    say "stopped."
    [ -n "$TTY_SAVED" ] && stty "$TTY_SAVED" 2>/dev/null
}
trap 'stop; exit 0' INT TERM HUP

wait "${PIDS[@]}"
stop
}

command_setup_ssh() {
# One-time: let the Karbon start the Jetson's cameras without a password.
# Run on the Karbon host (not in Docker):   bash scripts/vehicle.sh setup-ssh
#
# Creates a dedicated key (~/.ssh/id_ed25519_jetson -- your other keys are not
# touched), installs it on the Jetson (asks for the Jetson's password once),
# then tests the connection the same way bringup.launch.py uses it.
# Safe to re-run.
set -euo pipefail

JETSON=${1:-nova@192.168.100.2}
KEY="$HOME/.ssh/id_ed25519_jetson"
SSH=(ssh -F /dev/null -i "$KEY" -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=5)

mkdir -p "$HOME/.ssh"
chmod 700 "$HOME/.ssh"

if [ -f "$KEY" ]; then
    echo "== Using the existing key $KEY"
else
    echo "== Creating $KEY (no passphrase, so bringup can use it unattended)"
    ssh-keygen -q -t ed25519 -N '' -C "1811 karbon -> jetson" -f "$KEY"
fi

if "${SSH[@]}" "$JETSON" true 2>/dev/null; then
    echo "== The Jetson already accepts this key."
else
    echo "== Installing the key on $JETSON -- enter the Jetson's password once:"
    ssh-copy-id -i "$KEY.pub" "$JETSON"
fi

echo "== Testing the connection bringup will use..."
if ! "${SSH[@]}" "$JETSON" true; then
    echo "!! Still can't log in without a password. Is the Jetson on, and is 192.168.100.2 reachable (ping -c 2 192.168.100.2)?"
    exit 1
fi
echo "   OK: passwordless login to $JETSON works."

if "${SSH[@]}" "$JETSON" "test -f ~/1811-fall-2026/scripts/vehicle.sh"; then
    echo "   OK: the Jetson has scripts/vehicle.sh cameras."
elif "${SSH[@]}" "$JETSON" "git -C ~/1811-fall-2026 rev-parse --is-inside-work-tree >/dev/null 2>&1"; then
    echo "   The Jetson checkout is older; ./g can update it to the Karbon commit."
else
    echo "!! No Jetson checkout at ~/1811-fall-2026; follow docs/camera_link_setup.md first."
fi

cat <<EOF

Done. Close other container terminals, then start bringup from the repo root on
the Karbon host. The shortcut updates and builds the Jetson checkout, rebuilds
the Karbon workspace as needed, and opens RViz:
  ./g                 # cameras + LiDAR/ZED registration + RViz
  ./g panorama:=false # skip the default panorama
EOF
}

command_setup_link_jetson() {
# One-time Jetson setup for the Karbon <-> Jetson cable (192.168.100.1 <-> .2).
# Run on the Jetson host (ssh nova@192.168.100.2 from the Karbon), from a clone
# of this repo:   bash scripts/vehicle.sh setup-link-jetson
# Safe to re-run. Lists every change and asks before making any. Needs sudo.
#
# Pair with scripts/vehicle.sh setup-link-karbon on the Karbon, then verify both with
# scripts/vehicle.sh check-link. Full guide: docs/camera_link_setup.md.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
KARBON_IP=192.168.100.1
PROFILE="$REPO/config/fastdds_link.xml"
SYSCTL_FILE=/etc/sysctl.d/99-zz-1811-dds.conf
CHRONY_FILE=/etc/chrony/conf.d/1811-follow-karbon.conf
BASHRC="${BASHRC:-$HOME/.bashrc}"

# Point the ROS environment in a bashrc at the Karbon's settings: domain 0,
# Fast DDS, and the shared cable profile. Edits existing export lines in place
# and appends missing ones. Separate function so it can be tested on its own.
set_ros_env() {
    local file="$1" profile="$2"
    local -A want=(
        [ROS_DOMAIN_ID]=0
        [RMW_IMPLEMENTATION]=rmw_fastrtps_cpp
        [FASTRTPS_DEFAULT_PROFILES_FILE]="$profile"
    )
    local var
    for var in ROS_DOMAIN_ID RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE; do
        if grep -qE "^\s*export\s+$var=" "$file"; then
            sed -i -E "s|^(\s*)export\s+$var=.*|\1export $var=${want[$var]}  # 1811 Karbon link|" "$file"
        else
            echo "export $var=${want[$var]}  # 1811 Karbon link" >>"$file"
        fi
    done
}
[ "${SOURCE_ONLY:-0}" = 1 ] && return 0

cat <<EOF
This will set up the Jetson side of the Karbon link:

  1. ROS settings in $BASHRC (backup: $BASHRC.bak-1811), to match the Karbon:
       ROS_DOMAIN_ID       -> 0                (was: $(grep -oP '^\s*export\s+ROS_DOMAIN_ID=\K\S+' "$BASHRC" || echo unset))
       RMW_IMPLEMENTATION  -> rmw_fastrtps_cpp (was: $(grep -oP '^\s*export\s+RMW_IMPLEMENTATION=\K\S+' "$BASHRC" || echo unset))
       FASTRTPS_DEFAULT_PROFILES_FILE -> $PROFILE
  2. Network buffers: $SYSCTL_FILE (bigger UDP buffers for camera images).
  3. Time sync: install chrony (replaces systemd-timesyncd) and follow the
     Karbon ($KARBON_IP): $CHRONY_FILE
  4. Build jetson_bringup (the camera launch) in $REPO/ros2_ws against ~/zed_ws.

EOF
read -r -p "Continue? [y/N] " answer
[[ "$answer" =~ ^[Yy]$ ]] || { echo "Nothing changed."; exit 0; }

ip -4 addr | grep -q '192.168.100.2/' || {
    echo "!! No interface has 192.168.100.2 -- is this the Jetson, and is the cable connection up?"
    exit 1
}
[ -f "$PROFILE" ] || { echo "!! $PROFILE not found -- run this from the repo clone."; exit 1; }

echo
echo "== 1/4 ROS settings"
[ -d /opt/ros/humble/share/rmw_fastrtps_cpp ] || {
    sudo apt-get update -qq
    sudo apt-get install -y ros-humble-rmw-fastrtps-cpp
}
cp "$BASHRC" "$BASHRC.bak-1811"
set_ros_env "$BASHRC" "$PROFILE"
grep -nE 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|FASTRTPS_DEFAULT_PROFILES_FILE' "$BASHRC"

echo
echo "== 2/4 Network buffers"
sudo tee "$SYSCTL_FILE" >/dev/null <<'EOF'
# 1811: UDP buffers and IP fragment handling for ROS 2 camera images over the
# Karbon <-> Jetson cable. Written by scripts/vehicle.sh setup-link-jetson.
net.core.rmem_max=8388608
net.core.wmem_max=8388608
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sudo sysctl --quiet --system
sysctl net.core.rmem_max net.core.wmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh

echo
echo "== 3/4 Time sync (chrony)"
if ! command -v chronyd >/dev/null; then
    sudo apt-get update -qq
    sudo apt-get install -y chrony
fi
grep -q '^confdir /etc/chrony/conf.d' /etc/chrony/chrony.conf ||
    echo 'confdir /etc/chrony/conf.d' | sudo tee -a /etc/chrony/chrony.conf >/dev/null
sudo tee "$CHRONY_FILE" >/dev/null <<EOF
# 1811: follow the Karbon's clock over the cable. Written by scripts/vehicle.sh setup-link-jetson.
# 'prefer' makes chrony pick the Karbon whenever it's reachable; the internet
# pool from chrony.conf stays as a fallback.
server $KARBON_IP iburst prefer
EOF
sudo systemctl enable --now chrony
sudo systemctl restart chrony

echo
echo "== 4/4 Build jetson_bringup"
(
    set +u
    source /opt/ros/humble/setup.bash
    source "$HOME/zed_ws/install/local_setup.bash"
    cd "$REPO/ros2_ws"
    colcon build --symlink-install --packages-select jetson_bringup
)

echo
echo "Waiting for chrony to pick the Karbon (up to 30 s)..."
for _ in $(seq 15); do
    chronyc -n sources | grep -q "^\^\* $KARBON_IP" && break
    sleep 2
done
chronyc -n sources | grep -E "^MS|$KARBON_IP" || true

cat <<EOF

Done. Open a NEW terminal (or run: source ~/.bashrc) so the ROS settings apply,
then stop any old ROS daemon:   ros2 daemon stop
Check both machines:            bash scripts/vehicle.sh check-link
Start the cameras:              see docs/camera_link_setup.md
EOF
}

command_setup_link_karbon() {
# One-time Karbon setup for the Karbon <-> Jetson cable (192.168.100.1 <-> .2).
# Run on the Karbon host, NOT inside Docker:   bash scripts/vehicle.sh setup-link-karbon
# Safe to re-run. Lists every change and asks before making any. Needs sudo.
#
# Pair with scripts/vehicle.sh setup-link-jetson on the Jetson, then verify both with
# scripts/vehicle.sh check-link. Full guide: docs/camera_link_setup.md.
set -euo pipefail

CABLE_IF=enp4s0          # Karbon port the Jetson cable is on ("Orin Connection")
JETSON_IP=192.168.100.2
CABLE_NET=192.168.100.0/24
SYSCTL_FILE=/etc/sysctl.d/99-zz-1811-dds.conf
CHRONY_FILE=/etc/chrony/conf.d/1811-serve-jetson.conf

cat <<EOF
This will set up the Karbon side of the Jetson link:

  1. Firewall (ufw): allow everything arriving on $CABLE_IF from the Jetson
     ($JETSON_IP) -- ROS 2 (DDS) traffic and time sync. Nothing else changes.
  2. Network buffers: $SYSCTL_FILE
     (bigger UDP buffers so camera images aren't dropped).
  3. Time sync: install chrony (replaces systemd-timesyncd) and let it serve
     time to $CABLE_NET: $CHRONY_FILE
     The Karbon keeps syncing to the internet; the Jetson follows the Karbon.

EOF
read -r -p "Continue? [y/N] " answer
[[ "$answer" =~ ^[Yy]$ ]] || { echo "Nothing changed."; exit 0; }

ip -4 addr show "$CABLE_IF" 2>/dev/null | grep -q '192.168.100.1/' || {
    echo "!! $CABLE_IF doesn't have 192.168.100.1 -- is this the Karbon, and is the cable connection up?"
    exit 1
}

echo
echo "== 1/3 Firewall"
if sudo ufw status | grep -q '^Status: active'; then
    sudo ufw allow in on "$CABLE_IF" from "$JETSON_IP" comment '1811 Jetson link'
else
    echo "ufw is not active -- no rule needed."
fi

echo
echo "== 2/3 Network buffers"
# Values from the ROS 2 DDS tuning guide for large messages over a network.
# Named to sort after /etc/sysctl.d/99-ouster.conf, so these larger limits win.
sudo tee "$SYSCTL_FILE" >/dev/null <<'EOF'
# 1811: UDP buffers and IP fragment handling for ROS 2 camera images over the
# Karbon <-> Jetson cable. Written by scripts/vehicle.sh setup-link-karbon.
net.core.rmem_max=8388608
net.core.wmem_max=8388608
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sudo sysctl --quiet --system
sysctl net.core.rmem_max net.core.wmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh

echo
echo "== 3/3 Time sync (chrony)"
if ! command -v chronyd >/dev/null; then
    sudo apt-get update -qq
    sudo apt-get install -y chrony
fi
grep -q '^confdir /etc/chrony/conf.d' /etc/chrony/chrony.conf ||
    echo 'confdir /etc/chrony/conf.d' | sudo tee -a /etc/chrony/chrony.conf >/dev/null
sudo tee "$CHRONY_FILE" >/dev/null <<EOF
# 1811: serve time to the Jetson over the cable. Written by scripts/vehicle.sh setup-link-karbon.
allow $CABLE_NET
# Keep serving even when the internet is unreachable, so the two clocks stay
# together on the car (stratum 10 = only used when nothing better exists).
local stratum 10
EOF
sudo systemctl enable --now chrony
sudo systemctl restart chrony
sleep 2
chronyc tracking | grep -E 'Reference ID|System time|Leap status' || true

cat <<EOF

Done. Next:
  - Run scripts/vehicle.sh setup-link-jetson on the Jetson.
  - Recreate the Docker container so it picks up config/fastdds_link.xml:
      docker compose up -d --force-recreate dev
  - Check both machines: bash scripts/vehicle.sh check-link
EOF
}

command_check_link() {
# Check the Karbon <-> Jetson link after setup. READ-ONLY; no sudo needed.
# Run on either machine's host (not inside Docker):   bash scripts/vehicle.sh check-link
# Works out which machine it's on from the cable address. Prints PASS/FAIL per
# check and what to do about each FAIL. Guide: docs/camera_link_setup.md.

KARBON_IP=192.168.100.1
JETSON_IP=192.168.100.2
fails=0
pass() { printf '  PASS  %s\n' "$1"; }
fail() { printf '  FAIL  %s\n        -> %s\n' "$1" "$2"; fails=$((fails + 1)); }
info() { printf '  ....  %s\n' "$1"; }

if ip -4 addr | grep -q "$KARBON_IP/"; then
    role=karbon; peer=$JETSON_IP
elif ip -4 addr | grep -q "$JETSON_IP/"; then
    role=jetson; peer=$KARBON_IP
else
    echo "Neither $KARBON_IP nor $JETSON_IP is on this machine -- is the cable connection up?"
    exit 1
fi
echo "== $(hostname): $role"

echo "-- cable"
if ping -c 2 -W 2 "$peer" >/dev/null 2>&1; then
    pass "reach the other machine ($peer)"
else
    fail "reach the other machine ($peer)" "check the cable and that the other machine is on"
fi

echo "-- network buffers"
check_min() {
    local key=$1 min=$2 val
    val=$(sysctl -n "$key" 2>/dev/null || echo 0)
    if [ "$val" -ge "$min" ]; then pass "$key = $val"
    else fail "$key = $val (want >= $min)" "run scripts/vehicle.sh setup-link-$role"; fi
}
check_min net.core.rmem_max 8388608
check_min net.core.wmem_max 8388608
check_min net.ipv4.ipfrag_high_thresh 134217728

echo "-- time sync"
if ! command -v chronyc >/dev/null; then
    fail "chrony installed" "run scripts/vehicle.sh setup-link-$role"
elif [ "$role" = jetson ]; then
    if chronyc -n sources 2>/dev/null | grep -q "^\^\* $KARBON_IP"; then
        pass "following the Karbon's clock"
        info "$(chronyc -n tracking | grep 'System time' | sed 's/  */ /g')"
    else
        fail "following the Karbon's clock" "chrony hasn't selected $KARBON_IP -- wait a minute, then 'chronyc sources'; on the Karbon check the firewall rule"
    fi
else
    if chronyc -n tracking 2>/dev/null | grep -q 'Leap status *: Normal'; then
        pass "Karbon clock synced ($(chronyc -n tracking | grep -oP 'Reference ID *: \K.*'))"
    else
        fail "Karbon clock synced" "no time source yet -- check internet, or wait; it still serves the Jetson (local stratum 10)"
    fi
    if [ -f /etc/chrony/conf.d/1811-serve-jetson.conf ]; then pass "serving time to the cable"
    else fail "serving time to the cable" "run scripts/vehicle.sh setup-link-karbon"; fi
fi

echo "-- ROS settings"
if [ "$role" = karbon ]; then
    compose="$(cd "$(dirname "$0")/.." && pwd)/docker-compose.yml"
    if grep -qE '^\s*- FASTRTPS_DEFAULT_PROFILES_FILE=/vehicle_1811/config/fastdds_link.xml' "$compose"; then
        pass "docker-compose.yml uses config/fastdds_link.xml"
    else
        fail "docker-compose.yml uses config/fastdds_link.xml" "git pull the branch with the camera link changes"
    fi
    container=$(docker ps --format '{{.Names}}' 2>/dev/null | grep -m1 vehicle_1811)
    if [ -n "$container" ]; then
        prof=$(docker exec "$container" printenv FASTRTPS_DEFAULT_PROFILES_FILE 2>/dev/null)
        if [ -n "$prof" ]; then pass "running container $container has the profile"
        else fail "running container $container has the profile" "recreate it: docker compose up -d --force-recreate dev"; fi
    else
        info "no vehicle_1811 container running (start one with ./scripts/vehicle.sh dev)"
    fi
    # The ufw *service* runs even when the firewall is switched off; ENABLED=
    # in ufw.conf is what says whether rules are enforced (readable without sudo).
    if grep -qs '^ENABLED=yes' /etc/ufw/ufw.conf; then
        info "firewall is enabled -- the rule for $JETSON_IP needs sudo to list: sudo ufw status | grep $JETSON_IP"
    else
        pass "firewall not enabled -- nothing blocks the Jetson"
    fi
else
    for pair in "ROS_DOMAIN_ID=0" "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"; do
        if grep -qE "^\s*export\s+$pair(\s|$)" "$HOME/.bashrc"; then pass "~/.bashrc: $pair"
        else fail "~/.bashrc: $pair" "run scripts/vehicle.sh setup-link-jetson"; fi
    done
    prof=$(grep -oP '^\s*export\s+FASTRTPS_DEFAULT_PROFILES_FILE=\K\S+' "$HOME/.bashrc")
    if [ -n "$prof" ] && [ -f "$prof" ]; then pass "~/.bashrc: profile $prof"
    else fail "~/.bashrc: FASTRTPS_DEFAULT_PROFILES_FILE points at a real file" "run scripts/vehicle.sh setup-link-jetson"; fi
    if [ -f "$(cd "$(dirname "$0")/.." && pwd)/ros2_ws/install/jetson_bringup/share/jetson_bringup/launch/zed_cameras.launch.py" ]; then
        pass "jetson_bringup built"
    else
        fail "jetson_bringup built" "run scripts/vehicle.sh setup-link-jetson (step 4 builds it)"
    fi
fi

echo
if [ "$fails" -eq 0 ]; then
    echo "All checks passed. ROS check: on the other machine, 'ros2 topic list' should show this one's topics."
else
    echo "$fails check(s) failed -- see the -> lines above."
fi
exit "$fails"
}

command_survey_link() {
# Collect the facts needed to set up the Karbon <-> Jetson link (static IPs,
# DDS discovery, time sync, ZED cameras). READ-ONLY: changes nothing, needs no
# sudo, and prints no passwords or keys.
#
# Run on BOTH computers, on the host (not inside the Docker container):
#   bash scripts/vehicle.sh survey-link
# It prints a report and saves it to ~/link_survey_<hostname>.txt.

out="$HOME/link_survey_$(hostname).txt"

section() { printf '\n===== %s =====\n' "$1"; }
run() { printf '$ %s\n' "$*"; "$@" 2>&1 || true; }

{
section "machine"
run hostname
run uname -m
grep -E '^(PRETTY_NAME|VERSION_ID)=' /etc/os-release
[ -f /etc/nv_tegra_release ] && run head -1 /etc/nv_tegra_release

section "network interfaces (state, addresses)"
run ip -br link
run ip -br addr
section "cable plugged in? (1 = link up) and speed (Mb/s)"
for dev in /sys/class/net/*; do
    name=$(basename "$dev")
    [ "$name" = lo ] && continue
    case "$name" in docker*|br-*|veth*) continue ;; esac
    carrier=$(cat "$dev/carrier" 2>/dev/null || echo "?")
    speed=$(cat "$dev/speed" 2>/dev/null || echo "?")
    echo "$name carrier=$carrier speed=$speed"
done
run ip route
run ip neigh

section "network config tools"
for svc in NetworkManager systemd-networkd; do
    echo "$svc: $(systemctl is-active "$svc" 2>/dev/null)"
done
command -v nmcli >/dev/null && run nmcli -t -f NAME,TYPE,DEVICE,AUTOCONNECT connection show
run ls /etc/netplan
echo "ufw (firewall): $(grep -s '^ENABLED=' /etc/ufw/ufw.conf || echo 'not installed') (service: $(systemctl is-active ufw 2>/dev/null))"

section "time sync"
run timedatectl
for svc in chrony chronyd systemd-timesyncd ptp4l phc2sys; do
    echo "$svc: $(systemctl is-active "$svc" 2>/dev/null)"
done
command -v chronyc >/dev/null && run chronyc tracking
echo "internet (for an upstream time source): $(ping -c 1 -W 2 8.8.8.8 >/dev/null 2>&1 && echo yes || echo no)"

section "ROS on the host"
run ls /opt/ros
env | grep -E '^(ROS_|RMW_|FASTRTPS|FASTDDS|CYCLONEDDS)' || echo "(no ROS variables set in this shell)"
[ -f "$HOME/.bashrc" ] && grep -nE 'ros|ROS_|RMW|FASTRTPS|FASTDDS|zed_ws' "$HOME/.bashrc"

section "docker"
command -v docker >/dev/null && run docker --version
command -v docker >/dev/null && run docker ps --format '{{.Names}}  {{.Image}}  {{.Status}}'

section "ZED SDK and ROS wrapper"
if [ -d /usr/local/zed ]; then
    grep -hE 'define ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION' /usr/local/zed/include/sl/Camera.hpp 2>/dev/null
    run ls /usr/local/zed/tools
else
    echo "(no ZED SDK at /usr/local/zed)"
fi
if [ -d "$HOME/zed_ws/src" ]; then
    run ls "$HOME/zed_ws/src"
    for repo in "$HOME"/zed_ws/src/*/; do
        [ -d "$repo/.git" ] && echo "$(basename "$repo"): $(git -C "$repo" describe --tags --always 2>/dev/null)"
    done
    find "$HOME/zed_ws/src" -path '*launch*' \( -name '*.launch.py' -o -name '*.launch.xml' \) 2>/dev/null | sed "s|$HOME/||"
    find "$HOME/zed_ws/src" -path '*zed_wrapper/config*' -name '*.yaml' 2>/dev/null | sed "s|$HOME/||"
else
    echo "(no ~/zed_ws/src)"
fi
} 2>&1 | tee "$out"

echo
echo "Saved to $out -- paste the whole report back."
}

command_setup_machine() {
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
if [ -f scripts/vehicle.sh ]; then
  bash scripts/vehicle.sh fetch-radar
fi

# ---- Build the image -----------------------------------------------------
echo "Building the vehicle_1811 Docker image (this may take a while the"
echo "first time)..."
docker compose build

cat <<'EOF'

== Setup complete ==

Next steps:

  ./scripts/vehicle.sh dev

This drops you into the container with ROS 2 Humble and the workspace
built and sourced (the first run builds it). See README.md for:
  - rebuilding the workspace (`rebuild`) after code changes
  - running teleop (keyboard / gamepad)
  - WSL-specific USB passthrough (usbipd) if you're on a Windows laptop
  - display forwarding for the teleop pygame window

If you just got added to a new group (docker/dialout) above, log out and
back in (or restart your terminal) before continuing.
EOF
}

command_fetch_radar() {
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
}

command_check_jetson_update() (
  # Check the Jetson before stopping the running stack. The remote helper may
  # fetch the commit, but --check never switches branches or changes files.
  set -u
  local target=${1:-} branch=${NOVA_1811_STACK_BRANCH:-}
  local enabled=${NOVA_UI_JETSON_UPDATE:-false}
  local host=${NOVA_UI_JETSON_HOST:-} key=${NOVA_UI_JETSON_KEY:-} repo=${NOVA_UI_JETSON_REPO:-}
  local cameras=${NOVA_UI_JETSON_CAMERAS:-true} registration=${NOVA_UI_JETSON_REGISTRATION:-true}
  local helper remote output
  if [[ "${enabled,,}" != true ]] ||
     [[ "${cameras,,}" == false && "${registration,,}" != true ]]; then
    echo JETSON_SKIPPED; return 0
  fi
  if [[ ! "$target" =~ ^([[:xdigit:]]{40}|[[:xdigit:]]{64})$ || -z "$host" || -z "$key" || -z "$repo" ]] ||
     ! git check-ref-format --branch "$branch" >/dev/null 2>&1; then
    echo JETSON_UNAVAILABLE; return 0
  fi
  helper="$(cd "$(dirname "$0")/.." && pwd -P)/ros2_ws/src/obc_bringup/scripts/sync_jetson_repo.sh"
  if [[ ! -r "$helper" ]]; then echo JETSON_UNAVAILABLE; return 0; fi
  printf -v remote 'bash -s -- %q %q %q --check' "$repo" "$branch" "$target"
  if output=$(timeout 45s ssh -T -F /dev/null -i "$key" -o IdentitiesOnly=yes \
      -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
      -o StrictHostKeyChecking=accept-new -o UserKnownHostsFile=/root/.ssh/known_hosts \
      -o LogLevel=ERROR -- "$host" "$remote" < "$helper" 2>&1); then
    echo JETSON_READY
  else
    case "$output" in
      *'local changes'*) echo JETSON_DIRTY ;;
      *'diverged or is ahead'*) echo JETSON_DIVERGED ;;
      *) echo JETSON_UNAVAILABLE ;;
    esac
    # Keep the detailed remote error in the launch terminal for debugging.
    printf '%s\n' "$output" >&2
  fi
)

command_updates() (
  # The first line is a fixed status token; later lines are commit subjects.
  set -uo pipefail
  local action=$1 expected_target=${2:-}
  local repo branch revision clean uid gid updater_home="" update_ref="" current_root current_branch current_revision
  local changes target count readiness changed_submodules
  local -a git_exec
  repo="$(cd "$(dirname "$0")/.." && pwd -P)"
  branch=${NOVA_1811_STACK_BRANCH:-}
  revision=${NOVA_1811_STACK_REVISION:-}
  clean=${NOVA_1811_STACK_CLEAN:-0}
  if [[ ${NOVA_1811_STACK_ROOT:-} != "$repo" || -z "$branch" ]] ||
     [[ ! "$revision" =~ ^([[:xdigit:]]{40}|[[:xdigit:]]{64})$ ]]; then
    echo UNVERIFIED; return 0
  fi
  # Git runs as the checkout owner so the root container does not create root-owned files.
  uid=$(stat -c '%u' "$repo") || { echo PERMISSION; return 0; }
  gid=$(stat -c '%g' "$repo") || { echo PERMISSION; return 0; }
  git_exec=(git -C "$repo")
  if [[ $(id -u) == 0 && "$uid" != 0 ]]; then
    if ! command -v setpriv >/dev/null 2>&1; then echo PERMISSION; return 0; fi
    updater_home=$(mktemp -d /tmp/1811-update-home.XXXXXXXX) || { echo PERMISSION; return 0; }
    if ! chown "$uid:$gid" "$updater_home"; then rm -rf -- "$updater_home"; echo PERMISSION; return 0; fi
    if [[ -d /run/1811-update-ssh ]]; then ln -s /run/1811-update-ssh "$updater_home/.ssh"; fi
    git_exec=(setpriv --reuid "$uid" --regid "$gid" --clear-groups env
      "HOME=$updater_home" "XDG_CONFIG_HOME=$updater_home/.config" "GIT_CONFIG_GLOBAL=/dev/null" git -C "$repo")
  fi
  export GIT_TERMINAL_PROMPT=0 GCM_INTERACTIVE=never GIT_ASKPASS=/bin/false SSH_ASKPASS=/bin/false
  export GIT_SSH_COMMAND="${GIT_SSH_COMMAND:-ssh -o BatchMode=yes -o ConnectTimeout=8}"
  cleanup_software_update() {
    if [[ -n "$update_ref" ]]; then "${git_exec[@]}" update-ref -d "$update_ref" >/dev/null 2>&1 || true; fi
    if [[ -n "$updater_home" ]]; then rm -rf -- "$updater_home"; fi
  }
  trap cleanup_software_update EXIT
  if ! "${git_exec[@]}" check-ref-format --branch "$branch" >/dev/null 2>&1; then
    echo UNVERIFIED; return 0
  fi
  verify_running_checkout() {
    current_root=$("${git_exec[@]}" rev-parse --show-toplevel 2>/dev/null) || return 1
    current_branch=$("${git_exec[@]}" symbolic-ref --quiet --short HEAD 2>/dev/null) || return 1
    current_revision=$("${git_exec[@]}" rev-parse --verify HEAD 2>/dev/null) || return 1
    [[ "$current_root" == "$repo" && "$current_branch" == "$branch" && "$current_revision" == "$revision" ]]
  }
  if ! verify_running_checkout; then echo UNVERIFIED; return 0; fi
  changes=$("${git_exec[@]}" status --porcelain=v1 --untracked-files=all --ignore-submodules=none 2>/dev/null) || {
    echo PERMISSION; return 0;
  }
  if [[ "$action" == apply ]]; then
    if [[ "$clean" != 1 ]]; then echo RESTART; return 0; fi
    if [[ -n "$changes" ]]; then echo DIRTY; return 0; fi
    if [[ ! "$expected_target" =~ ^([[:xdigit:]]{40}|[[:xdigit:]]{64})$ ]]; then echo STALE; return 0; fi
  fi
  if ! "${git_exec[@]}" remote get-url origin >/dev/null 2>&1; then echo NO_REMOTE; return 0; fi
  # Fetch the running branch into a temporary ref without changing the working tree.
  update_ref="refs/nova-ui/update-${BASHPID}-${RANDOM}"
  if ! timeout 45s "${git_exec[@]}" -c submodule.recurse=false fetch --no-tags --no-recurse-submodules \
    --no-write-fetch-head --refmap= -- origin "+refs/heads/$branch:$update_ref" >/dev/null 2>&1; then
    echo NETWORK; return 0
  fi
  target=$("${git_exec[@]}" rev-parse --verify "$update_ref^{commit}" 2>/dev/null) || { echo ERROR; return 0; }
  if ! verify_running_checkout; then echo UNVERIFIED; return 0; fi
  # A remote change after the UI check requires a new check before applying it.
  if [[ "$action" == apply && "$target" != "$expected_target" ]]; then echo STALE; return 0; fi
  if [[ "$target" == "$revision" ]]; then echo CURRENT; return 0; fi
  if ! "${git_exec[@]}" merge-base --is-ancestor "$revision" "$target" >/dev/null 2>&1; then
    if "${git_exec[@]}" merge-base --is-ancestor "$target" "$revision" >/dev/null 2>&1; then
      echo AHEAD
    else
      echo DIVERGED
    fi
    return 0
  fi
  count=$("${git_exec[@]}" rev-list --count "$revision..$target" 2>/dev/null) || { echo ERROR; return 0; }
  # A changed gitlink needs its own checkout update. Do not rebuild a newer
  # parent with older submodule contents or move a developer's submodule HEAD.
  changed_submodules=$("${git_exec[@]}" diff-tree --no-commit-id --raw -r --no-renames "$revision" "$target" 2>/dev/null |
    awk '$1 == ":160000" || $2 == "160000" { changed = 1 } END { if (changed) print "yes" }') || {
    echo ERROR; return 0;
  }
  if [[ "$action" == check ]]; then
    readiness=READY
    if [[ "$clean" != 1 ]]; then readiness=RESTART
    elif [[ -n "$changes" ]]; then readiness=DIRTY
    elif [[ -n "$changed_submodules" ]]; then readiness=SUBMODULE
    fi
    printf 'AVAILABLE %s %s %s\n' "$target" "$count" "$readiness"
    # Commit subjects are shorter and more useful here than merge boilerplate.
    "${git_exec[@]}" -c log.showSignature=false log --no-merges --format='%<(160,trunc)%s' -n 20 "$revision..$target" 2>/dev/null || true
    return 0
  fi
  if [[ -n "$changed_submodules" ]]; then echo SUBMODULE; return 0; fi
  # Recheck after the network operation; never overwrite ignored local files.
  if ! verify_running_checkout; then echo UNVERIFIED; return 0; fi
  changes=$("${git_exec[@]}" status --porcelain=v1 --untracked-files=all --ignore-submodules=none 2>/dev/null) || {
    echo PERMISSION; return 0;
  }
  if [[ -n "$changes" ]]; then echo DIRTY; return 0; fi
  if ! "${git_exec[@]}" -c submodule.recurse=false merge --ff-only --no-edit --no-overwrite-ignore "$target" \
    >/dev/null 2>&1; then
    echo UPDATE_FAILED; return 0
  fi
  current_branch=$("${git_exec[@]}" symbolic-ref --quiet --short HEAD 2>/dev/null || true)
  current_revision=$("${git_exec[@]}" rev-parse --verify HEAD 2>/dev/null || true)
  if [[ "$current_branch" != "$branch" || "$current_revision" != "$target" ]]; then echo ERROR; return 0; fi
  echo UPDATED
)

usage() {
  cat <<'USAGE'
Usage: bash scripts/vehicle.sh <command> [arguments]

  dev [command ...]    Open the development container or run a command in it
  full-stack [args...] Launch the vehicle stack from inside the container
  cameras [args...]    Start camera and registration processes on the Jetson
  setup-ssh [host]     Set up SSH from the Karbon to the Jetson
  setup-link-jetson    Configure the Jetson side of the link
  setup-link-karbon    Configure the Karbon side of the link
  check-link           Check the Karbon/Jetson link
  survey-link          Collect a link diagnostic report
  setup-machine        Set up Docker and dependencies on the host
  fetch-radar          Fetch radar vendor libraries

For the usual vehicle launch, use ./g from the repository root.
USAGE
}

case ${1:-} in
  dev) shift; command_dev "$@" ;;
  full-stack) shift; command_full_stack "$@" ;;
  cameras) shift; command_cameras "$@" ;;
  setup-ssh) shift; command_setup_ssh "$@" ;;
  setup-link-jetson) shift; command_setup_link_jetson "$@" ;;
  setup-link-karbon) shift; command_setup_link_karbon "$@" ;;
  check-link) shift; command_check_link "$@" ;;
  survey-link) shift; command_survey_link "$@" ;;
  setup-machine) shift; command_setup_machine "$@" ;;
  fetch-radar) shift; command_fetch_radar "$@" ;;
  check-update) shift; command_updates check "$@" ;;
  check-jetson-update) shift; command_check_jetson_update "$@" ;;
  apply-update) shift; command_updates apply "$@" ;;
  ''|-h|--help) usage ;;
  *) printf 'Unknown vehicle command: %s\n\n' "$1" >&2; usage >&2; exit 2 ;;
esac
