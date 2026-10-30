#!/usr/bin/env bash
# Start the four ZED X cameras (and optionally the panorama) on the Jetson.
#
# Normally you don't run this yourself: the Karbon's bringup starts it over SSH
# (bringup.launch.py cameras:=true, panorama:=true). It can also be run by hand
# on the Jetson:
#   bash ~/1811-fall-2026/scripts/jetson_cameras.sh [--panorama] [--cameras=front,rear]
#
# Shows only the important lines; full logs go to ~/.ros/jetson_cameras/.
# Stops everything cleanly on Ctrl-C, or when the SSH connection from the
# Karbon closes -- so a camera is never left "in use" for the next start.
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
PANORAMA=false
CORNERS=false
RUN_CAMERAS=true
CAMERA_ARGS=()          # default: all four
for arg in "$@"; do
    case $arg in
        --panorama) PANORAMA=true ;;
        --corners) CORNERS=true ;;
        --no-cameras) RUN_CAMERAS=false ;;
        --cameras=*) CAMERA_ARGS=("cameras:=${arg#--cameras=}") ;;
        *) echo "usage: $0 [--panorama] [--corners] [--no-cameras] [--cameras=front,rear]"; exit 2 ;;
    esac
done

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

if $CORNERS; then
    if ! python3 -c 'import importlib.util as u; import sys; sys.exit(any(u.find_spec(n) is None for n in ("numpy", "open3d", "rclpy", "sensor_msgs_py")))'; then
        say "[ERROR] Corner detector needs Python Open3D, NumPy, rclpy and sensor_msgs_py on the Jetson."
        CORNERS=false
    elif ! cmp -s "$REPO/ros2_ws/src/lidar_perception/corner_detector.py" \
             "$REPO/ros2_ws/install/lidar_perception/lib/lidar_perception/corner_detector.py" ||
         ! cmp -s "$REPO/ros2_ws/src/lidar_perception/launch/corner_detector.launch.py" \
             "$REPO/ros2_ws/install/lidar_perception/share/lidar_perception/launch/corner_detector.launch.py"; then
        BUILD_LOG="$LOG_DIR/corner_build_$STAMP.log"
        say "building lidar_perception on Jetson (log: $BUILD_LOG)..."
        if (cd "$REPO/ros2_ws" && colcon build --symlink-install --packages-select lidar_perception >"$BUILD_LOG" 2>&1); then
            set +u
            source "$REPO/ros2_ws/install/local_setup.bash"
            set -u
        else
            say "[ERROR] lidar_perception build failed; see $BUILD_LOG"
            CORNERS=false
        fi
    fi
fi

if $RUN_CAMERAS; then
    start "cameras" cameras jetson_bringup zed_cameras.launch.py "${CAMERA_ARGS[@]}"
fi
if $PANORAMA; then
    start "panorama" panorama camera_panorama panorama.launch.py
fi
if $CORNERS; then
    start "corner_detector" corner_detector lidar_perception corner_detector.launch.py
fi

if [ ${#PIDS[@]} -eq 0 ]; then
    say "nothing to start."
    [ -n "$TTY_SAVED" ] && stty "$TTY_SAVED" 2>/dev/null
    exit 0
fi

# Only the lines that matter: each camera coming up, the panorama status, errors.
# --pid: tail quits when this script does.
tail -n +1 -F --pid=$$ "${LOGS[@]}" 2>/dev/null | grep --line-buffered -E \
    "=== zed_[a-z]+ started ===|Serial Number ->|Panorama ready|Corner detector started|now ready|Starting without|\[ERROR\]|\[FATAL\]|CAMERA NOT|not detected|failed" &
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
