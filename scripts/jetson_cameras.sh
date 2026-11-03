#!/usr/bin/env bash
# Start the four ZED X cameras (and optionally the panorama) on the Jetson.
#
# Normally you don't run this yourself: the Karbon's bringup starts it over SSH
# (bringup.launch.py cameras:=true, panorama:=true). It can also be run by hand
# on the Jetson:
#   bash ~/1811-fall-2026/scripts/jetson_cameras.sh [--panorama] [--cameras=front,rear] [--depth-cameras=front]
#
# Shows only the important lines; full logs go to ~/.ros/jetson_cameras/.
# Stops everything cleanly on Ctrl-C, or when the SSH connection from the
# Karbon closes -- so a camera is never left "in use" for the next start.
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
PANORAMA=false
REGISTRATION=false
RUN_CAMERAS=true
PREPARE=false
CAMERA_ARGS=()          # default: all four
DEPTH_ARGS=()           # default: depth disabled
for arg in "$@"; do
    case $arg in
        --panorama) PANORAMA=true ;;
        --registration|--corners) REGISTRATION=true ;;  # --corners supports existing scripts.
        --no-cameras) RUN_CAMERAS=false ;;
        --prepare) PREPARE=true ;;
        --cameras=*) CAMERA_ARGS=("cameras:=${arg#--cameras=}") ;;
        --depth-cameras=*) DEPTH_ARGS=("depth_cameras:=${arg#--depth-cameras=}") ;;
        *) echo "usage: $0 [--prepare] [--panorama] [--registration] [--no-cameras] [--cameras=front,rear] [--depth-cameras=front]"; exit 2 ;;
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
    if ! python3 -c 'import importlib.util as u; import sys; sys.exit(any(u.find_spec(n) is None for n in ("numpy", "open3d", "rclpy", "sensor_msgs_py", "std_msgs", "tf2_ros")))'; then
        say "[ERROR] LiDAR-camera registration needs Open3D, NumPy, rclpy, sensor_msgs_py, std_msgs and tf2_ros on the Jetson."
        REGISTRATION=false
    elif ! cmp -s "$REPO/ros2_ws/src/lidar_perception/lidar_camera_registration.py" \
             "$REPO/ros2_ws/install/lidar_perception/lib/lidar_perception/lidar_camera_registration.py" ||
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

if $RUN_CAMERAS; then
    start "cameras" cameras jetson_bringup zed_cameras.launch.py "${CAMERA_ARGS[@]}" "${DEPTH_ARGS[@]}"
fi
if $PANORAMA; then
    start "panorama" panorama camera_panorama panorama.launch.py
fi
if $REGISTRATION; then
    # An older checkout may still have the previous launch running under its old name.
    if pgrep -f '[r]os2 launch lidar_perception corner_detector[.]launch[.]py' >/dev/null; then
        say "[ERROR] The old corner_detector launch is still running; stop it before registration."
    else
        start "lidar_camera_registration" lidar_camera_registration lidar_perception lidar_camera_registration.launch.py
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
    "=== zed_[a-z]+ started ===|Serial Number ->|Panorama ready|LiDAR and camera registration started|Calibration quality|Calibrated ZED cameras|Skipped ZED clouds|No camera calibration accepted|Waiting for (a fresh LiDAR|ZED clouds|vehicle robot_description)|URDF update timed out|now ready|Starting without|\[ERROR\]|\[FATAL\]|CAMERA NOT|not detected|failed" &
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
