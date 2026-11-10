#!/usr/bin/env bash
# Start the full vehicle stack and RViz or the UI Hub from the Karbon host.
set -euo pipefail
cd "$(dirname "$0")"
original_args=("$@")
update_result=${NOVA_1811_UPDATE_RESULT:-}
unset NOVA_1811_UPDATE_RESULT

rviz_mode=auto
hub_mode=auto
cameras_mode=true
explicit_panorama=false
explicit_lidar_timestamp=false
jetson_update_mode=true
explicit_jetson_update=false
for arg in "$@"; do
  case "$arg" in
    rviz:=*) rviz_mode=${arg#rviz:=} ;;
    hub:=*) hub_mode=${arg#hub:=} ;;
    cameras:=*) cameras_mode=${arg#cameras:=} ;;
    panorama:=*) explicit_panorama=true ;;
    lidar_timestamp_mode:=*) explicit_lidar_timestamp=true ;;
    jetson_update:=*)
      jetson_update_mode=${arg#jetson_update:=}
      explicit_jetson_update=true ;;
  esac
done

if [[ "$explicit_jetson_update" == false ]]; then
  set -- "jetson_update:=true" "$@"
fi

if [[ "${jetson_update_mode,,}" == true ]]; then
  branch=$(git symbolic-ref --quiet --short HEAD 2>/dev/null || true)
  revision=$(git rev-parse --verify HEAD 2>/dev/null || true)
  if [[ -z "$branch" || -z "$revision" ]]; then
    printf '[g] Could not identify the Karbon Git branch and commit; skipping Jetson startup until the checkout is on a valid branch.\n' >&2
    revision=
  elif ! changes=$(git status --porcelain --untracked-files=normal -- \
      scripts/vehicle.sh scripts/jetson_cameras.sh scripts/prepare_zed_floor_overlay.py config/fastdds_link.xml \
      ros2_ws/src/jetson_bringup ros2_ws/src/camera_panorama ros2_ws/src/lidar_perception); then
    printf '[g] Could not inspect local Jetson source files; skipping Jetson startup.\n' >&2
    revision=
  elif [[ -n "$changes" ]]; then
    printf '[g] Local Jetson files have uncommitted changes on the Karbon; commit and push them before starting Jetson processes.\n' >&2
    revision=
  fi
  set -- "jetson_branch:=$branch" "jetson_revision:=$revision" "$@"
fi

if [[ "$explicit_panorama" == false ]]; then
  if [[ "${cameras_mode,,}" =~ ^[[:space:]]*false[[:space:]]*$ ]]; then
    set -- "panorama:=false" "$@"
  else
    set -- "panorama:=true" "$@"
  fi
fi

if [[ "$explicit_lidar_timestamp" == false ]]; then
  set -- "lidar_timestamp_mode:=TIME_FROM_ROS_TIME" "$@"
fi

# With a desktop, use the hub by default. An explicit RViz choice keeps its
# previous meaning, and a headless launch never tries to open a window.
if [[ "$hub_mode" == auto ]]; then
  if [[ -n ${DISPLAY:-} && "$rviz_mode" == auto ]]; then
    hub_mode=true
    set -- "hub:=true" "$@"
  else
    hub_mode=false
  fi
fi

if [[ "$rviz_mode" == auto ]]; then
  if [[ "${hub_mode,,}" == true || "${hub_mode,,}" == 1 ]]; then
    set -- "rviz:=false" "$@"
    rviz_mode=false
  elif [[ -n ${DISPLAY:-} ]]; then
    set -- "rviz:=true" "$@"
    rviz_mode=true
  else
    printf '[g] No DISPLAY is set; starting the vehicle stack in headless mode without a window.\n' >&2
    rviz_mode=false
  fi
fi

case "${rviz_mode,,}:${hub_mode,,}" in
  false:false|false:0|0:false|0:0) ;;
  *)
    if [[ -z ${DISPLAY:-} ]]; then
      printf '[g] Warning: DISPLAY is not set; the graphical window may not be able to open.\n' >&2
    elif ! command -v xhost >/dev/null 2>&1 || ! xhost +SI:localuser:root >/dev/null 2>&1; then
      printf '[g] Warning: Could not grant graphical display access; check xhost on the Karbon desktop.\n' >&2
    fi
    ;;
esac

# Show the hub's startup screen while Docker and colcon prepare the stack.
# Host PyQt is preferred; an existing vehicle image supplies it otherwise.
# A fresh machine without either gets the hub's normal in-container screen.
startup_token=
start_hub_splash() {
  local source="$PWD/ros2_ws/src/vehicle_ui_hub/src/screens.py" mode= session=
  [[ -n ${DISPLAY:-} && "${hub_mode,,}" =~ ^(true|1)$ && -f "$source" ]] || return 0
  if /usr/bin/python3 -c 'import PyQt5.QtWidgets' >/dev/null 2>&1; then
    mode=host
  elif docker image inspect vehicle_1811 >/dev/null 2>&1; then
    mode=container
  else
    return 0
  fi
  mkdir -p "$PWD/.nova-ui-startup" || return 0
  session=$(mktemp -d "$PWD/.nova-ui-startup/session.XXXXXX") || return 0
  startup_token=${session##*/}
  if [[ "$mode" == host ]]; then
    env -u PYTHONPATH -u QT_PLUGIN_PATH /usr/bin/python3 "$source" --host-splash "$session" "$$" \
      </dev/null >/dev/null 2>&1 &
  else
    docker run --rm --network none --pid host \
      -e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v "$PWD:/vehicle_1811:ro" -v "$PWD/.nova-ui-startup:/run/nova-ui-startup" \
      --entrypoint /usr/bin/python3 vehicle_1811 \
      /vehicle_1811/ros2_ws/src/vehicle_ui_hub/src/screens.py --host-splash "/run/nova-ui-startup/$startup_token" "$$" \
      </dev/null >/dev/null 2>&1 &
  fi
}
start_hub_splash

# Docker Compose normally reuses an existing image even if the Dockerfile has
# changed. Check the image label first so a normal launch avoids a rebuild.
image_recipe_files=(Dockerfile docker-compose.yml)
for file in .dockerignore Dockerfile.dockerignore .env; do
  [[ ! -f "$file" ]] || image_recipe_files+=("$file")
done
image_recipe=$(
  { sha256sum "${image_recipe_files[@]}" || exit 1; printf '%s\n' "${DOCKER_DEFAULT_PLATFORM:-}"; } |
    sha256sum
)
image_recipe=${image_recipe%% *}
image_name=$(docker compose config --images dev)
image_recipe_built=$(docker image inspect "$image_name" \
  --format '{{ index .Config.Labels "org.nova-utd.vehicle-image-recipe" }}' 2>/dev/null || true)

# The current Dockerfile has no copied build-context files. If that changes,
# let Docker check its cache on each start rather than overlooking source edits.
check_docker_cache=false
if grep -Eiq '^[[:space:]]*(COPY|ADD)[[:space:]]|--mount=' Dockerfile ||
   [[ -n ${COMPOSE_FILE:-} || -n ${COMPOSE_ENV_FILES:-} || -f compose.yaml || -f compose.yml || -f docker-compose.yaml ||
      -f compose.override.yaml || -f compose.override.yml ||
      -f docker-compose.override.yaml || -f docker-compose.override.yml ]] ||
   { [[ -f .env ]] && grep -Eq '^[[:space:]]*(export[[:space:]]+)?COMPOSE_FILE[[:space:]]*=' .env; }; then
  check_docker_cache=true
fi
if [[ "$image_recipe_built" != "$image_recipe" || "$check_docker_cache" == true ]]; then
  printf '[g] Building the vehicle container image (reusing cached Docker layers where possible)...\n'
  docker compose build --build-arg "NOVA_1811_IMAGE_RECIPE=$image_recipe" dev
fi

# On a first installation the image did not exist at the first attempt.
if [[ -z "$startup_token" ]]; then
  start_hub_splash
fi
if [[ -n "$startup_token" ]]; then
  set -- "hub_startup_token:=$startup_token" "$@"
fi

export NOVA_1811_FULL_STACK_BUILD=1
# A private session lets the hub ask this host process to restart the stack.
# Keeping ./g in the original terminal preserves its options and output.
supervisor_dir=
if [[ "${hub_mode,,}" =~ ^(true|1)$ ]]; then
  if mkdir -p "$PWD/.nova-ui-startup" &&
     supervisor_dir=$(mktemp -d "$PWD/.nova-ui-startup/run.XXXXXX"); then
    printf '%s\n' "$$" > "$supervisor_dir/supervisor_pid"
    set -- "hub_supervisor_token:=${supervisor_dir##*/}" "$@"
  fi
fi
if [[ "$update_result" =~ ^[A-Z_]+$ ]]; then
  set -- "hub_update_result:=$update_result" "$@"
fi
if [[ -z "$supervisor_dir" ]]; then
  exec ./scripts/vehicle.sh dev bash /vehicle_1811/scripts/vehicle.sh full-stack "$@"
fi

cleanup_supervisor() { rm -rf -- "$supervisor_dir"; }
trap cleanup_supervisor EXIT
update_requested=false
stop_requested=false
trap 'update_requested=true' USR1
trap 'stop_requested=true' INT TERM

# Bash otherwise gives a background job /dev/null as stdin, even when ./g was
# started from a terminal. Preserve stdin so Compose can detect the real TTY.
./scripts/vehicle.sh dev bash /vehicle_1811/scripts/vehicle.sh full-stack "$@" <&0 &
stack_pid=$!

stop_running_launch() {
  local launch_pid
  [[ -r "$supervisor_dir/launch_pid" ]] || return 1
  read -r launch_pid < "$supervisor_dir/launch_pid"
  [[ "$launch_pid" =~ ^[1-9][0-9]*$ ]] || return 1
  # Verify the PID still names this ROS launch before signalling it. The dev
  # container has the host PID namespace and can signal its root-owned launch.
  docker compose exec -T dev bash -c '
    pid=$1
    [[ -r /proc/$pid/cmdline ]] || exit 1
    command=$(tr "\\0" " " < "/proc/$pid/cmdline")
    [[ "$command" == *"ros2 launch obc_bringup bringup.launch.py"* ]] || exit 1
    kill -INT "$pid"
  ' bash "$launch_pid"
}

while :; do
  stack_status=0
  wait "$stack_pid" || stack_status=$?
  if [[ "$stop_requested" == true ]]; then
    # Ctrl-C still shuts the stack down when ./g is supervising the hub.
    stop_running_launch || kill -TERM "$stack_pid" 2>/dev/null || true
    while kill -0 "$stack_pid" 2>/dev/null; do wait "$stack_pid" || true; done
    exit 130
  fi
  if [[ "$update_requested" == true ]]; then
    update_requested=false
    target= branch= revision= clean=
    [[ ! -r "$supervisor_dir/request" ]] || read -r target < "$supervisor_dir/request"
    [[ ! -r "$supervisor_dir/branch" ]] || read -r branch < "$supervisor_dir/branch"
    [[ ! -r "$supervisor_dir/revision" ]] || read -r revision < "$supervisor_dir/revision"
    [[ ! -r "$supervisor_dir/clean" ]] || read -r clean < "$supervisor_dir/clean"
    rm -f -- "$supervisor_dir/request"
    if [[ ! "$target" =~ ^([[:xdigit:]]{40}|[[:xdigit:]]{64})$ ||
          ! "$revision" =~ ^([[:xdigit:]]{40}|[[:xdigit:]]{64})$ ||
          -z "$branch" || "$clean" != 1 ]] ||
       ! kill -0 "$stack_pid" 2>/dev/null || ! stop_running_launch; then
      printf 'STOP_FAILED\n' > "$supervisor_dir/error"
      continue
    fi
    printf 'accepted\n' > "$supervisor_dir/accepted"
    # Wait for ROS and the remote camera wrapper to finish before changing
    # either checkout. The normal ./g path then rebuilds and syncs the Jetson.
    while kill -0 "$stack_pid" 2>/dev/null; do wait "$stack_pid" || true; done
    if [[ "$stop_requested" == true ]]; then
      exit 130
    fi
    printf '[g] Applying the software update and restarting the vehicle stack...\n'
    update_output=$(docker compose exec -T \
      -e NOVA_1811_STACK_ROOT=/vehicle_1811 -e "NOVA_1811_STACK_BRANCH=$branch" \
      -e "NOVA_1811_STACK_REVISION=$revision" -e "NOVA_1811_STACK_CLEAN=$clean" \
      dev bash /vehicle_1811/scripts/vehicle.sh apply-update "$target") || update_output=UPDATE_FAILED
    update_result=${update_output%%$'\n'*}
    [[ "$update_result" =~ ^[A-Z_]+$ ]] || update_result=UPDATE_FAILED
    printf '[g] Software update status: %s\n' "$update_result"
    export NOVA_1811_UPDATE_RESULT=$update_result
    cleanup_supervisor
    trap - EXIT USR1 INT TERM
    exec ./g "${original_args[@]}"
  fi
  if ! kill -0 "$stack_pid" 2>/dev/null; then
    exit "$stack_status"
  fi
done
