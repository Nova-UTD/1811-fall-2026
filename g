#!/usr/bin/env bash
# Start the full vehicle stack and RViz or the UI Hub from the Karbon host.
set -euo pipefail
cd "$(dirname "$0")"

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
    printf '[g] Cannot identify the Karbon Git branch and commit; Jetson processes cannot start.\n' >&2
    revision=
  elif ! changes=$(git status --porcelain --untracked-files=normal -- \
      scripts/vehicle.sh scripts/jetson_cameras.sh scripts/prepare_zed_floor_overlay.py config/fastdds_link.xml \
      ros2_ws/src/jetson_bringup ros2_ws/src/camera_panorama ros2_ws/src/lidar_perception); then
    printf '[g] Cannot check Jetson source changes; Jetson processes cannot start.\n' >&2
    revision=
  elif [[ -n "$changes" ]]; then
    printf '[g] Jetson source has uncommitted changes on the Karbon; commit and push them before starting Jetson processes.\n' >&2
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
    printf '[g] No DISPLAY is set; starting without a graphical display.\n' >&2
    rviz_mode=false
  fi
fi

case "${rviz_mode,,}:${hub_mode,,}" in
  false:false|false:0|0:false|0:0) ;;
  *)
    if [[ -z ${DISPLAY:-} ]]; then
      printf '[g] DISPLAY is not set; the graphical display may not open.\n' >&2
    elif ! command -v xhost >/dev/null 2>&1 || ! xhost +SI:localuser:root >/dev/null 2>&1; then
      printf '[g] Could not grant graphical display access; check xhost on the Karbon desktop.\n' >&2
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
  printf '[g] Preparing the Docker image (using cached layers when available)...\n'
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
exec ./scripts/vehicle.sh dev bash /vehicle_1811/scripts/vehicle.sh full-stack "$@"
