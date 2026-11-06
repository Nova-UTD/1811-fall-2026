#!/usr/bin/env bash
# Start the full vehicle stack and RViz or the UI Hub from the Karbon host.
set -euo pipefail
cd "$(dirname "$0")"

rviz_mode=auto
hub_mode=auto
explicit_panorama=false
explicit_lidar_timestamp=false
jetson_update_mode=true
explicit_jetson_update=false
for arg in "$@"; do
  case "$arg" in
    rviz:=*) rviz_mode=${arg#rviz:=} ;;
    hub:=*) hub_mode=${arg#hub:=} ;;
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
      scripts/jetson_cameras.sh scripts/prepare_zed_floor_overlay.py config/fastdds_link.xml \
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
  set -- "panorama:=false" "$@"
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

export NOVA_1811_FULL_STACK_BUILD=1
exec ./scripts/dev.sh bash /vehicle_1811/scripts/full_stack.sh "$@"
