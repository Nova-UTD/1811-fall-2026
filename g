#!/usr/bin/env bash
# Start the full vehicle stack and RViz from the Karbon host.
set -euo pipefail
cd "$(dirname "$0")"

rviz_mode=auto
explicit_panorama=false
cameras_disabled=false
jetson_update_mode=true
explicit_jetson_update=false
for arg in "$@"; do
  case "$arg" in
    rviz:=*) rviz_mode=${arg#rviz:=} ;;
    panorama:=*) explicit_panorama=true ;;
    cameras:=*)
      if [[ "${arg,,}" == cameras:=false ]]; then cameras_disabled=true
      else cameras_disabled=false; fi ;;
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
      scripts/jetson_cameras.sh config/fastdds_link.xml \
      ros2_ws/src/jetson_bringup ros2_ws/src/camera_panorama ros2_ws/src/lidar_perception); then
    printf '[g] Cannot check Jetson source changes; Jetson processes cannot start.\n' >&2
    revision=
  elif [[ -n "$changes" ]]; then
    printf '[g] Jetson source has uncommitted changes on the Karbon; commit and push them before starting Jetson processes.\n' >&2
    revision=
  fi
  set -- "jetson_branch:=$branch" "jetson_revision:=$revision" "$@"
fi

if [[ "$explicit_panorama" == false && "$cameras_disabled" == false ]]; then
  set -- "panorama:=true" "$@"
fi

if [[ "$rviz_mode" == auto ]]; then
  if [[ -n ${DISPLAY:-} ]]; then
    set -- "rviz:=true" "$@"
    rviz_mode=true
  else
    printf '[g] No DISPLAY is set; starting without RViz.\n' >&2
    rviz_mode=false
  fi
fi

case "${rviz_mode,,}" in
  false|0) ;;
  *)
    if [[ -z ${DISPLAY:-} ]]; then
      printf '[g] DISPLAY is not set; RViz may not open.\n' >&2
    elif ! command -v xhost >/dev/null 2>&1 || ! xhost +SI:localuser:root >/dev/null 2>&1; then
      printf '[g] Could not grant RViz display access; check xhost on the Karbon desktop.\n' >&2
    fi
    ;;
esac

export NOVA_1811_FULL_STACK_BUILD=1
exec ./scripts/dev.sh bash /vehicle_1811/scripts/full_stack.sh "$@"
