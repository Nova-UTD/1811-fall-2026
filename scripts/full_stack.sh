#!/usr/bin/env bash
# Run inside the container. Extra arguments are passed to the ROS launch file.
set -euo pipefail

args=()
explicit_lidar=false
explicit_port=false
for arg in "$@"; do
  case "$arg" in
    lidar_ip:=*) explicit_lidar=true ;;
    port:=*) explicit_port=true ;;
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

exec ros2 launch obc_bringup bringup.launch.py "${args[@]}" "$@"
