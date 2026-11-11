#!/usr/bin/env bash
# Verify that an actual LiDAR cloud arrives after a recovery build.
set -u

printf '[reload] Waiting up to 45 seconds for a LiDAR cloud on /ouster/points...\n'
status=0
timeout 45s ros2 topic echo --once --field header \
  --qos-reliability best_effort --qos-depth 1 /ouster/points sensor_msgs/msg/PointCloud2 || status=$?

case "$status" in
  0)
    printf '[reload] LiDAR cloud received on /ouster/points.\n'
    ;;
  124)
    printf '[reload] No LiDAR cloud arrived in 45 seconds. The build is complete, but LiDAR is not ready; check sensor power, address, cable and UDP delivery.\n' >&2
    exit 1
    ;;
  130|143)
    exit "$status"
    ;;
  *)
    printf '[reload] Could not check /ouster/points (ROS command exited %s); the sensor status is unknown.\n' "$status" >&2
    exit 1
    ;;
esac
