# Pair clouds and floor planes using their header timestamps.
#
# Monotonic receipt times measure how long the node has held a sample.

# Author: Khoa Nguyen
# Email: minhkhoa.ee@outlook.com
# Copyright: 2021, Nova UTD
# License: MIT License

# Convert a ROS timestamp to seconds.
# It is used to compare LiDAR and camera timestamps.
def stamp_seconds(stamp):
    # stamp: a ROS timestamp with separate sec and nanosec fields.
    # Use one unit so LiDAR and camera header timestamps can be compared directly.
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_key(stamp):
    # Integer timestamps avoid losing nanoseconds when pairing a plane and cloud.
    return stamp.sec, stamp.nanosec


def matching_floor_plane(floor_plane_messages, cloud_header_stamp):
    # Use the most recent plane with the exact same image timestamp as the cloud.
    cloud_stamp_key = stamp_key(cloud_header_stamp)
    return next(
        (floor_plane_message for floor_plane_message in reversed(floor_plane_messages)
         if stamp_key(floor_plane_message.header.stamp) == cloud_stamp_key),
        None,
    )


def pair_error(lidar_message, camera_sample, now_monotonic_s, config):
    # camera_sample is (PointCloud2 message, Jetson receipt time in seconds)
    # or None. Return a rejection reason or None when the pair passes.
    if camera_sample is None:
        return "no cloud"
    camera_message, received_monotonic_s = camera_sample
    # Both monotonic readings come from the Jetson's local stopwatch.
    if now_monotonic_s - received_monotonic_s > config.max_cloud_age_s:
        return "camera cloud is stale"
    # The header stamps are comparable when both hosts share a time base.
    lidar_header_time_s = stamp_seconds(lidar_message.header.stamp)
    camera_header_time_s = stamp_seconds(camera_message.header.stamp)
    if not lidar_header_time_s or not camera_header_time_s:
        return "missing timestamp"
    if not lidar_message.header.frame_id or not camera_message.header.frame_id:
        return "missing frame ID"
    stamp_difference_s = abs(lidar_header_time_s - camera_header_time_s)
    if stamp_difference_s > config.max_camera_time_difference_s:
        return f"timestamp difference {stamp_difference_s:.2f}s"
    return None
