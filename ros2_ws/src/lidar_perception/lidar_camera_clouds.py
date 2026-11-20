# Convert ROS clouds, filter ground and walls, and build display clouds.

# Author: Khoa Nguyen
# Email: minhkhoa.ee@outlook.com
# Copyright: 2021, Nova UTD
# License: MIT License

import math

import numpy as np
import open3d as o3d
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from lidar_camera_timing import stamp_key
from lidar_perception_transforms import transform_matrix


# Convert LiDAR or ZED PointCloud2 message to an Open3D XYZ point cloud.
def point_cloud_from_message(cloud_message):
    # Read one [x, y, z] row in meters per point in the message's sensor frame.
    points_xyz_m = point_cloud2.read_points_numpy(
        cloud_message, field_names=("x", "y", "z"), skip_nans=True,
    )
    # Drop rows containing NaN or infinity.
    points_xyz_m = points_xyz_m[np.isfinite(points_xyz_m).all(axis=1)]
    cloud = o3d.geometry.PointCloud()
    # Open3D expects its own point-cloud object with double-precision coordinates.
    cloud.points = o3d.utility.Vector3dVector(points_xyz_m.astype(np.float64))
    return cloud


def remove_wall_patches(matching_cloud, up_direction_in_cloud, config):
    # matching_cloud is the downsampled Open3D cloud in its sensor frame.
    # up_direction_in_cloud has three XYZ components in that same frame.
    # config supplies the patch size, tilt, and distance thresholds.
    # Return the remaining cloud, wall count, and number of removed points.
    if len(matching_cloud.points) < config.min_points:
        return matching_cloud, 0, 0
    up_direction_in_cloud = np.asarray(up_direction_in_cloud, dtype=float)
    if (up_direction_in_cloud.shape != (3,) or not np.isfinite(up_direction_in_cloud).all()
            or np.linalg.norm(up_direction_in_cloud) < 1e-9):
        raise ValueError("wall filter needs a valid up direction")
    # Normalize the up direction to a unit vector.
    up_direction_in_cloud = up_direction_in_cloud / np.linalg.norm(up_direction_in_cloud)

    # Estimate normals on a copy so the input cloud retains its attributes.
    detection_cloud = o3d.geometry.PointCloud(matching_cloud)
    detection_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=config.fpfh_normal_radius_m, max_nn=30))
    # Open3D groups points on flat surfaces and returns boxes around the patches.
    patch_boxes = detection_cloud.detect_planar_patches(
        normal_variance_threshold_deg=40, coplanarity_deg=70, outlier_ratio=0.75,
        min_plane_edge_length=config.wall_min_height_m, min_num_points=config.min_points,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))

    cloud_points_m = np.asarray(matching_cloud.points)
    # Each True marks one row of cloud_points_m for removal.
    wall_point_mask = np.zeros(len(cloud_points_m), dtype=bool)
    wall_count = 0
    for patch_box in patch_boxes:
        # The patch box's axes 0 and 1 lie along the surface. Axis 2 is the
        # normal. patch_box.extent contains the three full lengths in meters.
        #
        # patch_box.R stores each patch axis as a column in the cloud's XYZ frame.
        #                  axis 0  axis 1  axis 2
        #       R =       [[ 1,      0,      0],   # X components
        #                  [ 0,      0,     -1],   # Y components
        #                  [ 0,      1,      0]]   # Z components
        # Axis 0 = [1, 0, 0], axis 1 = [0, 0, 1], axis 2 = [0, -1, 0].
        # A vertical wall has a normal perpendicular to up, so their dot
        # product is near 0. A floor has an absolute dot product near 1.
        patch_normal_in_cloud = patch_box.R[:, 2]
        if (abs(np.dot(patch_normal_in_cloud, up_direction_in_cloud))
                > math.sin(math.radians(config.wall_max_tilt_deg))):
            continue
        # R.T puts the patch axes in rows. Multiplying by up gives the signed
        # vertical rise per meter along each axis. With up = [0, 0, 1], the
        # example gives [0, 1, 0]. The absolute rises from axes 0 and 1 give
        # the patch's vertical span. Extents [2, 3, 0.1] m give 3 m of height.
        wall_height_m = np.dot(
            np.abs((patch_box.R.T @ up_direction_in_cloud)[:2]), patch_box.extent[:2],
        )
        if wall_height_m < config.wall_min_height_m:
            continue
        wall_count += 1
        # Expand the footprint along axes 0 and 1. Axis 2 bounds the distance
        # from the patch center along the normal.
        patch_half_bounds_m = patch_box.extent / 2.0
        patch_half_bounds_m[:2] += config.wall_edge_margin_m
        patch_half_bounds_m[2] = max(patch_half_bounds_m[2], config.wall_distance_tolerance_m)
        # Subtract the box center and project onto its axes.
        points_in_patch_frame_m = (cloud_points_m - patch_box.center) @ patch_box.R
        wall_point_mask |= np.all(np.abs(points_in_patch_frame_m) <= patch_half_bounds_m, axis=1)

    removed_point_count = int(np.count_nonzero(wall_point_mask))
    if not removed_point_count:
        return matching_cloud, wall_count, 0
    return (
        matching_cloud.select_by_index(np.flatnonzero(~wall_point_mask).tolist()),
        wall_count, removed_point_count,
    )


# This step is only used for RViz visualization.
# Pack all registered clouds as XYZRGB PointCloud2 for RViz.
def pack_registered_cloud(clouds_in_output_frame, output_header, config):
    # Each pair has a sensor name and a cloud already expressed in output_header.frame_id.
    # The header provides the output frame and stamp. config supplies the sensor colors.
    total_point_count = sum(len(cloud.points) for _, cloud in clouds_in_output_frame)
    packed_points = np.empty(
        total_point_count, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")],
    )
    start_index = 0
    for sensor_name, cloud in clouds_in_output_frame:
        xyz = np.asarray(cloud.points)
        end_index = start_index + len(xyz)
        (
            packed_points["x"][start_index:end_index],
            packed_points["y"][start_index:end_index],
            packed_points["z"][start_index:end_index],
        ) = xyz.T
        red, green, blue = getattr(config, f"{sensor_name}_rgb")  # Pick this sensor's configured color.
        packed_points["rgb"][start_index:end_index] = (red << 16) | (green << 8) | blue
        start_index = end_index

    output_message = PointCloud2()
    output_message.header = output_header
    output_message.height = 1
    output_message.width = total_point_count
    output_message.fields = [
        PointField(name=field_name, offset=offset, datatype=datatype, count=1)
        for field_name, offset, datatype in (
            ("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32),
            ("z", 8, PointField.FLOAT32), ("rgb", 12, PointField.UINT32),
        )
    ]
    output_message.is_bigendian = False
    output_message.point_step = 16
    output_message.row_step = 16 * total_point_count
    output_message.is_dense = True
    output_message.data = packed_points.tobytes()
    return output_message


def remove_zed_ground(camera_cloud, camera_header, floor_plane_message, tolerance_m):
    # The wrapper's floor pose uses the plane center as position and +Z as normal.
    # It is only valid for the exact image and camera frame used by this cloud.
    if floor_plane_message is None:
        raise ValueError("no ZED floor plane for this cloud timestamp")
    if (stamp_key(floor_plane_message.header.stamp) != stamp_key(camera_header.stamp)
            or not any(stamp_key(camera_header.stamp))):
        raise ValueError("ZED floor plane timestamp does not match the camera cloud")
    if not camera_header.frame_id or floor_plane_message.header.frame_id != camera_header.frame_id:
        raise ValueError("ZED floor plane frame does not match the camera cloud")
    plane_position, plane_orientation = (
        floor_plane_message.pose.position, floor_plane_message.pose.orientation,
    )
    plane_center_m = np.array([plane_position.x, plane_position.y, plane_position.z], dtype=float)
    plane_quaternion_xyzw = np.array(
        [plane_orientation.x, plane_orientation.y, plane_orientation.z, plane_orientation.w],
        dtype=float,
    )
    quaternion_length = np.linalg.norm(plane_quaternion_xyzw)
    if not np.isfinite(plane_center_m).all() or not np.isfinite(quaternion_length) or quaternion_length < 1e-12:
        raise ValueError("ZED floor plane contains an invalid center or orientation")
    # x, y, z, w are the unit quaternion components. Rotating the floor pose's
    # local +Z axis gives the plane normal in the camera frame.
    x, y, z, w = plane_quaternion_xyzw / quaternion_length
    plane_normal_in_camera = np.array([2 * (x*z + w*y), 2 * (y*z - w*x), 1 - 2 * (x*x + y*y)])
    camera_points_m = np.asarray(camera_cloud.points)
    floor_point_mask = np.abs((camera_points_m - plane_center_m) @ plane_normal_in_camera) <= tolerance_m
    return (
        camera_cloud.select_by_index(np.flatnonzero(~floor_point_mask).tolist()),
        int(np.count_nonzero(floor_point_mask)),
    )


def crop_calibration_cloud(sensor_cloud, tf_buffer, cloud_frame_id, box_frame_id, box_min_m, box_max_m):
    # box_min_m and box_max_m are XYZ corners in base_link or the camera body.
    # Use that frame for the bounds test and return the points in their sensor frame.
    box_from_cloud = (np.eye(4) if cloud_frame_id == box_frame_id else transform_matrix(
        tf_buffer.lookup_transform(box_frame_id, cloud_frame_id, Time()).transform))
    cloud_points_m = np.asarray(sensor_cloud.points)
    points_in_box_frame_m = cloud_points_m @ box_from_cloud[:3, :3].T + box_from_cloud[:3, 3]
    inside_box_mask = np.all((points_in_box_frame_m >= box_min_m) & (points_in_box_frame_m <= box_max_m), axis=1)
    return sensor_cloud.select_by_index(np.flatnonzero(inside_box_mask).tolist())


def lidar_up_from_imu(imu_samples, lidar_header, tf_buffer, now_monotonic_s, config):
    # Each sample contains an Imu message and its Jetson receipt time in seconds.
    # Header times select samples near the LiDAR scan. Local receipt times
    # reject samples held too long. Return a unit up vector in base_link.
    lidar_stamp = stamp_key(lidar_header.stamp)
    lidar_stamp_ns = lidar_stamp[0] * 1000000000 + lidar_stamp[1]
    if not lidar_stamp_ns:
        raise ValueError("LiDAR cloud has no timestamp for IMU matching")
    max_stamp_difference_ns = int(config.lidar_imu_max_time_difference_s * 1000000000)
    stationary_accelerations, imu_frame_ids, used_imu_stamps_ns = [], set(), set()
    for imu_message, received_monotonic_s in imu_samples:
        seconds, nanoseconds = stamp_key(imu_message.header.stamp)
        imu_stamp_ns = seconds * 1000000000 + nanoseconds
        if (not imu_stamp_ns or imu_stamp_ns in used_imu_stamps_ns or not imu_message.header.frame_id
                or abs(imu_stamp_ns - lidar_stamp_ns) > max_stamp_difference_ns
                or now_monotonic_s - received_monotonic_s > config.max_cloud_age_s):
            continue
        if imu_message.linear_acceleration_covariance[0] == -1 or imu_message.angular_velocity_covariance[0] == -1:
            continue
        acceleration = np.array([imu_message.linear_acceleration.x, imu_message.linear_acceleration.y,
                                 imu_message.linear_acceleration.z], dtype=float)
        angular_velocity = np.array([imu_message.angular_velocity.x, imu_message.angular_velocity.y,
                                     imu_message.angular_velocity.z], dtype=float)
        # Near 1 g and little rotation are necessary, but not sufficient, to
        # establish gravity. Calibration should be performed while parked.
        if (not np.isfinite(acceleration).all() or not np.isfinite(angular_velocity).all()
                or not 8.0 <= np.linalg.norm(acceleration) <= 11.5
                or np.linalg.norm(angular_velocity) > 0.20):
            continue
        stationary_accelerations.append(acceleration)
        imu_frame_ids.add(imu_message.header.frame_id)
        used_imu_stamps_ns.add(imu_stamp_ns)
    if len(stationary_accelerations) < 3:
        raise ValueError("need at least 3 fresh, stationary /ouster/imu samples near the LiDAR timestamp")
    if len(imu_frame_ids) != 1:
        raise ValueError("matched /ouster/imu samples have different frames")
    mean_acceleration = np.mean(stationary_accelerations, axis=0)
    acceleration_magnitude = np.linalg.norm(mean_acceleration)
    if not np.isfinite(acceleration_magnitude) or acceleration_magnitude < 8.0:
        raise ValueError("matched /ouster/imu accelerations do not give a consistent up direction")
    imu_frame_id = imu_frame_ids.pop()
    if imu_frame_id != "base_link":
        base_from_imu_transform = tf_buffer.lookup_transform("base_link", imu_frame_id, Time()).transform
        mean_acceleration = transform_matrix(base_from_imu_transform)[:3, :3] @ mean_acceleration
    return mean_acceleration / np.linalg.norm(mean_acceleration), len(stationary_accelerations)


def remove_lidar_ground(lidar_cloud, lidar_cloud_frame, tf_buffer, up_direction_in_base, config):
    # up_direction_in_base is the IMU-derived unit vector in base_link.
    # Fit low planes in base_link and return the kept points in the LiDAR frame.
    if not lidar_cloud_frame:
        raise ValueError("LiDAR cloud has no frame ID")
    base_from_cloud = (np.eye(4) if lidar_cloud_frame == "base_link" else transform_matrix(
        tf_buffer.lookup_transform("base_link", lidar_cloud_frame, Time()).transform))
    lidar_points_m = np.asarray(lidar_cloud.points)
    points_in_base_m = lidar_points_m @ base_from_cloud[:3, :3].T + base_from_cloud[:3, 3]
    near_ground_height_mask = np.abs(points_in_base_m[:, 2]) <= config.lidar_ground_search_height_m
    ground_search_cloud = o3d.geometry.PointCloud()
    ground_search_cloud.points = o3d.utility.Vector3dVector(points_in_base_m[near_ground_height_mask])
    ground_search_cloud = ground_search_cloud.voxel_down_sample(config.icp_voxel_size_m)
    minimum_inliers = config.lidar_ground_min_inliers
    min_normal_alignment = math.cos(math.radians(config.lidar_ground_max_tilt_deg))

    # A wall or low object can be the largest plane. Inspect several candidates
    # and prefer a valid plane closest to the nominal ground at base_link.
    best_plane = None
    for _ in range(4):
        if len(ground_search_cloud.points) < minimum_inliers:
            break
        # Plane coefficients are [a, b, c, d] for ax + by + cz + d = 0.
        plane_coefficients, plane_inlier_indices = ground_search_cloud.segment_plane(
            distance_threshold=config.lidar_ground_distance_tolerance_m,
            ransac_n=3, num_iterations=500)
        if len(plane_inlier_indices) < minimum_inliers:
            break
        plane_normal_in_base = np.asarray(plane_coefficients[:3], dtype=float)
        normal_length = np.linalg.norm(plane_normal_in_base)
        if np.isfinite(normal_length) and normal_length > 1e-9 and np.isfinite(plane_coefficients[3]):
            plane_normal_in_base /= normal_length
            plane_offset_m = plane_coefficients[3] / normal_length
            if np.dot(plane_normal_in_base, up_direction_in_base) < 0:
                plane_normal_in_base, plane_offset_m = -plane_normal_in_base, -plane_offset_m
            normal_alignment = float(np.clip(np.dot(plane_normal_in_base, up_direction_in_base), -1.0, 1.0))
            # Set x = y = 0 to find the plane height under the base_link origin.
            plane_height_at_base_m = (
                -plane_offset_m / plane_normal_in_base[2]
                if abs(plane_normal_in_base[2]) > 1e-9 else float("inf")
            )
            if (normal_alignment >= min_normal_alignment
                    and abs(plane_height_at_base_m) <= config.lidar_ground_search_height_m):
                plane_score = (abs(plane_height_at_base_m), -len(plane_inlier_indices))
                if best_plane is None or plane_score < best_plane[0]:
                    best_plane = (
                        plane_score, plane_normal_in_base, plane_offset_m, len(plane_inlier_indices),
                        plane_height_at_base_m, math.degrees(math.acos(normal_alignment)),
                    )
        ground_search_cloud = ground_search_cloud.select_by_index(plane_inlier_indices, invert=True)
    if best_plane is None:
        raise ValueError("no reliable LiDAR ground plane near base_link")
    _, plane_normal_in_base, plane_offset_m, ground_inlier_count, plane_height_at_base_m, plane_tilt_deg = best_plane
    # Remove the estimated road and everything below it. The positive tolerance
    # also removes noisy road points just above the plane.
    ground_point_mask = (
        points_in_base_m @ plane_normal_in_base + plane_offset_m
    ) <= config.lidar_ground_distance_tolerance_m
    return (
        lidar_cloud.select_by_index(np.flatnonzero(~ground_point_mask).tolist()),
        int(np.count_nonzero(ground_point_mask)), ground_inlier_count, plane_height_at_base_m, plane_tilt_deg,
    )


def up_in_cloud_frame(tf_buffer, cloud_frame_id, upright_frame_id):
    # Rotate +Z from base_link or the camera body into the cloud frame using TF.
    # The camera's body frame supplies its own approximate upright direction.
    if cloud_frame_id == upright_frame_id:
        return np.array([0.0, 0.0, 1.0])
    cloud_from_upright = tf_buffer.lookup_transform(cloud_frame_id, upright_frame_id, Time())
    return transform_matrix(cloud_from_upright.transform)[:3, 2]
