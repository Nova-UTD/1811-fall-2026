# ROS orchestration, automatic calibration state, and live vehicle model updates.

# Author: Khoa Nguyen
# Email: minhkhoa.ee@outlook.com
# Copyright: 2021, Nova UTD
# License: MIT License

import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from threading import Lock

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from interactive_calibration import CalibrationAlgorithms, InteractiveCalibration
from lidar_camera_algorithms import (
    confirm_initial_pose, estimate_initial_pose, pose_difference, prepare_fpfh,
    register_camera,
)
from lidar_camera_clouds import (
    crop_calibration_cloud, lidar_up_from_imu, pack_registered_cloud,
    point_cloud_from_message, remove_lidar_ground, remove_wall_patches,
    remove_zed_ground, up_in_cloud_frame,
)
from lidar_camera_config import CAMERAS, is_jetson_link, load_registration_config
from lidar_camera_diagnostics import (
    cloud_diagnostics, cloud_processing_failure_diagnostics,
    cloud_summary_unavailable_diagnostics, icp_quality_diagnostics,
    lidar_ground_diagnostics, log_fpfh_diagnostics, log_fpfh_preparation_started,
    log_lidar_fpfh_preparation_time, ransac_quality_diagnostics,
    wall_filter_diagnostics, zed_ground_diagnostics,
)
from lidar_camera_mounts import (
    calibrated_camera_mounts, camera_mount_matrix, save_calibrated_urdf,
    update_camera_joints,
)
from lidar_camera_timing import matching_floor_plane, pair_error
from lidar_perception_transforms import transform_matrix

REGISTERED_CLOUD_TOPIC = "/lidar_perception/registered_points"  # RViz reads the merged cloud here.
CALIBRATION_INPUT_PREFIX = "/lidar_perception/calibration_input"
_FLOOR_PLANE_UNSET = object()


# Main node for LiDAR and camera registration on the Jetson.
class LidarCameraRegistration(Node):
    # Receive sensor data, run calibration, and apply accepted camera mounts.

    def __init__(self):
        if not is_jetson_link():
            raise RuntimeError("LiDAR and camera registration only runs on the Jetson (192.168.100.2)")
        super().__init__("lidar_camera_registration")
        self.config = load_registration_config(self)

        # Separate callbacks keep receiving current clouds while FPFH/RANSAC runs.
        self.camera_group = MutuallyExclusiveCallbackGroup()
        self.lidar_group = MutuallyExclusiveCallbackGroup()
        self.calibration_group = MutuallyExclusiveCallbackGroup()
        self.description_group = MutuallyExclusiveCallbackGroup()
        # Sensor callbacks and the calibration timer run on separate threads.
        self.data_lock = Lock()  # Protect incoming messages and receipt times.
        self.state_lock = Lock()  # Protect poses, confirmations, and URDF updates.
        self.latest_lidar = None  # (message, local receipt time)
        # Measure existing subscriptions; do not duplicate large cloud streams for the UI.
        self.cloud_receipts = {name: deque(maxlen=120) for name in ("lidar", *CAMERAS)}
        self.imu_samples = deque(maxlen=512)  # (message, local receipt time)
        self.camera_clouds = {}
        # A small history allows independently delivered plane and cloud messages
        # to be paired by their exact camera image timestamp.
        self.floor_planes = {name: deque(maxlen=64) for name in CAMERAS}
        self.camera_poses = {}  # name -> (LiDAR frame, cloud frame, camera-to-LiDAR matrix)
        self.camera_anchors = {}  # The accepted global pose, used to limit later drift.
        self.pending_candidates = {}  # Unpublished global estimates awaiting confirmation.
        self.camera_mounts = {}  # name -> calibrated base-from-camera-body matrix
        self.camera_failures = {}  # Consecutive ICP failures per camera.
        self.retry_global = set()  # Cameras that need a new FPFH/RANSAC search.
        self.robot_description = None
        # Track the request so an old response cannot replace a newer pose.
        self.update_in_flight = False
        self.pending_update = None
        self.update_started = 0.0
        self.next_calibration = 0.0
        self.output_urdf = Path.home() / ".ros/lidar_camera_calibration/vehicle_1811_calibrated.urdf"
        self.saved_camera_mounts = {}
        self.restored_cameras = set()
        self.restore_retry_at = 0.0
        if self.config.restore_calibration_on_startup:
            try:
                self.saved_camera_mounts = calibrated_camera_mounts(self.output_urdf.read_text(encoding="utf-8"))
            except FileNotFoundError:
                pass  # First calibration on this Jetson.
            except (OSError, ValueError, ET.ParseError) as exc:
                self.get_logger().error(f"Could not load saved calibration {self.output_urdf}: {exc}")

        # QoS (Quality of Service) controls how ROS sends and receives messages.
        # depth=1 keeps the latest message in the ROS queue when messages arrive faster than they can be processed.
        # BEST_EFFORT allows a message to be dropped without retrying it, which helps keep large point clouds current.
        latest_scan = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        # RELIABLE delivery requests retransmission when a message is lost.
        # TRANSIENT_LOCAL lets this subscriber receive the last retained robot description when it connects after the publisher.
        # The publisher must offer compatible QoS and still be running for the retained message to be available.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # create_publisher(message_type, topic_name, qos) creates an object that sends messages to a ROS topic.
        # PointCloud2 is the ROS message type used to store a point cloud and its frame and timestamp.
        # REGISTERED_CLOUD_TOPIC is "/lidar_perception/registered_points" and latest_scan is the QoS defined above.
        # publish_registered_inputs() combines the LiDAR cloud and usable camera clouds into one colored cloud for RViz.
        # Each camera cloud is transformed into the LiDAR message frame using that camera's previously accepted pose.
        # Example: a front camera with an accepted pose can appear alongside the LiDAR in this one topic.
        # The published cloud is empty when no camera has both a usable cloud and an accepted pose.
        self.registered_pub = self.create_publisher(PointCloud2, REGISTERED_CLOUD_TOPIC, latest_scan)

        # self.input_pubs is a dictionary that stores one PointCloud2 publisher for each sensor name.
        # CAMERAS = ("front", "left", "rear", "right"), and *CAMERAS adds those four names after "lidar".
        # The loop runs once for each name: "lidar", "front", "left", "rear", and "right".
        # CALIBRATION_INPUT_PREFIX is "/lidar_perception/calibration_input".
        # Example: when name = "front", the f-string creates the topic "/lidar_perception/calibration_input/front".
        # Example: self.input_pubs["front"] is the publisher for the front camera's filtered cloud.
        # prepare_calibration_input() publishes each sensor's cropped and filtered cloud using its original frame and timestamp.
        # If wall filtering is enabled, the displayed cloud is downsampled and wall-filtered before publishing.
        # If wall filtering is disabled, the displayed cloud keeps full resolution after cropping and any enabled ground filter.
        # Before a camera pose is accepted, set RViz's fixed frame to that camera cloud's frame to inspect it without a camera-to-base TF.
        self.input_pubs = {
            name: self.create_publisher(PointCloud2, f"{CALIBRATION_INPUT_PREFIX}/{name}", latest_scan)
            for name in ("lidar", *CAMERAS)
        }
        # create_subscription(message_type, topic_name, callback, qos) calls the callback when a message arrives.
        # This subscription receives raw LiDAR PointCloud2 messages from "/ouster/points" in the frame named by msg.header.frame_id.
        # pcd_callback() stores the latest LiDAR message and its local receipt time in self.latest_lidar.
        # calibration_tick() later prepares that message as the reference cloud for camera-to-LiDAR registration.
        # callback_group=self.lidar_group prevents this callback and the IMU callback from running at the same time.
        # The ROS node owns its subscriptions and timer; no second list is needed.
        self.create_subscription(
            PointCloud2, "/ouster/points", self.pcd_callback, latest_scan,
            callback_group=self.lidar_group,
        )
        if self.config.lidar_ground_filter_enabled:
            # This subscription is created only when lidar_ground_filter_enabled is true.
            # "/ouster/imu" sends Imu messages with linear acceleration in m/s^2 and angular velocity in rad/s.
            # imu_callback() stores recent messages and their local receipt times in self.imu_samples.
            # The LiDAR ground filter selects fresh IMU samples near the LiDAR timestamp to estimate the up direction.
            # The filter checks for low angular velocity and acceleration near gravity before using those samples.
            # imu_qos uses depth=100 so the ROS subscription queue can hold multiple IMU messages.
            imu_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(
                Imu, "/ouster/imu", self.imu_callback, imu_qos,
                callback_group=self.lidar_group,
            )
        # floor_qos uses depth=64 because floor-plane messages can arrive separately from their matching camera clouds.
        floor_qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
        for camera in CAMERAS:
            # This loop creates one PointCloud2 subscription for each of the four ZED cameras.
            # Example: when camera = "front", topic = "/zed_front/zed_node/point_cloud/cloud_registered".
            # Each cloud is expressed in the camera frame named by msg.header.frame_id and is not yet aligned with the LiDAR.
            # camera_callback(name, msg) stores the latest message and receipt time under that camera's name.
            # When interactive calibration is enabled, the callback also sends the message to the Hub preview code.
            topic = f"/zed_{camera}/zed_node/point_cloud/cloud_registered"
            # lambda receives msg and calls camera_callback(name, msg) when a cloud arrives.
            # name=camera saves the current camera name when the callback is created.
            # Example: the front subscription calls camera_callback("front", msg), even after the loop moves to "left".
            self.create_subscription(
                PointCloud2, topic, lambda msg, name=camera: self.camera_callback(name, msg),
                latest_scan, callback_group=self.camera_group,
            )
            if self.config.ground_filter_enabled:
                # These four subscriptions are created only when ground_filter_enabled is true.
                # Example: the front camera sends a PoseStamped floor estimate on "/zed_front/zed_node/floor_plane".
                # The pose position is the plane center in the message frame, and its local +Z axis is the plane normal.
                # floor_callback(name, msg) stores the message in self.floor_planes under the corresponding camera name.
                # The ground filter uses the plane with the same timestamp and frame as the camera cloud to remove ground points.
                self.create_subscription(
                    PoseStamped, f"/zed_{camera}/zed_node/floor_plane",
                    lambda msg, name=camera: self.floor_callback(name, msg), floor_qos,
                    callback_group=self.camera_group,
                )

        # robot_state_publisher sends the current vehicle model as a String message on "/robot_description".
        # The String contains expanded URDF XML, including the vehicle links and camera joints.
        # description_callback() stores this model so the node can merge accepted camera mounts into the current URDF.
        # The callback also checks whether accepted mounts need to be restored after robot_state_publisher restarts.
        # robot_state_publisher publishes the vehicle's base-to-camera transforms after it accepts the updated URDF.
        self.create_subscription(
            String, "/robot_description", self.description_callback, latched,
            callback_group=self.description_group,
        )
        # This service client sends a request to update the robot_description parameter on robot_state_publisher.
        self.description_client = self.create_client(
            SetParameters, "/robot_state_publisher/set_parameters",
            callback_group=self.description_group,
        )
        # TransformListener creates subscriptions to "/tf" and "/tf_static" and stores their transforms in self.tf_buffer.
        # The node looks up these transforms when cropping sensor clouds and converting camera cloud poses into vehicle camera mounts.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.hub = None
        if self.config.interactive_calibration:
            algorithms = CalibrationAlgorithms(
                prepare_fpfh, log_fpfh_diagnostics, estimate_initial_pose, register_camera,
                camera_mount_matrix, confirm_initial_pose, update_camera_joints, save_calibrated_urdf)
            self.hub = InteractiveCalibration(self, CAMERAS, algorithms)
            for name in self.saved_camera_mounts:
                self.hub.reasons[name] = "Restoring saved calibration"
        # Poll for the first usable scan promptly; later attempts use the configured interval.
        self.create_timer(0.5, self.calibration_tick, callback_group=self.calibration_group)
        self.get_logger().info(
            f"LiDAR and camera registration started on Jetson — "
            f"ZED floor filter: {self.config.ground_filter_enabled}, "
            f"LiDAR ground filter: {self.config.lidar_ground_filter_enabled}, "
            f"wall filter: {self.config.wall_filter_enabled}, "
            f"min fitness (RANSAC {self.config.ransac_min_fitness:.2f}, "
            f"ICP {self.config.icp_min_fitness:.2f}), "
            f"RANSAC max distance: {self.config.ransac_max_distance_m:.2f} m, "
            f"pose confirmation: {self.config.initial_pose_confirmation_enabled}, "
            f"hub approval: {self.config.interactive_calibration}.")

    def description_callback(self, msg):
        # msg: a ROS String whose data contains the current robot_description URDF.
        with self.state_lock:
            self.robot_description = msg.data
            if self.camera_mounts and not self.update_in_flight:
                try:
                    current = calibrated_camera_mounts(msg.data)
                except (ValueError, ET.ParseError):
                    return  # The normal update path reports an invalid live model.
                # Karbon can restart independently while this Jetson keeps running.
                missing = {name: mount.copy() for name, mount in self.camera_mounts.items()
                           if name not in current}
                if missing:
                    self.saved_camera_mounts.update(missing)
                    self.restore_retry_at = 0.0

    def restore_saved_calibration(self):
        # Replay accepted mounts through the same Karbon service as calibration.
        now = time.monotonic()
        with self.state_lock:
            if not self.saved_camera_mounts or self.update_in_flight or now < self.restore_retry_at:
                return
            description = self.robot_description
            if not description or not self.description_client.service_is_ready():
                self.get_logger().warning("Waiting for vehicle robot_description to restore saved calibration.",
                                          throttle_duration_sec=30.0)
                return
            mounts = {name: matrix.copy() for name, matrix in self.saved_camera_mounts.items()}
            self.restore_retry_at = now + 2.0
            self.update_in_flight = True
            self.update_started = now
        try:
            # Copy only camera mounts. Keep current wheel, LiDAR and body geometry.
            description = update_camera_joints(description, mounts)
            request = SetParameters.Request()
            request.parameters = [Parameter("robot_description", Parameter.Type.STRING,
                                            description).to_parameter_msg()]
            future = self.description_client.call_async(request)
            with self.state_lock:
                self.pending_update = future
            future.add_done_callback(lambda result: self.restore_update_done(result, description, mounts))
        except Exception as exc:
            with self.state_lock:
                self.update_in_flight = False
                self.pending_update = None
            self.get_logger().error(f"Could not restore saved calibration; will retry: {exc}")

    def restore_update_done(self, future, description, mounts):
        with self.state_lock:
            if self.pending_update is not future:
                return
        try:
            result = future.result()
            if len(result.results) != 1 or not result.results[0].successful:
                reason = result.results[0].reason if result.results else "no parameter result"
                raise RuntimeError(reason)
            with self.state_lock:
                self.robot_description = description
                newly_restored = set(mounts).difference(self.camera_mounts)
                self.camera_mounts.update(mounts)
                self.restored_cameras.update(mounts)
                self.saved_camera_mounts.clear()
                self.next_calibration = 0.0
                if self.hub is not None:
                    # Reapplying after a Karbon restart must not cancel Recompute.
                    self.hub.accepted.update(newly_restored)
                    for name in mounts:
                        if name in self.hub.accepted:
                            self.hub.reasons[name] = "Restored saved calibration"
            self.get_logger().info("Restored saved camera calibration: " + ", ".join(mounts))
        except Exception as exc:
            self.get_logger().error(f"Vehicle rejected saved calibration; will retry: {exc}")
        finally:
            with self.state_lock:
                self.pending_update = None
                self.update_in_flight = False

    def seed_restored_pose(self, name, msg):
        # Recover cloud -> LiDAR from the persisted body mount and current driver
        # frames. This is matrix composition, with no FPFH/RANSAC or ICP work.
        with self.data_lock:
            lidar = self.latest_lidar
        if lidar is None:
            return
        frames = (lidar[0].header.frame_id, msg.header.frame_id)
        with self.state_lock:
            cached = self.camera_poses.get(name)
            mount = self.camera_mounts.get(name)
            if mount is None or (cached is not None and cached[:2] == frames):
                return
            mount = mount.copy()
        try:
            # Resolve each internal transform independently of the restored TF.
            base_from_lidar = (np.eye(4) if frames[0] == "base_link" else transform_matrix(
                self.tf_buffer.lookup_transform("base_link", frames[0], Time()).transform))
            body = f"zed_{name}_camera_link"
            camera_from_body = (np.eye(4) if frames[1] == body else transform_matrix(
                self.tf_buffer.lookup_transform(frames[1], body, Time()).transform))
            pose = np.linalg.inv(base_from_lidar) @ mount @ np.linalg.inv(camera_from_body)
        except TransformException:
            return  # The drivers may still be publishing their internal TFs.
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.get_logger().warning(f"Could not restore {name} cloud transform: {exc}",
                                      throttle_duration_sec=30.0)
            return
        with self.state_lock:
            # A newly accepted pose takes precedence over this callback's snapshot.
            if np.array_equal(self.camera_mounts.get(name), mount):
                self.camera_poses[name] = (*frames, pose)
                self.camera_anchors[name] = (*frames, pose.copy())

    def camera_callback(self, camera_name, cloud_message):
        # Save the latest PointCloud2 for this camera with a local receipt time.
        with self.data_lock:
            received_monotonic_s = time.monotonic()
            self.camera_clouds[camera_name] = (cloud_message, received_monotonic_s)
            self.cloud_receipts[camera_name].append(received_monotonic_s)
        if camera_name in getattr(self, "restored_cameras", ()):
            self.seed_restored_pose(camera_name, cloud_message)
        if self.config.interactive_calibration:
            try:
                self.hub.publish_cloud(camera_name, cloud_message)
            except Exception as exc:
                self.get_logger().warning(
                    f"Could not publish hub preview cloud for {camera_name}: {exc}",
                    throttle_duration_sec=30.0,
                )

    def floor_callback(self, camera_name, floor_plane_message):
        # Keep each camera's plane separate, including when its cloud arrives later.
        with self.data_lock:
            self.floor_planes[camera_name].append(floor_plane_message)

    def pcd_callback(self, lidar_message):
        # Record when the LiDAR sample was received on the Jetson.
        with self.data_lock:
            received_monotonic_s = time.monotonic()
            self.latest_lidar = (lidar_message, received_monotonic_s)
            self.cloud_receipts["lidar"].append(received_monotonic_s)

    def imu_callback(self, imu_message):
        # Keep a short timestamped history so a scan can use its own IMU samples.
        with self.data_lock:
            self.imu_samples.append((imu_message, time.monotonic()))

    def prepare_calibration_input(self, sensor_name, cloud_message, floor_plane=_FLOOR_PLANE_UNSET):
        # sensor_name selects the LiDAR or a ZED. Keep points inside its crop
        # box before filtering. Return separate matching and display clouds.
        started_monotonic_s = time.monotonic()
        imu_samples = ()
        if sensor_name == "lidar" and self.config.lidar_ground_filter_enabled:
            with self.data_lock:
                imu_samples = tuple(self.imu_samples)
        full_resolution_cloud = point_cloud_from_message(cloud_message)
        valid_count = len(full_resolution_cloud.points)
        if sensor_name == "lidar":
            crop_reference_frame = "base_link"
            crop_min_m, crop_max_m = self.config.lidar_box_min_m, self.config.lidar_box_max_m
        else:
            crop_reference_frame = f"zed_{sensor_name}_camera_link"
            crop_min_m, crop_max_m = self.config.camera_box_min_m, self.config.camera_box_max_m
        try:
            full_resolution_cloud = crop_calibration_cloud(
                full_resolution_cloud, self.tf_buffer, cloud_message.header.frame_id,
                crop_reference_frame, crop_min_m, crop_max_m,
            )
            cropped_count = len(full_resolution_cloud.points)
            if sensor_name == "lidar" and self.config.lidar_ground_filter_enabled:
                up_in_base_frame, imu_sample_count = lidar_up_from_imu(
                    imu_samples, cloud_message.header, self.tf_buffer, started_monotonic_s, self.config,
                )
                (
                    full_resolution_cloud, ground_points_removed, ground_plane_inlier_count,
                    ground_height_m, ground_tilt_deg,
                ) = remove_lidar_ground(
                    full_resolution_cloud, cloud_message.header.frame_id,
                    self.tf_buffer, up_in_base_frame, self.config,
                )
                if self.config.cloud_diagnostics_enabled:
                    self.get_logger().info(lidar_ground_diagnostics(
                        cropped_count, ground_points_removed, len(full_resolution_cloud.points),
                        ground_plane_inlier_count, ground_height_m, ground_tilt_deg,
                        imu_sample_count, self.config.lidar_ground_distance_tolerance_m))
            elif sensor_name != "lidar" and self.config.ground_filter_enabled:
                if floor_plane is _FLOOR_PLANE_UNSET:
                    with self.data_lock:
                        floor_plane = matching_floor_plane(
                            self.floor_planes[sensor_name], cloud_message.header.stamp,
                        )
                full_resolution_cloud, ground_points_removed = remove_zed_ground(
                    full_resolution_cloud, cloud_message.header, floor_plane, self.config.ground_distance_tolerance_m)
                if self.config.cloud_diagnostics_enabled:
                    self.get_logger().info(zed_ground_diagnostics(
                        sensor_name, cropped_count, ground_points_removed, len(full_resolution_cloud.points),
                        self.config.ground_distance_tolerance_m))
            matching_cloud = full_resolution_cloud.voxel_down_sample(self.config.icp_voxel_size_m)
            voxel_count = len(matching_cloud.points)
            wall_count, wall_points_removed = None, 0
            if self.config.wall_filter_enabled:
                up_in_sensor_frame = up_in_cloud_frame(
                    self.tf_buffer, cloud_message.header.frame_id, crop_reference_frame,
                )
                matching_cloud, wall_count, wall_points_removed = remove_wall_patches(
                    matching_cloud, up_in_sensor_frame, self.config,
                )
                self.get_logger().debug(wall_filter_diagnostics(
                    sensor_name, wall_count, wall_points_removed, len(matching_cloud.points),
                    time.monotonic() - started_monotonic_s))
        except Exception as exc:
            if self.config.cloud_diagnostics_enabled:
                self.get_logger().info(cloud_processing_failure_diagnostics(sensor_name, cloud_message, valid_count, exc))
            raise
        if self.config.cloud_diagnostics_enabled:
            try:
                self.get_logger().info(cloud_diagnostics(
                    sensor_name, cloud_message, valid_count, cropped_count, voxel_count,
                    matching_cloud, wall_count, wall_points_removed))
            except Exception as exc:
                self.get_logger().info(cloud_summary_unavailable_diagnostics(sensor_name, exc))
        display_cloud = matching_cloud if self.config.wall_filter_enabled else full_resolution_cloud
        self.input_pubs[sensor_name].publish(
            pack_registered_cloud([(sensor_name, display_cloud)], cloud_message.header, self.config),
        )
        return matching_cloud, display_cloud

    def publish_registered_inputs(self, header, lidar, cameras, reasons, poses):
        # These display clouds are wall-filtered when enabled, full resolution
        # otherwise. Only previously accepted camera poses are used in the overlay.
        registered = []
        if lidar is not None:
            for name, (msg, cloud) in cameras.items():
                cached = poses.get(name)
                if reasons.get(name) or cached is None or cached[:2] != (header.frame_id, msg.header.frame_id):
                    continue
                camera = o3d.geometry.PointCloud(cloud)
                camera.transform(cached[2])
                registered.append((name, camera))
        clouds = [("lidar", lidar), *registered] if registered else []
        if self.config.visualization_voxel_size_m > 0:
            clouds = [(name, cloud.voxel_down_sample(self.config.visualization_voxel_size_m))
                      for name, cloud in clouds]
        self.registered_pub.publish(pack_registered_cloud(clouds, header, self.config))

    def calibration_tick(self):
        # Process one snapshot at the configured interval. Publish its display
        # inputs first, even if timestamps or the vehicle URDF block calibration.
        if getattr(self, "saved_camera_mounts", None):
            self.restore_saved_calibration()
            return  # Restore first, even while interactive calibration is stopped.
        interactive_mode = self.config.interactive_calibration
        # The Hub skips accepted cameras and cameras waiting for approval to be applied.
        skipped_camera_names = set()
        calibration_session = None
        if interactive_mode:
            skipped_camera_names, calibration_session = self.hub.calibration_snapshot()
            if skipped_camera_names is None or len(skipped_camera_names) == len(CAMERAS):
                return
        now_monotonic_s = time.monotonic()
        if now_monotonic_s < self.next_calibration:
            return
        self.next_calibration = now_monotonic_s + self.config.calibration_interval_s

        with self.data_lock:
            lidar_sample = self.latest_lidar
            # Each snapshot entry is (PointCloud2 message, monotonic receipt time).
            camera_samples = {}
            for camera_name, camera_sample in self.camera_clouds.items():
                if camera_name not in skipped_camera_names:
                    camera_samples[camera_name] = camera_sample
            # Keep the matching plane with each cloud snapshot while slow cloud
            # processing and incoming callbacks continue independently.
            floor_planes_by_camera = {}
            if self.config.ground_filter_enabled:
                for camera_name, (camera_message, _) in camera_samples.items():
                    floor_planes_by_camera[camera_name] = matching_floor_plane(
                        self.floor_planes[camera_name], camera_message.header.stamp,
                    )
        with self.state_lock:
            robot_description = self.robot_description
            cached_camera_poses = self.camera_poses.copy()
            accepted_camera_mounts = self.camera_mounts.copy()
            cameras_needing_global_search = self.retry_global.copy()
            urdf_update_in_flight = self.update_in_flight
            pending_urdf_update = self.pending_update
            urdf_update_started_s = self.update_started

        lidar_message = lidar_sample[0] if lidar_sample else None
        lidar_matching_cloud = None
        lidar_display_cloud = None
        prepared_camera_clouds = {}
        camera_displays = {}
        input_errors_by_camera = {}
        lidar_input_failed = False

        # Each input keeps its original frame and timestamp. The displayed LiDAR
        # can be viewed in base_link even when every camera calibration fails.
        # lidar_sample[1] is the local receipt time, in seconds.
        if lidar_sample and now_monotonic_s - lidar_sample[1] <= self.config.max_cloud_age_s:
            try:
                lidar_matching_cloud, lidar_display_cloud = self.prepare_calibration_input(
                    "lidar", lidar_message,
                )
            except Exception as exc:
                self.input_pubs["lidar"].publish(pack_registered_cloud([], lidar_message.header, self.config))
                lidar_input_failed = True
                self.get_logger().warning(f"LiDAR calibration input failed: {exc}",
                                          throttle_duration_sec=30.0)
        elif lidar_message:
            self.input_pubs["lidar"].publish(pack_registered_cloud([], lidar_message.header, self.config))

        if interactive_mode and not self.hub.session_active(calibration_session):
            return
        # Prepare every fresh camera before RANSAC. This makes all
        # camera inputs available even if one camera's RANSAC takes a long time.
        for camera_name, camera_sample in camera_samples.items():
            if interactive_mode and not self.hub.session_active(calibration_session):
                return
            camera_message, received_monotonic_s = camera_sample
            if now_monotonic_s - received_monotonic_s > self.config.max_cloud_age_s:
                self.input_pubs[camera_name].publish(pack_registered_cloud([], camera_message.header, self.config))
                continue
            try:
                camera_matching_cloud, camera_display_cloud = self.prepare_calibration_input(
                    camera_name, camera_message, floor_planes_by_camera.get(camera_name),
                )
                prepared_camera_clouds[camera_name] = (camera_message, camera_matching_cloud)
                camera_displays[camera_name] = (camera_message, camera_display_cloud)
            except Exception as exc:
                self.input_pubs[camera_name].publish(pack_registered_cloud([], camera_message.header, self.config))
                input_errors_by_camera[camera_name] = f"calibration input failed: {exc}"

        # A None entry means the camera passed the timestamp checks.
        rejection_reasons = {
            camera_name: pair_error(
                lidar_message, camera_samples.get(camera_name), now_monotonic_s, self.config,
            ) if lidar_matching_cloud is not None else "no usable LiDAR cloud"
            for camera_name in CAMERAS
        }
        rejection_reasons.update(input_errors_by_camera)
        for camera_name, (_, camera_matching_cloud) in prepared_camera_clouds.items():
            if not rejection_reasons[camera_name] and len(camera_matching_cloud.points) < self.config.min_points:
                rejection_reasons[camera_name] = (
                    f"too few camera points after filtering ({len(camera_matching_cloud.points)})"
                )
        if interactive_mode:
            self.hub.note_reasons(rejection_reasons, skipped_camera_names, calibration_session)
        usable_camera_clouds = {
            camera_name: prepared_camera_clouds[camera_name]
            for camera_name in prepared_camera_clouds if not rejection_reasons[camera_name]
        }
        if lidar_message:
            try:
                self.publish_registered_inputs(
                    lidar_message.header, lidar_display_cloud, camera_displays,
                    rejection_reasons, cached_camera_poses,
                )
            except Exception as exc:
                self.get_logger().error(f"Could not publish registration overlay: {exc}")

        # Wait for the previous URDF update before sending another one; otherwise
        # an older response could replace a newer calibration.
        if urdf_update_in_flight and not interactive_mode:
            if now_monotonic_s - urdf_update_started_s <= max(30.0, 3 * self.config.calibration_interval_s):
                return  # Never send overlapping robot_description updates.

            # Stop waiting if the service does not respond (30 s by default).
            # Recheck under the lock in case the response arrived just now.
            expired = False
            with self.state_lock:
                if self.update_in_flight and self.pending_update is pending_urdf_update:
                    self.pending_update = None
                    self.update_in_flight = False
                    expired = True
            if expired:
                # Cancel the local wait and allow a later timer tick to retry.
                if pending_urdf_update is not None:
                    pending_urdf_update.cancel()
                self.get_logger().warning("URDF update timed out waiting for robot_state_publisher; will retry on the next pass.")
            return

        if lidar_matching_cloud is None:
            if not lidar_input_failed:
                self.get_logger().warning("Waiting for a fresh LiDAR point cloud on /ouster/points before calibrating.",
                                          throttle_duration_sec=30.0)
            return
        if len(lidar_matching_cloud.points) < self.config.min_points:
            self.get_logger().warning(f"Too few LiDAR points after filtering ({len(lidar_matching_cloud.points)} remaining) — skipping calibration pass.",
                                      throttle_duration_sec=30.0)
            return
        if not usable_camera_clouds:
            details = "; ".join(
                f"{camera_name}: {rejection_reasons[camera_name]}"
                for camera_name in CAMERAS if rejection_reasons[camera_name] != "no cloud"
            )
            message = "Waiting for ZED clouds" + (f": {details}" if details else "")
            self.get_logger().warning(message, throttle_duration_sec=30.0)
            return

        # A pose must be written into the live vehicle URDF before it can supply TF.
        # Wait until both the current description and its update service are available.
        if not interactive_mode and (not robot_description or not self.description_client.service_is_ready()):
            self.get_logger().warning("Waiting for vehicle robot_description and parameter service from the Karbon.",
                                      throttle_duration_sec=30.0)
            return

        try:
            if interactive_mode:
                self.hub.calibrate(
                    lidar_matching_cloud, lidar_message.header, usable_camera_clouds,
                    rejection_reasons, calibration_session,
                )
            else:
                self.calibrate(
                    lidar_matching_cloud, lidar_message.header, usable_camera_clouds,
                    rejection_reasons, cached_camera_poses, accepted_camera_mounts,
                    cameras_needing_global_search, robot_description,
                )
        except Exception as exc:
            # Report the failure; a later timer tick can try again.
            self.get_logger().error(f"Calibration failed: {exc}")

    def calibrate(self, lidar, header, cameras, reasons, poses, previous_mounts, retry_global, description):
        # lidar: the downsampled Open3D LiDAR cloud used as the fixed reference.
        # header: the LiDAR message header, including its frame ID and timestamp.
        # cameras: names mapped to (original ROS message, filtered Open3D cloud).
        # reasons: camera names mapped to a pairing error, or None when usable.
        # poses: names mapped to (LiDAR frame, camera frame, camera-to-LiDAR matrix).
        # previous_mounts: accepted camera-body-to-base_link matrices by camera name.
        # retry_global: cameras needing a new FPFH/RANSAC guess before ICP.
        # description: the current expanded vehicle URDF as an XML string.
        lidar_features = None
        updated = {}
        mounts = {}
        confirmed_names = set()
        skipped = []
        log_quality = (self.get_logger().info if self.config.cloud_diagnostics_enabled
                       else self.get_logger().debug)

        def global_failure(name, frames, reason):
            # A failed search is not a new estimate. Keep this camera's earlier
            # valid observations so an intermittent failure does not erase 1/3.
            with self.state_lock:
                previous = self.pending_candidates.get(name)
                count = (len(previous["poses"])
                         if previous is not None and previous["frames"] == frames else 0)
            if count:
                reason += f"; keeping {count}/{self.config.initial_pose_confirmations} confirmations"
            return reason

        for name in CAMERAS:
            if name not in cameras:
                if reasons[name] != "no cloud":
                    skipped.append(f"{name}: {reasons[name]}")
                continue
            msg, camera = cameras[name]
            frames = (header.frame_id, msg.header.frame_id)
            cached = poses.get(name)
            try:
                global_search = cached is None or cached[:2] != frames or name in retry_global
                if global_search:
                    # A changed cloud frame changes the meaning of the pose.
                    # Never reuse this camera's observations from another frame.
                    with self.state_lock:
                        previous = self.pending_candidates.get(name)
                        if previous is not None and previous["frames"] != frames:
                            self.pending_candidates.pop(name)
                    if lidar_features is None:
                        started = time.monotonic()
                        log_fpfh_preparation_started(self.get_logger(), "lidar")
                        lidar_features = prepare_fpfh(lidar, self.config)
                        if self.config.cloud_diagnostics_enabled:
                            reason = ("not computed: too few points after FPFH downsampling"
                                      if lidar_features[0] is None else None)
                            log_fpfh_diagnostics(self.get_logger(), "lidar", lidar_features[1], reason)
                        log_lidar_fpfh_preparation_time(self.get_logger(), time.monotonic() - started)
                    # No manual ZED mount enters FPFH/RANSAC: geometry gives the initial pose.
                    metrics = {}
                    started = time.monotonic()
                    log_fpfh_preparation_started(self.get_logger(), name)
                    report_features = None
                    if self.config.cloud_diagnostics_enabled:
                        report_features = lambda features, why: log_fpfh_diagnostics(
                            self.get_logger(), name, features, why)
                    initial_pose, reason = estimate_initial_pose(
                        camera, lidar_features, self.config, metrics, report_features)
                    if metrics:
                        log_quality(ransac_quality_diagnostics(
                            name, metrics, self.config, time.monotonic() - started))
                    if initial_pose is None:
                        skipped.append(f"{name}: {global_failure(name, frames, reason)}")
                        continue
                else:
                    initial_pose = cached[2]
                metrics = {}
                pose, reason = register_camera(camera, lidar, initial_pose, self.config, metrics)
                if metrics:
                    log_quality(icp_quality_diagnostics(name, metrics, self.config))

                # Keep small ICP updates near the accepted global mount.
                # Comparing with the original anchor also prevents many small
                # updates from drifting a long way over time.
                if pose is not None and not global_search:
                    with self.state_lock:
                        anchor = self.camera_anchors.get(name)
                    if anchor is not None and anchor[:2] == frames:
                        translation, angle = pose_difference(pose, anchor[2])
                        if (translation > self.config.pose_agreement_translation_m
                                or angle > self.config.pose_agreement_rotation_deg):
                            pose = None
                            reason = f"ICP drift from confirmed pose too large ({translation:.2f} m, {angle:.1f} deg)"
                if pose is None:
                    if global_search:
                        reason = global_failure(name, frames, reason)
                    else:
                        with self.state_lock:
                            failures = self.camera_failures.get(name, 0) + 1
                            self.camera_failures[name] = failures
                            if failures >= self.config.max_icp_failures:
                                self.retry_global.add(name)
                                reason += "; next attempt will retry FPFH/RANSAC"
                    skipped.append(f"{name}: {reason}")
                    continue

                if global_search and self.config.initial_pose_confirmation_enabled:
                    # RANSAC is random: require fresh, independently computed
                    # estimates to agree before publishing the first pose.
                    with self.state_lock:
                        confirmed, reason = confirm_initial_pose(
                            self.pending_candidates, name, frames, pose,
                            header.stamp, msg.header.stamp, self.config)
                    if not confirmed:
                        skipped.append(f"{name}: {reason}")
                        continue
                with self.state_lock:
                    self.camera_failures.pop(name, None)  # ICP succeeded, even if a TF is still missing.

                # ICP maps cloud -> LiDAR. URDF needs body -> base, including
                # the driver's internal optical/body and LiDAR frame transforms.
                mount = camera_mount_matrix(self.tf_buffer, *frames, f"zed_{name}_camera_link", pose)
                updated[name] = (*frames, pose)
                mounts[name] = mount
                if global_search:
                    confirmed_names.add(name)
            except (TransformException, ValueError) as exc:
                skipped.append(f"{name}: {exc}")
            except Exception as exc:
                skipped.append(f"{name}: registration error: {exc}")

        if skipped:
            self.get_logger().warning("Skipped ZED clouds: " + "; ".join(skipped),
                                      throttle_duration_sec=30.0)
        if not updated:
            return
        try:
            # Include earlier accepted mounts even if the description topic was
            # delayed or the vehicle publisher restarted with its base URDF.
            new_description = update_camera_joints(description, {**previous_mounts, **mounts})
        except (ET.ParseError, ValueError) as exc:
            self.get_logger().error(f"Could not update camera joints in robot_description: {exc}")
            return
        request = SetParameters.Request()
        request.parameters = [Parameter("robot_description", Parameter.Type.STRING,
                                        new_description).to_parameter_msg()]
        with self.state_lock:
            self.update_in_flight = True
            self.update_started = time.monotonic()
        try:
            future = self.description_client.call_async(request)
            with self.state_lock:
                self.pending_update = future
            future.add_done_callback(
                lambda result: self.description_update_done(result, new_description, updated, mounts, confirmed_names))
        except Exception as exc:
            with self.state_lock:
                self.update_in_flight = False
            self.get_logger().error(f"Could not request URDF update: {exc}")

    def description_update_done(self, future, description, updated, mounts, confirmed_names):
        # future: the pending robot_state_publisher parameter-update response.
        # description: the proposed updated URDF as an XML string.
        # updated: names mapped to (LiDAR frame, camera frame, camera-to-LiDAR matrix).
        # mounts: camera names mapped to the proposed camera-body-to-base_link matrices.
        # confirmed_names: cameras whose new global pose becomes the drift anchor.
        with self.state_lock:
            if future is not self.pending_update:
                return  # A timed-out request must not replace a newer estimate.
        try:
            result = future.result()
            if len(result.results) != 1 or not result.results[0].successful:
                reason = result.results[0].reason if result.results else "no parameter result"
                raise RuntimeError(reason)
            # Use a pose for visualization and the next ICP only after the
            # vehicle publisher accepts that same pose in its URDF.
            calibrated = []
            with self.state_lock:
                if future is not self.pending_update:
                    return
                self.robot_description = description
                self.camera_poses.update(updated)
                self.camera_mounts.update(mounts)
                for name in updated:
                    if name in confirmed_names or name not in self.camera_anchors:
                        lidar_frame, camera_frame, pose = updated[name]
                        self.camera_anchors[name] = (lidar_frame, camera_frame, pose.copy())
                        calibrated.append(name)
                    self.camera_failures.pop(name, None)
                    self.retry_global.discard(name)
                    self.pending_candidates.pop(name, None)
            try:
                save_calibrated_urdf(description, self.output_urdf)
            except OSError as exc:
                self.get_logger().error(f"Live URDF updated but could not save {self.output_urdf}: {exc}")
            if calibrated:
                self.get_logger().info("Calibrated ZED cameras: " + ", ".join(calibrated))
            for name, (lidar_frame, camera_frame, pose) in updated.items():
                self.get_logger().debug(f"{name}: {camera_frame} -> {lidar_frame} camera-to-LiDAR pose:\n"
                                        f"{np.array2string(pose, precision=4)}")
        except Exception as exc:
            self.get_logger().error(f"Vehicle robot_state_publisher rejected calibration: {exc}")
        finally:
            with self.state_lock:
                if self.pending_update is future:
                    self.pending_update = None
                    self.update_in_flight = False


def main(args=None):
    # args: optional ROS command-line arguments.
    rclpy.init(args=args)  # Initialize ROS before creating the node.
    node = None
    executor = None
    try:
        node = LidarCameraRegistration()
        # Separate threads let camera callbacks run during LiDAR processing.
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()  # Process sensor messages until shutdown.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C or ROS shutdown should exit without an error traceback.
    finally:
        # Stop callback threads before destroying their node and ROS context.
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
