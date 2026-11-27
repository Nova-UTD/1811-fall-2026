# Connect the shared registration node to 1811's operator-reviewed calibration.

# Author: Khoa Nguyen
# Email: minhkhoa.ee@outlook.com
# Copyright: 2021, Nova UTD
# License: MIT License

from collections import deque
from threading import RLock
import time

from interactive_calibration import InteractiveCalibration
from lidar_camera_calibration.clouds import pack_registered_cloud
from lidar_camera_calibration.node import LidarCameraRegistration
from lidar_camera_calibration.timing import matching_floor_plane, pair_error
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor


class VehicleLidarCameraRegistration(LidarCameraRegistration):

    def __init__(self):
        super().__init__()
        interactive = self.declare_parameter(
            'interactive_calibration', False, ParameterDescriptor(read_only=True),
        ).value
        self.hub = None
        if interactive:
            if not self.config.apply_urdf:
                raise ValueError('interactive_calibration requires apply_urdf')
            # The shared restore callback locks the same state as Hub services.
            # Hold this lock across both updates so the Hub sees them together.
            self.state_lock = RLock()
            self.cloud_receipts = {
                name: deque(maxlen=120) for name in ('lidar', *self.config.cameras)
            }
            self.hub = InteractiveCalibration(self)
            for name in self.saved_camera_mounts:
                self.hub.reasons[name] = 'Restoring saved calibration'

    def lidar_callback(self, lidar_message):
        super().lidar_callback(lidar_message)
        if self.hub is not None:
            with self.data_lock:
                self.cloud_receipts['lidar'].append(self.latest_lidar[1])

    def camera_callback(self, camera_name, cloud_message):
        super().camera_callback(camera_name, cloud_message)
        if self.hub is None:
            return
        with self.data_lock:
            self.cloud_receipts[camera_name].append(self.camera_clouds[camera_name][1])
        try:
            self.hub.publish_cloud(camera_name, cloud_message)
        except Exception as exc:
            self.get_logger().warning(
                f'Could not publish hub preview cloud for {camera_name}: {exc}',
                throttle_duration_sec=30.0,
            )

    def restore_update_done(self, future, description, mounts):
        if self.hub is None:
            super().restore_update_done(future, description, mounts)
            return
        with self.state_lock:
            if self.pending_update is not future:
                return
            newly_restored = mounts.keys() - self.camera_mounts.keys()
            super().restore_update_done(future, description, mounts)
            if self.saved_camera_mounts:
                return  # The shared node will retry a failed restore.
            # A Karbon restart reapplies live mounts without cancelling Recompute.
            self.hub.accepted.update(newly_restored)
            for name in mounts.keys() & self.hub.accepted:
                self.hub.reasons[name] = 'Restored saved calibration'

    def calibration_tick(self):
        if self.hub is None:
            super().calibration_tick()
            return
        if self.saved_camera_mounts:
            self.restore_saved_calibration()
            return

        now_monotonic_s = time.monotonic()
        with self.state_lock:
            stopped_camera_names, session = self.hub.calibration_snapshot()
            if stopped_camera_names is None or self.config.cameras.keys() <= stopped_camera_names:
                return
            if now_monotonic_s < self.next_calibration:
                return
            self.next_calibration = now_monotonic_s + self.config.calibration_interval_s
            accepted_poses = self.camera_poses.copy()

        with self.data_lock:
            lidar_sample = self.latest_lidar
            camera_samples = {}
            floor_planes = {}
            for name, sample in self.camera_clouds.items():
                if name in stopped_camera_names:
                    continue
                camera_samples[name] = sample
                # Keep the exact timestamp-matched floor plane with the cloud.
                floor_topic = self.config.cameras[name].floor_plane_topic
                if self.config.ground_filter_enabled and floor_topic:
                    floor_planes[name] = matching_floor_plane(
                        self.floor_planes[name], sample[0].header.stamp,
                    )

        prepared_clouds = {}
        input_errors = {}
        samples = {'lidar': lidar_sample, **camera_samples}
        for sensor_name, sample in samples.items():
            if not self.hub.session_active(session):
                return
            if sample is None:
                continue
            message, received_monotonic_s = sample
            if now_monotonic_s - received_monotonic_s <= self.config.max_cloud_age_s:
                try:
                    prepared_clouds[sensor_name] = self.prepare_calibration_input(
                        sensor_name, message, floor_planes.get(sensor_name),
                    )
                    continue
                except Exception as exc:
                    input_errors[sensor_name] = f'calibration input failed: {exc}'
                    if sensor_name == 'lidar':
                        self.get_logger().warning(
                            f'LiDAR {input_errors[sensor_name]}', throttle_duration_sec=30.0,
                        )
            # Clear the previous RViz input when a sensor is stale or its filtering failed.
            empty_cloud = pack_registered_cloud([], message.header, self.config)
            self.input_pubs[sensor_name].publish(empty_cloud)

        lidar = lidar_sample[0] if lidar_sample else None
        lidar_matching_cloud, lidar_display_cloud = prepared_clouds.pop('lidar', (None, None))
        lidar_rejection = None
        if lidar_matching_cloud is None:
            missing_reason = 'LiDAR cloud is stale' if lidar else 'no LiDAR cloud'
            lidar_rejection = input_errors.get('lidar', missing_reason)
            if 'lidar' not in input_errors:
                self.get_logger().warning(
                    f'Waiting for a fresh LiDAR point cloud on {self.config.lidar_topic}.',
                    throttle_duration_sec=30.0,
                )
        elif len(lidar_matching_cloud.points) < self.config.min_points:
            remaining = len(lidar_matching_cloud.points)
            lidar_rejection = f'Too few LiDAR points after filtering ({remaining} remaining)'
            self.get_logger().warning(lidar_rejection, throttle_duration_sec=30.0)
        rejection_reasons = {}
        matching_cameras = {}
        display_cameras = {}
        for name in self.config.cameras:
            if name in stopped_camera_names:
                continue
            reason = lidar_rejection
            if reason is None:
                reason = pair_error(lidar, camera_samples.get(name), now_monotonic_s, self.config)
            reason = input_errors.get(name, reason)
            if name in prepared_clouds:
                matching_cloud, display_cloud = prepared_clouds[name]
                message = camera_samples[name][0]
                display_cameras[name] = (message, display_cloud)
                if reason is None and len(matching_cloud.points) < self.config.min_points:
                    remaining = len(matching_cloud.points)
                    reason = f'too few camera points after filtering ({remaining})'
                if reason is None:
                    matching_cameras[name] = (message, matching_cloud)
            rejection_reasons[name] = reason

        self.hub.note_reasons(rejection_reasons, stopped_camera_names, session)
        if lidar is not None:
            try:
                self.publish_registered_inputs(
                    lidar.header, lidar_display_cloud, display_cameras, rejection_reasons,
                    accepted_poses,
                )
            except Exception as exc:
                self.get_logger().error(f'Could not publish registration overlay: {exc}')

        if matching_cameras:
            # The Hub proposes poses even before the Karbon is available. Only
            # its Accept service can queue a request to update the vehicle URDF.
            try:
                self.hub.calibrate(
                    lidar_matching_cloud, lidar.header, matching_cameras, rejection_reasons,
                    session,
                )
            except Exception as exc:
                self.get_logger().error(f'Hub calibration failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = executor = None
    try:
        node = VehicleLidarCameraRegistration()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
