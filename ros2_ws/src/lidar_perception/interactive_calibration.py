"""Operator review and full-cloud previews for LiDAR-camera calibration.

This is the calibration backend for the independent vehicle UI Hub. The hub
uses preview-only frames; only an explicitly accepted pose updates the URDF.
"""
import copy
import json
import math
import time
from typing import Callable, NamedTuple

import numpy as np
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

HUB_PREFIX = "/lidar_perception/hub"
# Full camera clouds can be large; keep review previews responsive without
# sending every camera cloud across the Jetson-to-Karbon link.
PREVIEW_INTERVAL_S = 0.5


class CalibrationAlgorithms(NamedTuple):
    prepare_fpfh: Callable
    log_fpfh_diagnostics: Callable
    estimate_initial_pose: Callable
    register_camera: Callable
    camera_mount_matrix: Callable
    confirm_initial_pose: Callable
    update_camera_joints: Callable
    save_calibrated_urdf: Callable


def quaternion_from_matrix(matrix):
    # Convert a rigid rotation to ROS quaternion order (x, y, z, w).
    rotation = np.asarray(matrix, dtype=float)[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale, 0.25 * scale)
    axis = int(np.argmax(np.diag(rotation)))
    if axis == 0:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        return (0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[2, 1] - rotation[1, 2]) / scale)
    if axis == 1:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        return ((rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale)
    scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
    return ((rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale,
            0.25 * scale, (rotation[1, 0] - rotation[0, 1]) / scale)


def finite_matrix(matrix):
    """Return a JSON-safe homogeneous matrix, or None if it is invalid."""
    if matrix is None:
        return None
    try:
        values = np.asarray(matrix, dtype=float)
        return values.tolist() if values.shape == (4, 4) and np.isfinite(values).all() else None
    except (TypeError, ValueError):
        return None


class InteractiveCalibration:
    def __init__(self, node, cameras, algorithms):
        self.node, self.cameras, self.algorithms = node, tuple(cameras), algorithms
        self.active = False
        self.session = 0
        self.accepted = set()
        self.accepted_quality = {}
        self.candidates = {}
        self.reviews = {}
        self.review_published = set()
        self.review_published_at = {}
        self._last_preview = {}
        self.accept_queue = {}
        self.applying = None
        self.generation = {name: 0 for name in self.cameras}
        self.reasons = {name: "Waiting to start" for name in self.cameras}
        self.revision = 0
        self.group = MutuallyExclusiveCallbackGroup()
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = node.create_publisher(String, f"{HUB_PREFIX}/status", latched)
        self.tf = TransformBroadcaster(node)
        self.preview_pubs = {name: node.create_publisher(PointCloud2, f"{HUB_PREFIX}/clouds/{name}", latest)
                             for name in self.cameras}
        self.services = [node.create_service(
            Trigger, f"{HUB_PREFIX}/{action}",
            lambda request, response, a=action: self.control(a, None, response),
            callback_group=self.group) for action in ("start", "stop")]
        for name in self.cameras:
            for action in ("review", "accept", "recompute", "release"):
                self.services.append(node.create_service(
                    Trigger, f"{HUB_PREFIX}/{name}/{action}",
                    lambda request, response, a=action, n=name: self.control(a, n, response),
                    callback_group=self.group))
        self.timer = node.create_timer(0.5, self.tick, callback_group=self.group)

    def stopped_cameras(self):
        return self.calibration_snapshot()[0]

    def calibration_snapshot(self):
        """Snapshot cameras to skip (None when inactive) and the current session."""
        with self.node.state_lock:
            stopped = self.accepted | set(self.accept_queue) if self.active else None
            return stopped, self.session

    def session_active(self, session):
        """Check whether work still belongs to the active session."""
        with self.node.state_lock:
            return self.active and session == self.session

    def note_reasons(self, reasons, stopped, session=None):
        with self.node.state_lock:
            if not self.active or (session is not None and session != self.session):
                return
            for name, reason in reasons.items():
                if reason and name not in stopped and name not in self.accepted and name not in self.accept_queue:
                    self.reasons[name] = reason

    def control(self, action, name, response):
        # Review freezes the displayed proposal, while new estimates continue in
        # the background. Accept always commits that exact reviewed proposal.
        with self.node.state_lock:
            if action == "start":
                if not self.active:
                    self.session += 1
                    self.active = True
                    self.node.next_calibration = 0.0
                    for camera in self.cameras:
                        if camera not in self.accepted and camera not in self.accept_queue:
                            self.reasons[camera] = "Waiting for sensor data"
                response.success, response.message = True, "Calibration started"
            elif action == "stop":
                if self.active:
                    self.active = False
                    self.session += 1  # Discard results from the stopped session.
                    for camera in self.cameras:
                        self.candidates.pop(camera, None)
                        self.reviews.pop(camera, None)
                        self.review_published.discard(camera)
                        self.review_published_at.pop(camera, None)
                        self.node.pending_candidates.pop(camera, None)
                        self.node.camera_failures.pop(camera, None)
                        if camera not in self.accepted and camera in self.node.camera_poses:
                            # Keep the last accepted pose even if a replacement is applying.
                            self.accepted.add(camera)
                            if camera not in self.accept_queue:
                                self.reasons[camera] = "Previous calibration retained"
                        if camera in self.accepted or camera in self.accept_queue:
                            continue
                        self.generation[camera] += 1
                        self.node.retry_global.add(camera)
                        self.reasons[camera] = "Calibration stopped"
                response.success, response.message = True, "Calibration stopped; accepted poses retained"
            elif name not in self.cameras or not self.active:
                response.success, response.message = False, "Start calibration first"
            elif getattr(self.node, "saved_camera_mounts", None):
                response.success, response.message = False, "Wait for saved calibration to finish restoring"
            elif action == "review":
                candidate = self.candidates.get(name)
                if name in self.accepted or name in self.accept_queue:
                    response.success, response.message = False, "This camera is already accepted or applying"
                elif candidate is None or not candidate["ready"]:
                    response.success, response.message = False, "No valid proposal is ready"
                else:
                    self.reviews[name] = candidate
                    self.review_published.discard(name)
                    self.review_published_at.pop(name, None)
                    response.success, response.message = True, "Review preview selected"
            elif action == "accept":
                review = self.reviews.get(name)
                preview_time = self.review_published_at.get(name)
                if (review is None or name not in self.review_published or preview_time is None
                        or time.monotonic() - preview_time > self.node.config.max_cloud_age_s):
                    response.success, response.message = False, "Wait for a fresh review preview"
                elif name in self.accept_queue:
                    response.success, response.message = False, "Camera update is already being applied"
                else:
                    self.accept_queue[name] = review
                    self.reviews.pop(name, None)
                    self.review_published.discard(name)
                    self.review_published_at.pop(name, None)
                    self.generation[name] += 1  # Discard any calculation already in flight.
                    self.reasons[name] = "Applying accepted calibration"
                    response.success, response.message = True, "Accepted; applying vehicle transform"
            elif action == "recompute":
                if name in self.accept_queue:
                    response.success, response.message = False, "Wait for the current update to finish"
                else:
                    self.generation[name] += 1
                    self.accepted.discard(name)
                    self.reviews.pop(name, None)
                    self.review_published.discard(name)
                    self.review_published_at.pop(name, None)
                    self.candidates.pop(name, None)
                    self.node.pending_candidates.pop(name, None)
                    self.node.camera_failures.pop(name, None)
                    self.node.retry_global.add(name)
                    self.reasons[name] = "Recomputing from fresh scans"
                    self.node.next_calibration = 0.0
                    response.success, response.message = True, "Recomputing this camera"
            elif action == "release":
                self.reviews.pop(name, None)
                self.review_published.discard(name)
                self.review_published_at.pop(name, None)
                response.success, response.message = True, "Review closed"
            else:
                response.success, response.message = False, "Unknown calibration action"
        return response

    def status(self):
        now = time.monotonic()
        with self.node.data_lock:
            lidar = self.node.latest_lidar
            samples = self.node.camera_clouds.copy()
            receipts = {name: tuple(times) for name, times in self.node.cloud_receipts.items()}

        def rate(name):
            # Show a recent receive rate, never a stale or single-sample estimate.
            times = [stamp for stamp in receipts.get(name, ()) if 0 <= now - stamp <= 5.0]
            if len(times) < 2 or now - times[-1] > 3.0 or times[-1] <= times[0]:
                return None
            return round((len(times) - 1) / (times[-1] - times[0]), 1)
        with self.node.state_lock:
            cameras = {}
            for name in self.cameras:
                candidate = self.candidates.get(name)
                if name in self.accept_queue:
                    state = "applying"
                elif name in self.accepted:
                    state = "accepted"
                elif name in self.reviews:
                    state = "reviewing"
                elif not self.active:
                    state = "idle"
                elif candidate and candidate["ready"]:
                    state = "ready"
                else:
                    state = "searching"
                previous = self.node.pending_candidates.get(name)
                # Report the held review first, then applying, proposed, or accepted poses.
                selected = self.reviews.get(name)
                source = "review"
                if selected is None:
                    selected, source = self.accept_queue.get(name), "applying"
                if selected is None:
                    selected, source = candidate, "proposal"
                if selected is None:
                    pose = self.node.camera_poses.get(name)
                    if pose is not None:
                        selected = {"frames": pose[:2], "pose": pose[2],
                                    "mount": self.node.camera_mounts.get(name),
                                    "quality": self.accepted_quality.get(name)}
                        source = "accepted" if name in self.accepted else "previous"
                transform = None
                if selected is not None:
                    transform = {
                        "source": source,
                        "lidar_frame": selected["frames"][0],
                        "cloud_frame": selected["frames"][1],
                        "body_frame": f"zed_{name}_camera_link",
                        "preview_frame": f"hub_zed_{name}_cloud",
                        "cloud_to_lidar": finite_matrix(selected.get("pose")),
                        "mount_to_base": finite_matrix(selected.get("mount")),
                        "quality": selected.get("quality"),
                    }
                cameras[name] = {
                    "state": state,
                    "detail": self.reasons[name],
                    "confirmations": len(previous["poses"]) if previous else 0,
                    "required": self.node.config.initial_pose_confirmations if self.node.config.initial_pose_confirmation_enabled else 1,
                    "review_visible": (name in self.review_published
                                       and now - self.review_published_at.get(name, float("-inf"))
                                       <= self.node.config.max_cloud_age_s),
                    "sensor_online": name in samples and now - samples[name][1] <= self.node.config.max_cloud_age_s,
                    "cloud_hz": rate(name),
                    "transform": transform,
                }
            payload = {"active": self.active, "cameras": cameras,
                       "lidar_online": lidar is not None and now - lidar[1] <= self.node.config.max_cloud_age_s,
                       "lidar_hz": rate("lidar")}
        self.status_pub.publish(String(data=json.dumps(payload)))

    def tick(self):
        self.apply_queued()
        self.status()

    def apply_queued(self):
        # Serialize URDF updates so accepting multiple cameras never loses one.
        with self.node.state_lock:
            if getattr(self.node, "saved_camera_mounts", None):
                return
            if self.node.update_in_flight:
                if (self.applying is not None
                        and time.monotonic() - self.node.update_started > max(30.0, 3 * self.node.config.calibration_interval_s)):
                    self.reasons[self.applying] = "Still waiting for vehicle transform update"
                return  # A timed-out ROS request may still run; never overlap updates.
            if not self.accept_queue:
                candidate = None
            else:
                name = next(iter(self.accept_queue))
                candidate = self.accept_queue[name]
                description = self.node.robot_description
                mounts = self.node.camera_mounts.copy()
                if not description or not self.node.description_client.service_is_ready():
                    self.reasons[name] = "Waiting for vehicle robot_description service"
                    candidate = None
                else:
                    self.node.update_in_flight = True
                    self.node.update_started = time.monotonic()
                    self.applying = name
        if candidate is None:
            return
        try:
            new_description = self.algorithms.update_camera_joints(description, {**mounts, name: candidate["mount"]})
            request = SetParameters.Request()
            request.parameters = [Parameter("robot_description", Parameter.Type.STRING,
                                            new_description).to_parameter_msg()]
            future = self.node.description_client.call_async(request)
            with self.node.state_lock:
                self.node.pending_update = future
            future.add_done_callback(lambda result: self.update_done(result, name, candidate, new_description))
        except Exception as exc:
            with self.node.state_lock:
                self.accept_queue.pop(name, None)
                self.applying = None
                self.node.update_in_flight = False
                self.reasons[name] = f"Could not apply calibration: {exc}"
            self.node.get_logger().error(f"Could not apply {name} calibration: {exc}")

    def update_done(self, future, name, candidate, description):
        try:
            result = future.result()
            if len(result.results) != 1 or not result.results[0].successful:
                reason = result.results[0].reason if result.results else "no parameter result"
                raise RuntimeError(reason)
            with self.node.state_lock:
                if self.node.pending_update is not future:
                    return
                frames = candidate["frames"]
                pose = candidate["pose"].copy()
                self.node.robot_description = description
                self.node.camera_poses[name] = (*frames, pose)
                self.node.camera_mounts[name] = candidate["mount"].copy()
                self.accepted_quality[name] = candidate.get("quality")
                self.node.camera_anchors[name] = (*frames, pose.copy())
                self.accepted.add(name)
                self.candidates.pop(name, None)
                self.node.pending_candidates.pop(name, None)
                self.node.retry_global.discard(name)
                self.node.camera_failures.pop(name, None)
                self.reasons[name] = "Accepted; calibration stopped"
            try:
                self.algorithms.save_calibrated_urdf(description, self.node.output_urdf)
            except OSError as exc:
                with self.node.state_lock:
                    self.reasons[name] = f"Live calibration active; could not save: {exc}"
                self.node.get_logger().error(f"Live URDF updated but could not save {self.node.output_urdf}: {exc}")
            else:
                self.node.get_logger().info(f"Saved camera calibration to {self.node.output_urdf}")
            self.node.get_logger().info(f"Accepted and applied {name} camera calibration.")
        except Exception as exc:
            with self.node.state_lock:
                if self.node.pending_update is future:
                    self.reasons[name] = f"Vehicle transform update failed: {exc}"
            self.node.get_logger().error(f"Could not apply {name} calibration: {exc}")
        finally:
            with self.node.state_lock:
                if self.node.pending_update is future:
                    self.accept_queue.pop(name, None)
                    self.applying = None
                    self.node.pending_update = None
                    self.node.update_in_flight = False

    def publish_cloud(self, name, msg):
        publisher = self.preview_pubs[name]
        if not publisher.get_subscription_count():
            return
        now = time.monotonic()
        with self.node.state_lock:
            selected = (self.reviews.get(name) or self.accept_queue.get(name)
                        or self.candidates.get(name))
            if selected is None:
                accepted = self.node.camera_poses.get(name)
                selected = {"frames": accepted[:2], "pose": accepted[2]} if accepted else None
            valid = selected is not None and selected["frames"][1] == msg.header.frame_id
            signature = ((msg.header.frame_id, selected["frames"],
                          np.asarray(selected["pose"]).tobytes(), self.reviews.get(name) is selected)
                         if valid else (msg.header.frame_id, None))
            previous = self._last_preview.get(name)
            if previous and previous[1] == signature and now - previous[0] < PREVIEW_INTERVAL_S:
                return
        if not valid:
            # Clear the old preview until a pose for this cloud frame is available.
            publisher.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=msg.header.stamp, frame_id="base_link"), []))
            with self.node.state_lock:
                self._last_preview[name] = (time.monotonic(), signature)
            return
        transform = TransformStamped()
        transform.header = Header(stamp=msg.header.stamp, frame_id=selected["frames"][0])
        transform.child_frame_id = f"hub_zed_{name}_cloud"
        pose = selected["pose"]
        translation = transform.transform.translation
        translation.x, translation.y, translation.z = (float(value) for value in pose[:3, 3])
        quaternion = transform.transform.rotation
        quaternion.x, quaternion.y, quaternion.z, quaternion.w = (float(value) for value in quaternion_from_matrix(pose))
        self.tf.sendTransform(transform)
        preview = copy.copy(msg)
        preview.header = Header(stamp=msg.header.stamp, frame_id=transform.child_frame_id)
        publisher.publish(preview)
        with self.node.state_lock:
            sent_at = time.monotonic()
            self._last_preview[name] = (sent_at, signature)
            if self.reviews.get(name) is selected and msg.width * msg.height:
                self.review_published.add(name)
                self.review_published_at[name] = sent_at

    def calibrate(self, lidar, header, cameras, reasons, session=None):
        # Produce proposals only. No vehicle TF is changed until the operator
        # reviews and accepts one of these exact poses.
        if session is None:
            with self.node.state_lock:
                session = self.session
        lidar_features = None
        for name in self.cameras:
            with self.node.state_lock:
                if not self.active or session != self.session:
                    return
                if name in self.accepted or name in self.accept_queue:
                    continue
                generation = self.generation[name]
            if name not in cameras:
                with self.node.state_lock:
                    if self.active and session == self.session and self.generation[name] == generation:
                        self.reasons[name] = reasons[name]
                continue
            msg, camera = cameras[name]
            frames = (header.frame_id, msg.header.frame_id)
            try:
                if lidar_features is None:
                    lidar_features = self.algorithms.prepare_fpfh(lidar, self.node.config)
                    if self.node.config.cloud_diagnostics_enabled:
                        why = "not computed: too few points after FPFH downsampling" if lidar_features[0] is None else None
                        self.algorithms.log_fpfh_diagnostics(self.node.get_logger(), "lidar", lidar_features[1], why)
                if not self.session_active(session):
                    return
                metrics = {}
                report = None
                if self.node.config.cloud_diagnostics_enabled:
                    report = lambda features, why: self.algorithms.log_fpfh_diagnostics(self.node.get_logger(), name, features, why)
                initial_pose, reason = self.algorithms.estimate_initial_pose(camera, lidar_features, self.node.config, metrics, report)
                ransac_metrics = metrics
                if not self.session_active(session):
                    return
                if initial_pose is None:
                    raise ValueError(reason)
                metrics = {}
                registered, pose, reason = self.algorithms.register_camera(camera, lidar, initial_pose, self.node.config, metrics)
                if not self.session_active(session):
                    return
                if registered is None:
                    raise ValueError(reason)
                if self.node.config.cloud_diagnostics_enabled and metrics:
                    self.node.get_logger().info(
                        f"Calibration quality {name}: ICP fitness {metrics['fitness']:.3f}, "
                        f"RMSE {metrics['rmse']:.3f} m, {metrics['matches']} matches")
                mount = self.algorithms.camera_mount_matrix(self.node.tf_buffer, *frames, f"zed_{name}_camera_link", pose)
                with self.node.state_lock:
                    if (session != self.session or self.generation[name] != generation or name in self.accepted
                            or name in self.accept_queue or not self.active):
                        continue
                    if self.node.config.initial_pose_confirmation_enabled:
                        _, reason = self.algorithms.confirm_initial_pose(
                            self.node.pending_candidates, name, frames, pose, header.stamp, msg.header.stamp, self.node.config)
                    else:
                        reason = "Proposal ready for review"
                    self.revision += 1
                    self.candidates[name] = {
                        "frames": frames, "pose": pose.copy(), "mount": mount.copy(),
                        "ready": True, "revision": self.revision,
                        "quality": {
                            key: float(value) for key, value in {
                                "ransac_fitness": ransac_metrics.get("fitness"),
                                "ransac_matches": ransac_metrics.get("matches"),
                                "icp_fitness": metrics.get("fitness"),
                                "icp_rmse_m": metrics.get("rmse"),
                                "icp_matches": metrics.get("matches"),
                            }.items() if value is not None and math.isfinite(float(value))
                        },
                    }
                    self.reasons[name] = reason
            except Exception as exc:
                report_failure = False
                with self.node.state_lock:
                    if (self.active and session == self.session and self.generation[name] == generation and name not in self.accepted
                            and name not in self.accept_queue):
                        previous = self.node.pending_candidates.get(name)
                        count = len(previous["poses"]) if previous and previous["frames"] == frames else 0
                        suffix = f"; keeping {count}/{self.node.config.initial_pose_confirmations}" if count else ""
                        self.reasons[name] = f"{exc}{suffix}"
                        report_failure = True
                if report_failure:
                    self.node.get_logger().warning(f"Hub calibration {name}: {exc}", throttle_duration_sec=30.0)

