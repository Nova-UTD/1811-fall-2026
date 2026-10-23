"""Route recorder -- Stage A1 of teach-and-repeat.

Subscribes /odometry and, while recording is enabled, appends downsampled
(x, y, yaw) waypoints to an in-memory buffer. Call the ~save service (or let
it save on shutdown) to write that buffer to a CSV file in exactly the
format control.pure_pursuit_core.load_csv_path expects -- so the recorded
file can be handed straight to pure_pursuit_node's path_file param to close
a teach-and-repeat cycle, no route_publisher required yet.

>>> Must run in the same continuous odometry session you intend to repeat
in <<< -- restarting KISS-ICP between TEACH and REPEAT moves the `odom`
frame's origin, and the recorded route silently stops matching where the
vehicle actually thinks it is. See docs/teach_and_repeat_guide.md and this
package's README.

Standalone, "TEACH mode" here is just: this node is running and `enabled`
is true. With mode_manager, the gamepad's record button drives the services.
record_on_start (default true) means the simplest flow is just "launch it,
drive, ctrl-c" -- for multiple takes in
one process, use the ~start_recording / ~stop_recording / ~save services
instead of restarting the node.
"""
import os
from datetime import datetime

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_srvs.srv import Trigger

from routing import route_recorder_core as rrc


class RouteRecorderNode(Node):

    def __init__(self):
        super().__init__('route_recorder_node')

        self.declare_parameter('odom_topic', '/odometry')
        # m -- guide's recommendation is "~every 0.1-0.2 m of travel"
        self.declare_parameter('min_spacing_m', 0.15)
        # Directory routes are saved into when output_file isn't set
        # (created if missing). Inside the repo (bind-mounted into the
        # container as /vehicle_1811) so it's visible on the host too;
        # gitignored -- see .gitignore. Empty string -> <cwd>/routes instead.
        self.declare_parameter('output_dir', '/vehicle_1811/routes')
        # Exact output path. Empty string -> auto-generate a timestamped
        # name inside output_dir when a take starts.
        self.declare_parameter('output_file', '')
        self.declare_parameter('record_on_start', True)
        # s -- how long without odometry before warning while actively
        # recording. Purely informational; doesn't stop anything.
        self.declare_parameter('odom_timeout', 1.0)
        # Every successful save is also published here, latched, so a running
        # pure_pursuit_node (path_file:='') picks up the new route without a
        # restart. Empty string -> don't publish.
        self.declare_parameter('path_topic', '/planning/path')

        p = self.get_parameter
        self._output_dir_param = p('output_dir').value
        self._output_file_param = p('output_file').value
        self._odom_timeout = p('odom_timeout').value

        self._buffer = rrc.RouteBuffer(min_spacing_m=p('min_spacing_m').value)
        self._enabled = p('record_on_start').value
        self._current_output_path = self._resolve_output_path()
        self._last_odom_stamp = None
        self._warned_stale = False
        self._frame_id = 'odom'

        self._path_pub = None
        if p('path_topic').value:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self._path_pub = self.create_publisher(Path, p('path_topic').value, latched)

        self.create_subscription(Odometry, p('odom_topic').value, self._on_odom, 10)

        self.create_service(Trigger, '~/start_recording', self._on_start_recording)
        self.create_service(Trigger, '~/stop_recording', self._on_stop_recording)
        self.create_service(Trigger, '~/save', self._on_save)

        self._watchdog_timer = self.create_timer(1.0, self._check_odom_stale)

        if self._enabled:
            self.get_logger().info(
                f'Recording from startup -- output will be saved to {self._current_output_path}')
        else:
            self.get_logger().info(
                'record_on_start is false -- call ~start_recording to begin')

    def _resolve_output_path(self) -> str:
        if self._output_file_param:
            return self._output_file_param
        output_dir = self._output_dir_param or os.path.join(os.getcwd(), 'routes')
        os.makedirs(output_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        return os.path.join(output_dir, f'route_{ts}.csv')

    def _on_odom(self, msg: Odometry):
        self._last_odom_stamp = self.get_clock().now()
        self._warned_stale = False
        self._frame_id = msg.header.frame_id or self._frame_id
        if not self._enabled:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = rrc.yaw_from_quaternion(msg.pose.pose.orientation)
        added = self._buffer.maybe_add(x, y, yaw)
        if added:
            self.get_logger().info(
                f'recording... {len(self._buffer)} points, {self._buffer.length_m:.1f} m so far',
                throttle_duration_sec=2.0)

    def _check_odom_stale(self):
        if not self._enabled or self._last_odom_stamp is None or self._warned_stale:
            return
        age_s = (self.get_clock().now() - self._last_odom_stamp).nanoseconds * 1e-9
        if age_s > self._odom_timeout:
            self.get_logger().warning(
                f'No odometry for {age_s:.1f}s while recording -- lidar dropout? '
                'the gap will show up as a straight-line skip in the route.')
            self._warned_stale = True

    def _on_start_recording(self, request, response):
        self._buffer.clear()
        self._current_output_path = self._resolve_output_path()
        self._enabled = True
        response.success = True
        response.message = (
            f'recording started, buffer cleared, will save to {self._current_output_path}')
        self.get_logger().info(response.message)
        return response

    def _on_stop_recording(self, request, response):
        self._enabled = False
        response.success = True
        response.message = (
            f'recording stopped, {len(self._buffer)} points buffered (not yet saved)')
        self.get_logger().info(response.message)
        return response

    def _on_save(self, request, response):
        response.success, response.message = self._save_to_disk()
        return response

    def _save_to_disk(self):
        if not self._buffer.points:
            msg = 'nothing to save -- buffer is empty'
            self.get_logger().warning(msg)
            return False, msg
        with open(self._current_output_path, 'w') as f:
            f.write(rrc.format_csv(self._buffer.points))
        msg = (f'saved {len(self._buffer)} points ({self._buffer.length_m:.1f} m) '
               f'to {self._current_output_path}')
        self.get_logger().info(msg)
        self._publish_path()
        return True, msg

    def _publish_path(self):
        if self._path_pub is None:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._frame_id
        for pt in self._buffer.points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = pt.x
            pose.pose.position.y = pt.y
            pose.pose.orientation.z = math.sin(pt.yaw / 2.0)
            pose.pose.orientation.w = math.cos(pt.yaw / 2.0)
            path.poses.append(pose)
        self._path_pub.publish(path)


def main():
    rclpy.init()
    node = RouteRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        # Best-effort save so ctrl-c doesn't silently lose an unsaved take.
        if node._buffer.points:
            node.get_logger().info('shutting down -- saving buffered route')
            node._save_to_disk()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
