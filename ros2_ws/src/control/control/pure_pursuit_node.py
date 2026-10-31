"""Pure pursuit path-tracking controller.

Subscribes to the vehicle's live pose (nav_msgs/Odometry, default topic
/odometry -- KISS-ICP's output per docs/teach_and_repeat_guide.md) and a
path -- either a CSV file loaded once at startup (for bench-testing before
route_publisher exists) or a latched nav_msgs/Path on /planning/path.
Publishes vehicle_msgs/VehicleCommand on /cmd/auto at a fixed control rate.

Per the guide's interface contract, this node never talks to
/vehicle_command or the Arduino directly -- mode_manager gates /cmd/auto
with a deadman switch before it reaches serial_bridge. For a bench test
without mode_manager, remap the output:
    ros2 run control pure_pursuit_node --ros-args -r /cmd/auto:=/vehicle_command

Safety: if the path hasn't loaded, or odometry hasn't been heard from
recently (odom_timeout), this publishes an all-zero VehicleCommand
(throttle=0, steer=0, brake=0) every tick rather than repeating the last
command -- it never drives on stale data.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from vehicle_msgs.msg import VehicleCommand

from control import pure_pursuit_core as ppc


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # m, base Ld
        self.declare_parameter('lookahead_distance', 0.25)
        # m per m/s of speed; 0 = fixed Ld (fixed oscillates low-speed, cuts corners high-speed)
        self.declare_parameter('lookahead_speed_gain', 0.0)
        # m -- must stay <= lookahead_distance or it silently overrides it via the clamp
        self.declare_parameter('min_lookahead', 0.2)
        self.declare_parameter('max_lookahead', 3.0)             # m
        # points; how far ahead of last_idx closest_index searches each tick
        self.declare_parameter('closest_index_window', 5)
        # m -- measured, vehicle_1811_description/urdf/vehicle_1811.urdf.xacro
        self.declare_parameter('wheelbase', 0.937)
        # rad -- TODO-MEASURE off the chassis, see README
        self.declare_parameter('max_steer_angle', 0.35)
        # +1 or -1; flip during blocks calibration if wheels steer away from the path
        self.declare_parameter('steer_sign', 1.0)
        # m/s -- ~2 mph, the guide's recommended first-drive speed
        self.declare_parameter('target_speed_mps', 0.9)
        # m/s -- 12.5 mph; MUST match serial_bridge_node's MAX_SPEED_MPH, see README
        self.declare_parameter('max_speed_mps', 5.588)
        self.declare_parameter('curvature_speed_gain', 0.0)      # 0 = constant target_speed_mps
        self.declare_parameter('min_speed_mps', 0.4)
        self.declare_parameter('goal_tolerance', 0.3)            # m
        # CSV path; empty -> subscribe path_topic instead
        self.declare_parameter('path_file', '')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('cmd_topic', '/cmd/auto')
        self.declare_parameter('control_rate', 30.0)             # Hz
        self.declare_parameter('odom_timeout', 0.5)              # s

        p = self.get_parameter
        self._lookahead_base = p('lookahead_distance').value
        self._lookahead_gain = p('lookahead_speed_gain').value
        self._min_lookahead = p('min_lookahead').value
        self._max_lookahead = p('max_lookahead').value
        self._closest_index_window = p('closest_index_window').value
        self._wheelbase = p('wheelbase').value
        self._max_steer_angle = p('max_steer_angle').value
        self._steer_sign = p('steer_sign').value
        self._target_speed_mps = p('target_speed_mps').value
        self._max_speed_mps = p('max_speed_mps').value
        self._curvature_speed_gain = p('curvature_speed_gain').value
        self._min_speed_mps = p('min_speed_mps').value
        self._goal_tolerance = p('goal_tolerance').value
        self._odom_timeout = p('odom_timeout').value
        cmd_topic = p('cmd_topic').value
        odom_topic = p('odom_topic').value
        path_topic = p('path_topic').value
        path_file = p('path_file').value

        self._path = []
        self._last_idx = 0
        self._goal_reached = False
        self._last_odom = None        # (x, y, yaw, speed_mps)
        self._last_odom_stamp = None  # rclpy.time.Time, set on receipt

        if path_file:
            try:
                self._path = ppc.load_csv_path(path_file)
                self.get_logger().info(f'Loaded {len(self._path)} waypoints from {path_file}')
            except (OSError, ValueError) as exc:
                self.get_logger().error(f'Failed to load path_file "{path_file}": {exc}')
        else:
            # route_publisher (per the guide) publishes this latched; match its
            # durability or this subscription will never see a path published
            # before it started.
            latched = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(Path, path_topic, self._on_path, latched)
            self.get_logger().info(
                f'path_file not set -- waiting for a latched Path on {path_topic}')

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._cmd_pub = self.create_publisher(VehicleCommand, cmd_topic, 10)

        control_rate = p('control_rate').value
        self._timer = self.create_timer(1.0 / control_rate, self._control_tick)

    def _on_path(self, msg: Path):
        pts = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if len(pts) < 2:
            self.get_logger().warning('Received Path with < 2 poses, ignoring')
            return
        self._path = ppc.path_from_xy(pts)
        self._last_idx = 0
        self._goal_reached = False
        self.get_logger().info(f'Loaded {len(self._path)} waypoints from /planning/path')

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = ppc.yaw_from_quaternion(msg.pose.pose.orientation)
        speed = msg.twist.twist.linear.x
        self._last_odom = (x, y, yaw, speed)
        self._last_odom_stamp = self.get_clock().now()

    def _publish_zero(self):
        cmd = VehicleCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(cmd)

    def _control_tick(self):
        if not self._path:
            self._publish_zero()
            return

        if self._last_odom is None or self._last_odom_stamp is None:
            self._publish_zero()
            return

        age_s = (self.get_clock().now() - self._last_odom_stamp).nanoseconds * 1e-9
        if age_s > self._odom_timeout:
            self.get_logger().warning(
                f'Odometry stale ({age_s:.2f}s > {self._odom_timeout:.2f}s) -- publishing zero',
                throttle_duration_sec=2.0)
            self._publish_zero()
            return

        x, y, yaw, speed = self._last_odom
        pose = (x, y)

        # Advance path-progress first -- goal detection needs it below, not
        # just raw distance (see is_near_path_end's docstring: a loop route's
        # last waypoint sits near its first one, so starting REPEAT from
        # wherever TEACH just ended would otherwise look like an instant
        # arrival before the vehicle has moved at all).
        self._last_idx = ppc.closest_index(
            self._path, pose, self._last_idx, window=self._closest_index_window)

        goal_dist = ppc.distance(pose, self._path[-1].xy)
        near_end = ppc.is_near_path_end(self._last_idx, len(self._path))
        if near_end and goal_dist <= self._goal_tolerance:
            if not self._goal_reached:
                self.get_logger().info(
                    f'Goal reached (within {self._goal_tolerance} m) -- stopping')
                self._goal_reached = True
            self._publish_zero()
            return

        lookahead_dist = ppc.scaled_lookahead(
            speed, self._lookahead_base, self._lookahead_gain,
            self._min_lookahead, self._max_lookahead)

        lookahead_pt, self._last_idx, _reached_end = ppc.find_lookahead_point(
            self._path, pose, self._last_idx, lookahead_dist)

        local_x, local_y = ppc.world_to_body(pose, yaw, lookahead_pt)
        actual_ld = math.hypot(local_x, local_y)
        if actual_ld < 1e-6:
            actual_ld = lookahead_dist

        curvature = ppc.curvature_from_local_y(local_y, actual_ld)
        delta = ppc.steering_angle(curvature, self._wheelbase, self._max_steer_angle)
        steer_cmd = ppc.normalize_steer(delta, self._max_steer_angle, self._steer_sign)

        speed_cmd = ppc.curvature_speed(
            self._target_speed_mps, curvature, self._curvature_speed_gain, self._min_speed_mps)
        throttle_cmd = ppc.clamp(speed_cmd / self._max_speed_mps, -1.0, 1.0)

        cmd = VehicleCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.throttle = float(throttle_cmd)
        cmd.steer = float(steer_cmd)
        cmd.brake = 0.0
        self._cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
