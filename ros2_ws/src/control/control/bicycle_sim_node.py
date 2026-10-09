"""Closed-loop kinematic-bicycle simulator, for testing pure_pursuit_node
without hardware or a running KISS-ICP.

Subscribes to the same VehicleCommand pure_pursuit_node publishes, integrates
a simple bicycle model, and publishes the result back as nav_msgs/Odometry --
so pure_pursuit_node genuinely closes the loop against its own output instead
of just replaying a canned pose along the recorded path. Point it at
config/sample_loop.csv and watch (via `ros2 topic echo /odometry` or RViz)
whether it actually converges onto the path from an offset start.

    ros2 launch control pure_pursuit.launch.py use_sim:=true

Not a substitute for the real vehicle -- it has no watchdog, no braking
dynamics, no wheel slip. It only proves the pure-pursuit geometry and
sign/scale conversions are self-consistent before you burn bench time.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from vehicle_msgs.msg import VehicleCommand
import math

from control import pure_pursuit_core as ppc


class BicycleSimNode(Node):

    def __init__(self):
        super().__init__('bicycle_sim_node')

        self.declare_parameter('cmd_topic', '/cmd/auto')
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('wheelbase', 0.937)
        self.declare_parameter('max_steer_angle', 0.35)
        # m/s -- 12.5 mph, matches serial_bridge_node's MAX_SPEED_MPH
        self.declare_parameter('max_speed_mps', 5.588)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('rate', 50.0)
        # s -- stop trusting a stale command. NOTE: the real firmware has no
        # equivalent yet (repo root README's "Known issues": checkStaleness()
        # exists but isn't enabled) -- this sim is more forgiving than real
        # hardware right now, don't read a clean sim run as proof the vehicle
        # itself will stop safely.
        self.declare_parameter('cmd_timeout', 0.5)

        p = self.get_parameter
        self._wheelbase = p('wheelbase').value
        self._max_steer_angle = p('max_steer_angle').value
        self._max_speed_mps = p('max_speed_mps').value
        self._cmd_timeout = p('cmd_timeout').value

        self._x = p('start_x').value
        self._y = p('start_y').value
        self._yaw = p('start_yaw').value

        self._last_cmd = None         # (throttle, steer)
        self._last_cmd_stamp = None

        self.create_subscription(VehicleCommand, p('cmd_topic').value, self._on_cmd, 10)
        self._odom_pub = self.create_publisher(Odometry, p('odom_topic').value, 10)

        rate = p('rate').value
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._step)

    def _on_cmd(self, msg: VehicleCommand):
        # Brake is ignored for simplicity -- treat brake>0 as throttle=0, no
        # deceleration dynamics modeled.
        throttle = 0.0 if msg.brake > 0.0 else msg.throttle
        self._last_cmd = (throttle, msg.steer)
        self._last_cmd_stamp = self.get_clock().now()

    def _step(self):
        throttle, steer = 0.0, 0.0
        if self._last_cmd is not None and self._last_cmd_stamp is not None:
            age_s = (self.get_clock().now() - self._last_cmd_stamp).nanoseconds * 1e-9
            if age_s <= self._cmd_timeout:
                throttle, steer = self._last_cmd

        self._x, self._y, self._yaw, speed = ppc.step_bicycle_model(
            self._x, self._y, self._yaw, throttle, steer, self._dt,
            self._wheelbase, self._max_speed_mps, self._max_steer_angle)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        odom.twist.twist.linear.x = speed
        self._odom_pub.publish(odom)


def main():
    rclpy.init()
    node = BicycleSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
