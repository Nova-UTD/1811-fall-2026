"""
mode_manager_node -- the only node that publishes /vehicle_command.

Subscribes:
  /joy          sensor_msgs/Joy          mode buttons, deadman, brake takeover
  /cmd/manual   vehicle_msgs/VehicleCommand  gamepad_node (remapped off /vehicle_command)
  /cmd/auto     vehicle_msgs/VehicleCommand  pure_pursuit_node
Publishes:
  /vehicle_command  VehicleCommand at publish_rate, whatever the current mode allows
  /guardian/mode    std_msgs/String, latched -- DISABLED / MANUAL / AUTONOMOUS
Calls (record button):
  /route_recorder_node/start_recording, stop_recording, save

Publishing at a fixed rate (not only on input) keeps the Arduino's 250 ms
dead-link cutoff fed with a deliberate command, so a quiet source turns into an
explicit stop from here instead of a coast from the firmware.

All decisions live in mode_manager_core; this file only moves messages.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vehicle_msgs.msg import VehicleCommand

from mode_manager import mode_manager_core as mmc


def to_core(msg: VehicleCommand) -> mmc.Command:
    return mmc.Command(throttle=msg.throttle, steer=msg.steer, brake=msg.brake)


class ModeManagerNode(Node):

    def __init__(self):
        super().__init__('mode_manager_node')

        defaults = mmc.GamepadMap()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter('input_timeout', 0.5)     # s -- older input counts as lost
        self.declare_parameter('stop_brake', 1.0)        # 0..1 -- brake used for a forced stop
        self.declare_parameter('takeover_brake', 0.2)    # 0..1 -- trigger past this = takeover
        self.declare_parameter('publish_rate', 20.0)     # Hz -- well inside the 250 ms cutoff
        self.declare_parameter('recorder_node', '/route_recorder_node')

        p = self.get_parameter
        gamepad = mmc.GamepadMap(**{name: p(name).value for name in vars(defaults)})
        self._mm = mmc.ModeManager(
            gamepad=gamepad,
            input_timeout_s=p('input_timeout').value,
            stop_brake=p('stop_brake').value,
            takeover_brake=p('takeover_brake').value,
        )

        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.create_subscription(VehicleCommand, '/cmd/manual', self._on_manual, 10)
        self.create_subscription(VehicleCommand, '/cmd/auto', self._on_auto, 10)
        self._cmd_pub = self.create_publisher(VehicleCommand, '/vehicle_command', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._mode_pub = self.create_publisher(String, '/guardian/mode', latched)

        recorder = p('recorder_node').value.rstrip('/')
        self._rec_start = self.create_client(Trigger, f'{recorder}/start_recording')
        self._rec_stop = self.create_client(Trigger, f'{recorder}/stop_recording')
        self._rec_save = self.create_client(Trigger, f'{recorder}/save')
        self._recording = False
        self._record_busy = False

        self._last_mode = None
        self._last_deadman = False
        self._publish_mode()
        self.create_timer(1.0 / p('publish_rate').value, self._tick)
        self.get_logger().info(
            f'Mode manager up in {self._mm.mode}. Buttons: manual={gamepad.manual_button} '
            f'disable={gamepad.disable_button} autonomous={gamepad.autonomous_button} '
            f'deadman={gamepad.deadman_button} record={gamepad.record_button}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs -----------------------------------------------------------

    def _on_joy(self, msg: Joy):
        reason_before = self._mm.reason
        events = self._mm.on_joy(msg.buttons, msg.axes, self._now())
        if self._mm.reason != reason_before and self._mm.reason.startswith(mmc.REFUSED):
            self.get_logger().warning(self._mm.reason)
        self._publish_mode()

        deadman = self._mm.deadman_held
        if self._mm.mode == mmc.AUTONOMOUS and deadman != self._last_deadman:
            self.get_logger().info('deadman held -- driving' if deadman
                                   else 'deadman released -- stopping')
        self._last_deadman = deadman

        if mmc.EVENT_RECORD_TOGGLE in events:
            self._toggle_recording()

    def _on_manual(self, msg: VehicleCommand):
        self._mm.on_manual(to_core(msg), self._now())

    def _on_auto(self, msg: VehicleCommand):
        self._mm.on_auto(to_core(msg), self._now())

    # --- output -----------------------------------------------------------

    def _tick(self):
        out = self._mm.output(self._now())
        cmd = VehicleCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.throttle = float(out.throttle)
        cmd.steer = float(out.steer)
        cmd.brake = float(out.brake)
        self._cmd_pub.publish(cmd)

    def _publish_mode(self):
        if self._mm.mode == self._last_mode:
            return
        self._last_mode = self._mm.mode
        self.get_logger().info(f'MODE -> {self._mm.mode} ({self._mm.reason})')
        self._mode_pub.publish(String(data=self._mm.mode))

    # --- record button ------------------------------------------------------

    def _toggle_recording(self):
        if self._record_busy:
            return
        if self._mm.mode == mmc.AUTONOMOUS:
            self.get_logger().warning('record button ignored while AUTONOMOUS')
            return
        if not self._rec_start.service_is_ready():
            self.get_logger().error('route_recorder_node not reachable -- is bringup running?')
            return
        self._record_busy = True
        if not self._recording:
            self._call(self._rec_start, self._after_start)
        else:
            self._call(self._rec_stop, self._after_stop)

    def _call(self, client, then):
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: then(f.result()))

    def _after_start(self, result):
        self._record_busy = False
        if result is not None and result.success:
            self._recording = True
            self.get_logger().info(f'TEACH: {result.message}')
        else:
            self.get_logger().error(f'start_recording failed: {result}')

    def _after_stop(self, result):
        if result is None or not result.success:
            self._record_busy = False
            self.get_logger().error(f'stop_recording failed: {result}')
            return
        self._recording = False
        self._call(self._rec_save, self._after_save)

    def _after_save(self, result):
        self._record_busy = False
        if result is not None and result.success:
            self.get_logger().info(
                f'TEACH done: {result.message}. Route sent to pure_pursuit -- '
                'watch /cmd/auto for a dry run, then Start + hold deadman to repeat.')
        else:
            self.get_logger().error(f'save failed: {result}')


def main():
    rclpy.init()
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
