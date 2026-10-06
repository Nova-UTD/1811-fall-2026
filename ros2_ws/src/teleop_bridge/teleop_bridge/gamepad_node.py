import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from vehicle_msgs.msg import VehicleCommand

# --- Axis mapping (8BitDo SN30 Pro over USB / WIRED) ---
# Full layout as measured on this pad via `ros2 topic echo /joy`:
#   axes[0] left  stick X   -> (unused)    right = NEGATIVE
#   axes[1] left  stick Y   -> throttle    up    = POSITIVE
#   axes[2] left  trigger   -> brake       rests at +1.0, falls to -1.0 pressed
#   axes[3] right stick X   -> steering    right = NEGATIVE
#   axes[4] right stick Y   -> (unused)    up    = POSITIVE
#
# NOTE: axis numbers depend on the pad's mode, which is chosen by a button combo
# AT POWER-ON, not by the cable. Over Bluetooth this pad reported steering on
# axes[2] and the trigger on axes[5]. The constants below are for the WIRED
# layout above -- re-verify with `ros2 topic echo /joy` if the pad is ever
# repaired, power-cycled into another mode, or swapped.
AXIS_LEFT_STICK_Y = 1     # left stick vertical    -> throttle
AXIS_RIGHT_STICK_X = 3    # right stick horizontal -> steering
AXIS_LEFT_TRIGGER = 2     # left trigger           -> brake
HIGHEST_AXIS_USED = 3     # guard: a shorter axes[] than this means a mode/device change

INVERT_THROTTLE = False   # up = forward, and axes[1] up is already positive.
# axes[3] reports right as NEGATIVE; invert so right-stick = steer right.
# If it steers the wrong way on the vehicle, flip to False.
INVERT_STEER = True

# >>> Set this from what axes[2] (the brake trigger) reads AT REST <<<
#   - rests at ~0.0, rises toward 1.0 when pressed   -> False
#   - rests at ~+1.0, falls toward -1.0 when pressed -> True  (verified on this pad)
# If this is wrong, brake reads ~0.5 at rest, braking stays on, and throttle is
# permanently forced to 0 (the car will refuse to move). So check it.
TRIGGER_RESTS_AT_PLUS_ONE = True

DEADZONE = 0.05


class GamepadNode(Node):
    def __init__(self):
        super().__init__('gamepad_node')
        self.sub = self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.pub = self.create_publisher(VehicleCommand, '/vehicle_command', 10)

    def apply_deadzone(self, value: float) -> float:
        return 0.0 if abs(value) < DEADZONE else value

    def trigger_to_brake(self, raw: float) -> float:
        """Normalize a trigger axis to 0.0 (released) .. 1.0 (fully pressed)."""
        brake = (1.0 - raw) / 2.0 if TRIGGER_RESTS_AT_PLUS_ONE else raw
        return max(0.0, min(1.0, brake))

    def on_joy(self, msg: Joy):
        # A pad that re-enumerates in a different mode can publish a shorter
        # axes[]. Indexing past the end would raise inside this callback, which
        # takes down rclpy.spin() -- and with no firmware watchdog, the Arduino
        # then holds the LAST command forever. Drop the message instead.
        if len(msg.axes) <= HIGHEST_AXIS_USED:
            self.get_logger().error(
                f'/joy has {len(msg.axes)} axes, need at least {HIGHEST_AXIS_USED + 1} -- '
                'ignoring. The gamepad likely enumerated in a different mode; '
                're-check the axis mapping with `ros2 topic echo /joy`.',
                throttle_duration_sec=5.0)
            return

        cmd = VehicleCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()

        raw_throttle = self.apply_deadzone(msg.axes[AXIS_LEFT_STICK_Y])
        if INVERT_THROTTLE:
            raw_throttle = -raw_throttle

        raw_steer = self.apply_deadzone(msg.axes[AXIS_RIGHT_STICK_X])
        if INVERT_STEER:
            raw_steer = -raw_steer

        brake = self.trigger_to_brake(msg.axes[AXIS_LEFT_TRIGGER])
        braking = brake > DEADZONE

        # Brake overrides throttle -- never send both nonzero at once.
        cmd.throttle = 0.0 if braking else raw_throttle
        cmd.steer = raw_steer
        cmd.brake = brake if braking else 0.0

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = GamepadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
