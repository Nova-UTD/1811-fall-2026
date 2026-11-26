import math

import rclpy
from rclpy.executors import ExternalShutdownException
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
# Scale manual throttle only; the bridge's 40 mph normalization also serves autonomy.
MANUAL_THROTTLE_SCALE = 0.75

# Manual throttle curve. Squared, so small stick movements give fine control
# at low speed while full stick still reaches full speed. It starts at
# MIN_MOVING_MPH rather than 0: the VESC ignores speed commands below its
# minimum (900 ERPM, about 4.6 mph with the firmware's wheel constants), so a
# curve from 0 would leave the first ~40% of the stick doing nothing. Stick
# just past the deadzone asks for MIN_MOVING_MPH; full stick asks for
# MANUAL_THROTTLE_SCALE of the bridge's maximum (30 mph).
MIN_MOVING_MPH = 5.0
BRIDGE_MAX_SPEED_MPH = 40.0   # must match serial_bridge_node.MAX_SPEED_MPH
MIN_MOVING_THROTTLE = MIN_MOVING_MPH / BRIDGE_MAX_SPEED_MPH


def throttle_curve(stick: float) -> float:
    """Map a stick value (-1..1, after the deadzone) to a normalized throttle, keeping its sign."""
    if abs(stick) < DEADZONE:
        return 0.0
    travel = min(1.0, (abs(stick) - DEADZONE) / (1.0 - DEADZONE))  # 0 at the deadzone, 1 at full
    shaped = MIN_MOVING_THROTTLE + (MANUAL_THROTTLE_SCALE - MIN_MOVING_THROTTLE) * travel ** 2
    return math.copysign(shaped, stick)


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
        # axes[]. Indexing past the end would raise inside this callback and
        # take down rclpy.spin(). Drop the message so the mode manager can
        # detect that manual commands have gone stale.
        if len(msg.axes) <= HIGHEST_AXIS_USED:
            self.get_logger().error(
                f'Gamepad on /joy reported only {len(msg.axes)} axes '
                f'(need at least {HIGHEST_AXIS_USED + 1}) — ignoring input. '
                'The controller may have started in a different mode; verify `/joy` axes.',
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
        cmd.throttle = 0.0 if braking else throttle_curve(raw_throttle)
        cmd.steer = raw_steer
        cmd.brake = brake if braking else 0.0

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = GamepadNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C is a normal stop, not a crash
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
