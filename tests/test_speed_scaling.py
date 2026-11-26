"""Check gamepad and autonomous speed scaling without a vehicle or serial device."""

import importlib.util
import json
import math
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import yaml


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'ros2_ws' / 'src'
sys.path.insert(0, str(SRC / 'control'))
sys.path.insert(0, str(SRC / 'teleop_bridge'))


class FakeVehicleCommand:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.throttle = 0.0
        self.steer = 0.0
        self.brake = 0.0


# Import the real callbacks without requiring generated vehicle_msgs in this test.
fake_vehicle_msgs = types.ModuleType('vehicle_msgs')
fake_msgs = types.ModuleType('vehicle_msgs.msg')
fake_msgs.VehicleCommand = FakeVehicleCommand
fake_msgs.VehicleState = type('VehicleState', (), {})
fake_vehicle_msgs.msg = fake_msgs
with patch.dict(sys.modules, {'vehicle_msgs': fake_vehicle_msgs, 'vehicle_msgs.msg': fake_msgs}):
    bridge_spec = importlib.util.spec_from_file_location(
        'speed_test_bridge', SRC / 'teleop_bridge/teleop_bridge/serial_bridge_node.py')
    bridge = importlib.util.module_from_spec(bridge_spec)
    bridge_spec.loader.exec_module(bridge)
    gamepad_spec = importlib.util.spec_from_file_location(
        'speed_test_gamepad', SRC / 'teleop_bridge/teleop_bridge/gamepad_node.py')
    gamepad = importlib.util.module_from_spec(gamepad_spec)
    gamepad_spec.loader.exec_module(gamepad)
    pursuit_spec = importlib.util.spec_from_file_location(
        'speed_test_pursuit', SRC / 'control/control/pure_pursuit_node.py')
    pursuit = importlib.util.module_from_spec(pursuit_spec)
    pursuit_spec.loader.exec_module(pursuit)


class FakeClock:
    def now(self):
        return self

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=0)

    def to_msg(self):
        return None


class SpeedScalingTest(unittest.TestCase):
    def setUp(self):
        self.link = Mock()
        self.bridge = SimpleNamespace(link=self.link, get_logger=lambda: Mock())

    def serial_speed(self, command):
        bridge.SerialBridgeNode.on_command(self.bridge, command)
        return json.loads(self.link.write.call_args.args[0])['speed']

    def test_serial_normalization_and_clamping(self):
        for throttle, expected in ((-1.5, -40.0), (-0.5, -20.0), (0.0, 0.0),
                                   (0.5, 20.0), (1.0, 40.0), (1.5, 40.0)):
            with self.subTest(throttle=throttle):
                command = FakeVehicleCommand()
                command.throttle = throttle
                self.assertEqual(self.serial_speed(command), expected)

    def test_manual_stick_follows_the_curve_without_scaling_steering_or_brake(self):
        publisher = Mock()
        node = SimpleNamespace(
            apply_deadzone=lambda value: gamepad.GamepadNode.apply_deadzone(None, value),
            trigger_to_brake=lambda value: gamepad.GamepadNode.trigger_to_brake(None, value),
            get_clock=FakeClock, pub=publisher,
        )
        # Full stick is still 75% (30 mph); half stick is squared: 5 + 25 * (0.45 / 0.95)^2.
        for throttle, expected in ((-1.0, -30.0), (-0.5, -10.609), (0.0, 0.0),
                                   (0.5, 10.609), (1.0, 30.0)):
            with self.subTest(throttle=throttle):
                joy = SimpleNamespace(axes=[0.0, throttle, 1.0, -0.6])
                gamepad.GamepadNode.on_joy(node, joy)
                command = publisher.publish.call_args.args[0]
                self.assertEqual(self.serial_speed(command), expected)
                self.assertAlmostEqual(command.steer, 0.6)
                self.assertEqual(command.brake, 0.0)

        gamepad.GamepadNode.on_joy(node, SimpleNamespace(axes=[0.0, 1.0, -1.0, -0.6]))
        command = publisher.publish.call_args.args[0]
        self.assertEqual(self.serial_speed(command), 0.0)
        self.assertEqual(command.brake, 1.0)
        self.assertAlmostEqual(command.steer, 0.6)

    def test_manual_curve_starts_where_the_vesc_starts_moving(self):
        mph = lambda stick: gamepad.throttle_curve(stick) * bridge.MAX_SPEED_MPH  # noqa: E731
        self.assertEqual(gamepad.throttle_curve(0.0), 0.0)
        self.assertEqual(gamepad.throttle_curve(gamepad.DEADZONE - 1e-6), 0.0)
        # Just past the deadzone already asks for the minimum moving speed...
        self.assertAlmostEqual(mph(gamepad.DEADZONE), gamepad.MIN_MOVING_MPH)
        self.assertAlmostEqual(mph(-gamepad.DEADZONE), -gamepad.MIN_MOVING_MPH)
        # ...which the VESC turns at: its default minimum is 900 ERPM, and the
        # firmware sends 196 ERPM per mph (12 in wheel, 7 pole pairs).
        erpm_per_mph = 5280 / 60 / math.pi * 7
        self.assertGreater(gamepad.MIN_MOVING_MPH * erpm_per_mph, 900)
        # Rises with the stick all the way, reaching the old full-stick speed.
        sticks = [gamepad.DEADZONE + i * (1 - gamepad.DEADZONE) / 50 for i in range(51)]
        speeds = [mph(s) for s in sticks]
        self.assertTrue(all(b > a for a, b in zip(speeds, speeds[1:])))
        self.assertAlmostEqual(speeds[-1], 30.0)
        # Squared: half travel gives a quarter of the range above the minimum.
        self.assertAlmostEqual(mph(gamepad.DEADZONE + (1 - gamepad.DEADZONE) / 2), 5 + 25 / 4)
        self.assertAlmostEqual(mph(-0.7), -mph(0.7))  # reverse mirrors forward

    def test_gamepad_and_bridge_agree_on_the_maximum_speed(self):
        self.assertEqual(gamepad.BRIDGE_MAX_SPEED_MPH, bridge.MAX_SPEED_MPH)

    def test_autonomous_target_still_reaches_serial_as_same_speed(self):
        config = yaml.safe_load((SRC / 'control/config/pure_pursuit.yaml').read_text())['/**']['ros__parameters']
        publisher = Mock()
        node = SimpleNamespace(
            _path=pursuit.ppc.path_from_xy([(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]),
            _last_idx=0, _last_odom=(0.0, 0.0, 0.0, 0.0), _last_odom_stamp=FakeClock(),
            _odom_timeout=config['odom_timeout'], _closest_index_window=config['closest_index_window'],
            _goal_tolerance=config['goal_tolerance'], _goal_reached=False,
            _lookahead_base=config['lookahead_distance'], _lookahead_gain=config['lookahead_speed_gain'],
            _min_lookahead=config['min_lookahead'], _max_lookahead=config['max_lookahead'],
            _wheelbase=config['wheelbase'], _max_steer_angle=config['max_steer_angle'],
            _steer_sign=config['steer_sign'], _target_speed_mps=config['target_speed_mps'],
            _max_speed_mps=config['max_speed_mps'], _curvature_speed_gain=config['curvature_speed_gain'],
            _min_speed_mps=config['min_speed_mps'], _cmd_pub=publisher,
            get_clock=FakeClock, get_logger=lambda: Mock(),
        )
        pursuit.PurePursuitNode._control_tick(node)
        command = publisher.publish.call_args.args[0]
        self.assertAlmostEqual(self.serial_speed(command), 0.9 / 0.44704, places=3)


if __name__ == '__main__':
    unittest.main()
