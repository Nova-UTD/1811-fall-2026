"""The serial bridge rides out the Arduino dropping off USB and picks it up again.

On Nov 20 the Arduino dropped off USB while the car was running; the write in
the command callback raised, the bridge exited, and nothing reached the
Arduino again until the whole stack was restarted. These tests unplug a fake
Arduino (a pseudo-terminal, which fails like a removed USB serial device) and
plug it back in, through the real pyserial.
"""

import importlib.util
import os
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'ros2_ws' / 'src'

fake_vehicle_msgs = types.ModuleType('vehicle_msgs')
fake_msgs = types.ModuleType('vehicle_msgs.msg')
fake_msgs.VehicleCommand = type('VehicleCommand', (), {})
fake_msgs.VehicleState = type('VehicleState', (), {})
fake_vehicle_msgs.msg = fake_msgs
with patch.dict(sys.modules, {'vehicle_msgs': fake_vehicle_msgs, 'vehicle_msgs.msg': fake_msgs}):
    spec = importlib.util.spec_from_file_location(
        'reconnect_test_bridge', SRC / 'teleop_bridge/teleop_bridge/serial_bridge_node.py')
    bridge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bridge)


class FakeArduino:
    """A pseudo-terminal behind a stable path, like /dev/serial/by-id/<Arduino>."""

    def __init__(self, path):
        self.path = path
        self.master = None

    def plug_in(self):
        self.master, slave = os.openpty()
        slave_path = os.ttyname(slave)
        os.close(slave)
        if os.path.lexists(self.path):
            os.remove(self.path)
        os.symlink(slave_path, self.path)

    def unplug(self):
        os.close(self.master)  # the slave side now fails reads and writes, like a removed device
        self.master = None
        os.remove(self.path)

    def received(self, timeout=2.0):
        """Everything the bridge has sent since the last call."""
        os.set_blocking(self.master, False)
        data, deadline = b'', time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                chunk = os.read(self.master, 4096)
            except BlockingIOError:
                if data:
                    break
                time.sleep(0.02)
                continue
            data += chunk
        return data

    def send(self, text):
        os.write(self.master, text.encode('ascii'))


def command(throttle=0.0, steer=0.0, brake=0.0):
    return SimpleNamespace(throttle=throttle, steer=steer, brake=brake)


class ReconnectTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.arduino = FakeArduino(os.path.join(self.tmp.name, 'arduino'))
        self.log = Mock()
        self.now = 100.0
        self.link = bridge.ArduinoLink(self.arduino.path, 57600, self.log, clock=lambda: self.now)
        self.node = SimpleNamespace(link=self.link, get_logger=lambda: self.log)

    def tearDown(self):
        self.link.close()
        if self.arduino.master is not None:
            os.close(self.arduino.master)
        self.tmp.cleanup()

    def drive(self, **kw):
        """What the mode manager's 20 Hz command does: must never raise."""
        bridge.SerialBridgeNode.on_command(self.node, command(**kw))

    def test_unplug_and_replug_while_driving(self):
        self.arduino.plug_in()
        self.assertTrue(self.link.connect())
        self.drive(throttle=0.25)
        self.assertIn(b'"speed": 10.0', self.arduino.received())

        self.arduino.unplug()
        for _ in range(20):         # a second of commands with no Arduino
            self.drive(throttle=0.25)
        self.assertFalse(self.link.connected)
        self.assertEqual(self.log.error.call_count, 1, 'one clear error, not one per command')
        self.assertIn('Lost the Arduino', self.log.error.call_args.args[0])

        self.now += 3.0
        self.assertFalse(self.link.connect())   # still unplugged: waits, doesn't raise
        self.arduino.plug_in()
        self.assertTrue(self.link.connect())     # the 1 s timer's next try
        reconnected = self.log.info.call_args.args[0]
        self.assertIn('Reconnected', reconnected)
        self.assertIn('after 3.0 s', reconnected)
        self.assertIn('20 drive commands were not delivered', reconnected)

        self.drive(throttle=-0.5, steer=0.2)
        self.assertIn(b'"speed": -20.0, "steering": 0.2', self.arduino.received())

    def test_unplug_while_the_reader_is_waiting(self):
        self.arduino.plug_in()
        self.assertTrue(self.link.connect())
        lines = []

        def reader():
            while len(lines) < 3:
                line = self.link.readline()
                if line is None:
                    lines.append(None)
                    return
                if line:
                    lines.append(line)

        thread = threading.Thread(target=reader, daemon=True)
        thread.start()
        self.arduino.send('{"speed": 1.0}\n')
        time.sleep(0.3)
        self.arduino.unplug()
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive(), 'the reader must not hang on a removed device')
        self.assertEqual(lines[0].strip(), b'{"speed": 1.0}')
        self.assertIsNone(lines[-1])
        self.drive()                 # the writer finds the port already closed: no second error
        self.assertEqual(self.log.error.call_count, 1)

    def test_no_arduino_at_startup_keeps_waiting(self):
        self.assertFalse(self.link.connect())
        self.assertEqual(self.log.warning.call_count, 1)
        self.assertIn('Waiting for the Arduino', self.log.warning.call_args.args[0])
        self.drive(throttle=1.0)     # commands before it appears are dropped, not fatal
        for _ in range(5):           # retries every second don't flood the log...
            self.now += 1.0
            self.link.connect()
        self.assertEqual(self.log.warning.call_count, 1)
        self.now += 10.0             # ...but it reminds every 10 s
        self.link.connect()
        self.assertEqual(self.log.warning.call_count, 2)

        self.arduino.plug_in()
        self.assertTrue(self.link.connect())
        self.assertIn('Connected to the Arduino on', self.log.info.call_args.args[0])
        self.drive(throttle=0.5)
        self.assertIn(b'"speed": 20.0', self.arduino.received())

    def test_replug_under_another_tty_number_follows_the_by_id_link(self):
        by_id = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_X-if00'
        link = bridge.ArduinoLink('/dev/ttyACM0', 57600, self.log)
        with patch.object(bridge.os.path, 'exists', lambda p: p != '/dev/ttyACM0'), \
                patch.object(bridge.glob, 'glob', lambda pattern: [by_id] if 'by-id' in pattern else []):
            self.assertEqual(link._pick_port(), by_id)
        with patch.object(bridge.os.path, 'exists', lambda p: True):
            self.assertEqual(link._pick_port(), '/dev/ttyACM0')   # the configured port wins while it exists

    def test_out_of_range_commands_are_still_clamped(self):
        line, clamped = bridge.command_line(command(throttle=2.0, steer=-3.0, brake=-1.0))
        self.assertTrue(clamped)
        self.assertEqual(line, b'{"speed": 40.0, "steering": -1.0, "braking": 0.0}\n')
        self.assertFalse(bridge.command_line(command(throttle=0.5))[1])


class LaunchRespawnTest(unittest.TestCase):
    def test_drive_chain_nodes_restart_if_they_die(self):
        sources = {
            'serial_bridge_node': (SRC / 'teleop_bridge/launch/teleop_bridge.launch.py').read_text(),
            'joy_node': (SRC / 'obc_bringup/launch/bringup.launch.py').read_text(),
            'gamepad_node': (SRC / 'obc_bringup/launch/bringup.launch.py').read_text(),
            'mode_manager_node': (SRC / 'obc_bringup/launch/bringup.launch.py').read_text(),
        }
        for name, text in sources.items():
            with self.subTest(node=name):
                start = text.index(f"name='{name}'")
                end = text.find('Node(', start)   # this node's arguments end where the next node starts
                self.assertIn('respawn=True', text[start:end if end != -1 else len(text)])


if __name__ == '__main__':
    unittest.main()
