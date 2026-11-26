import glob
import json
import os
import threading
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vehicle_msgs.msg import VehicleCommand, VehicleState

MAX_SPEED_MPH = 40.0  # Target at normalized command 1.0; actual VESC speed is unverified.
RECONNECT_PERIOD_S = 1.0    # how often to try the Arduino again while it's missing
WAITING_REMINDER_S = 10.0   # repeat the "waiting for the Arduino" warning this often

# What a serial port raises when the device under it disappears. A port that
# another thread closes mid-read raises the last three.
PORT_ERRORS = (serial.SerialException, OSError, TypeError, AttributeError, ValueError)


def resolve_port(requested):
    """Turn port='auto' into a concrete device path.

    /dev/ttyACM* numbers are assigned in plug order and shift across replugs
    and reboots -- pointing the bridge at whatever happens to be ACM2 today is
    how it silently ended up writing JSON at a Linux serial console. The
    /dev/serial/by-id/ links are derived from the device's USB descriptors, so
    they follow the Arduino instead of the enumeration order.

    Anything other than 'auto' (or empty) is passed through untouched, so an
    explicit port:=/dev/ttyACM0 still overrides.
    """
    if requested and requested != 'auto':
        return requested

    candidates = sorted(glob.glob('/dev/serial/by-id/*'))
    if not candidates:
        raise RuntimeError(
            "port:='auto' found nothing in /dev/serial/by-id/. The Arduino is "
            'not enumerated on the host. Check `ls -l /dev/serial/by-id/` and '
            '`ls /dev/ttyACM*` OUTSIDE the container first -- the /dev '
            'bind-mount only reflects what the host already sees. On WSL, '
            'run `usbipd attach` again (it does not survive a replug).')

    arduinos = [c for c in candidates if 'arduino' in os.path.basename(c).lower()]
    if len(arduinos) == 1:
        return arduinos[0]
    if len(arduinos) > 1:
        raise RuntimeError(
            f'port:=auto found {len(arduinos)} Arduino-like devices: {arduinos}. '
            'Pass the right one explicitly with port:=<path>.')
    if len(candidates) == 1:
        return candidates[0]
    raise RuntimeError(
        f'port:=auto found {len(candidates)} serial devices but none look like an '
        f'Arduino: {candidates}. Pass one explicitly with port:=<path>.')


def command_line(msg):
    """
    Clamp a VehicleCommand and encode it as the firmware's JSON line.

    Returns (line as bytes, whether anything had to be clamped).
    """
    # Clamp before anything reaches the firmware. VehicleCommand.steer and
    # .throttle are defined as -1..1 and .brake as 0..1, but nothing
    # upstream enforced it: gamepad_node derives commands from the axes
    # without clamping, so a miscalibrated device could put an
    # out-of-range value on the wire and drive the steering past its
    # mechanical limit. This is the last line of defense before hardware.
    throttle = max(-1.0, min(1.0, msg.throttle))
    steer = max(-1.0, min(1.0, msg.steer))
    brake = max(0.0, min(1.0, msg.brake))
    clamped = (throttle, steer, brake) != (msg.throttle, msg.steer, msg.brake)

    # throttle is normalized -1..1; firmware wants an actual target speed in mph.
    payload = {
        'speed': round(throttle * MAX_SPEED_MPH, 3),
        'steering': round(steer, 3),
        'braking': round(brake, 3),
    }
    return (json.dumps(payload) + '\n').encode('ascii'), clamped


class ArduinoLink:
    """
    The serial connection to the Arduino, opened again whenever it drops.

    The Arduino can drop off USB while the car runs (a loose cable, the board
    losing power). That used to raise out of the command callback and kill
    the whole bridge, so nothing reached the Arduino again until the stack was
    restarted, even once it was plugged back in. Now a failed read or write
    closes the port and connect(), called every RECONNECT_PERIOD_S, opens it
    again. Closing at once also matters for the replug: while the dead port
    stays open the kernel can't reuse its /dev/ttyACM number, so the Arduino
    would come back as the next one.
    """

    def __init__(self, requested, baud, logger, open_serial=serial.Serial, clock=time.monotonic):
        self._requested = requested
        self._baud = baud
        self._log = logger
        self._open_serial = open_serial
        self._clock = clock
        self._lock = threading.Lock()
        self._ser = None
        self.port = None          # the device last opened
        self._lost_at = None      # when the connection dropped; None before the first one
        self._dropped = 0         # drive commands that couldn't be sent since then
        self._waiting = None      # (reason, time logged) while the Arduino can't be opened

    @property
    def connected(self):
        return self._ser is not None

    def _pick_port(self):
        port = resolve_port(self._requested)
        if self._requested not in ('', 'auto') and not os.path.exists(port):
            # After a replug the Arduino can come back under another
            # /dev/ttyACM number; follow its /dev/serial/by-id/ link instead.
            try:
                return resolve_port('auto')
            except RuntimeError:
                pass  # nothing better: report the configured port
        return port

    def connect(self):
        """Open the port unless it is open already. Returns True when connected."""
        if self._ser is not None:
            return True
        try:
            port = self._pick_port()
            ser = self._open_serial(port, self._baud, timeout=0.1)
        except RuntimeError as exc:
            self._report_waiting(str(exc))
            return False
        except (serial.SerialException, OSError, ValueError) as exc:
            self._report_waiting(
                f'{exc}. Available: {sorted(glob.glob("/dev/serial/by-id/*")) or "none"} / '
                f'{sorted(glob.glob("/dev/ttyACM*")) or "no /dev/ttyACM*"}')
            return False
        with self._lock:
            self._ser, self.port = ser, port
        # Log the device unconditionally: "which port did it actually open" is
        # the first question every time the vehicle doesn't move.
        if self._lost_at is None:
            self._log.info(f'Connected to the Arduino on {port} at {self._baud} baud.')
        else:
            self._log.info(
                f'Reconnected to the Arduino on {port} after '
                f'{self._clock() - self._lost_at:.1f} s without it '
                f'({self._dropped} drive commands were not delivered). Driving works again.')
        self._waiting = None
        self._dropped = 0
        return True

    def _report_waiting(self, reason):
        now = self._clock()
        if (self._waiting and self._waiting[0] == reason
                and now - self._waiting[1] < WAITING_REMINDER_S):
            return
        self._waiting = (reason, now)
        self._log.warning(
            f'Waiting for the Arduino, trying every {RECONNECT_PERIOD_S:.0f} s; '
            f'drive commands cannot reach it until then. {reason}')

    def write(self, data):
        """Send bytes. False if the Arduino isn't connected (never raises)."""
        ser = self._ser
        if ser is None:
            self._dropped += 1
            return False
        try:
            ser.write(data)
        except PORT_ERRORS as exc:
            self._lose(ser, f'sending a drive command ({exc})')
            self._dropped += 1
            return False
        return True

    def readline(self):
        """Return one line: b'' after the 0.1 s timeout, None while disconnected."""
        ser = self._ser
        if ser is None:
            return None
        try:
            return ser.readline()
        except PORT_ERRORS as exc:
            self._lose(ser, f'reading ({exc})')
            return None

    def _lose(self, ser, what):
        with self._lock:
            if self._ser is not ser:
                return  # the other thread got here first
            self._ser = None
            self._lost_at = self._clock()
            self._dropped = 0
        try:
            ser.close()
        except PORT_ERRORS:
            pass
        self._log.error(
            f'Lost the Arduino on {self.port} while {what}. Drive commands cannot reach '
            'it, so the motor stops and the car coasts. Reconnecting every '
            f'{RECONNECT_PERIOD_S:.0f} s; if this keeps happening, check its USB cable and power.')

    def close(self):
        with self._lock:
            ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except PORT_ERRORS:
                pass


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 57600)
        self.link = ArduinoLink(self.get_parameter('port').value,
                                self.get_parameter('baud').value, self.get_logger())
        # Not finding the Arduino at startup isn't fatal either: keep trying,
        # so plugging it in afterwards is enough.
        self.link.connect()
        self.create_timer(RECONNECT_PERIOD_S, self.link.connect)

        self.sub = self.create_subscription(
            VehicleCommand, '/vehicle_command', self.on_command, 10)
        self.state_pub = self.create_publisher(VehicleState, '/vehicle_state', 10)

        self._stop = False
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def on_command(self, msg: VehicleCommand):
        line, clamped = command_line(msg)
        if clamped:
            self.get_logger().warning(
                f'Clamped out-of-range drive command to safe limits: '
                f'throttle={msg.throttle:.3f}, steer={msg.steer:.3f}, brake={msg.brake:.3f}.',
                throttle_duration_sec=2.0)
        self.link.write(line)

    def _read_loop(self):
        # The firmware may send nothing, a JSON object, or (as observed) bare
        # numbers / debug text. Parse defensively so a stray line never kills
        # this thread and silently stops state read-back.
        while not self._stop and rclpy.ok():
            raw = self.link.readline()
            if raw is None:
                time.sleep(0.1)  # disconnected; the reconnect timer opens it again
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            if not line:
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                self.get_logger().warning(
                    f'Ignored malformed non-JSON line from Arduino: {line!r}',
                    throttle_duration_sec=5.0)
                continue
            # json.loads happily returns floats/ints/lists for lines like "2.5".
            # We only handle JSON objects; skip anything else.
            if not isinstance(data, dict):
                self.get_logger().warning(
                    f'Ignored unexpected serial payload (expected JSON object): {line!r}',
                    throttle_duration_sec=5.0)
                continue
            state = VehicleState()
            state.header.stamp = self.get_clock().now().to_msg()
            # mph -> m/s, if firmware echoes speed back
            state.speed_mps = float(data.get('speed', 0.0)) * 0.44704
            state.battery_v = float(data.get('battery', 0.0))
            state.watchdog_tripped = bool(data.get('watchdog', 0))
            self.state_pub.publish(state)

    def destroy_node(self):
        self._stop = True
        self.link.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C is a normal stop, not a crash
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
