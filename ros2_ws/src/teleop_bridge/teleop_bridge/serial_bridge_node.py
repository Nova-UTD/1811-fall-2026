import glob
import json
import os
import threading

import rclpy
import serial
from rclpy.node import Node
from vehicle_msgs.msg import VehicleCommand, VehicleState

MAX_SPEED_MPH = 12.5  # lowered for bench/first-drive testing; raise once trusted


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


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 57600)
        baud = self.get_parameter('baud').value

        port = resolve_port(self.get_parameter('port').value)
        # Log the resolved device unconditionally: "which port did it actually
        # open" is the first question every time the vehicle doesn't move.
        self.get_logger().info(f'Opening serial port {port} at {baud} baud')
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as exc:
            raise RuntimeError(
                f'Could not open {port}: {exc}. Available: '
                f'{sorted(glob.glob("/dev/serial/by-id/*")) or "none"} / '
                f'{sorted(glob.glob("/dev/ttyACM*")) or "no /dev/ttyACM*"}') from exc

        self.sub = self.create_subscription(
            VehicleCommand, '/vehicle_command', self.on_command, 10)
        self.state_pub = self.create_publisher(VehicleState, '/vehicle_state', 10)

        self._stop = False
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def on_command(self, msg: VehicleCommand):
        # Clamp before anything reaches the firmware. VehicleCommand.steer and
        # .throttle are defined as -1..1 and .brake as 0..1, but nothing
        # upstream enforced it: gamepad_node passes msg.axes[i] straight
        # through, so a bad axis index or a miscalibrated device could put an
        # out-of-range value on the wire and drive the steering past its
        # mechanical limit. This is the last line of defense before hardware.
        throttle = max(-1.0, min(1.0, msg.throttle))
        steer = max(-1.0, min(1.0, msg.steer))
        brake = max(0.0, min(1.0, msg.brake))
        if (throttle, steer, brake) != (msg.throttle, msg.steer, msg.brake):
            self.get_logger().warning(
                f'Command out of range, clamped: throttle={msg.throttle:.3f} '
                f'steer={msg.steer:.3f} brake={msg.brake:.3f}',
                throttle_duration_sec=2.0)

        # throttle is normalized -1..1; firmware wants an actual target speed in mph.
        speed_mph = throttle * MAX_SPEED_MPH
        payload = {
            "speed": round(speed_mph, 3),
            "steering": round(steer, 3),
            "braking": round(brake, 3),
        }
        line = json.dumps(payload) + "\n"
        self.ser.write(line.encode('ascii'))

    def _read_loop(self):
        # The firmware may send nothing, a JSON object, or (as observed) bare
        # numbers / debug text. Parse defensively so a stray line never kills
        # this thread and silently stops state read-back.
        while not self._stop and rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException as exc:
                # Don't `continue` here: if the Arduino is unplugged, readline()
                # fails instantly and forever, and a bare continue spins a hot
                # loop that floods the log. The port is gone; stop reading.
                self.get_logger().error(f'Serial read failed, stopping read thread: {exc}')
                return
            if not line:
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                self.get_logger().warning(
                    f'Non-JSON serial line, ignoring: {line!r}',
                    throttle_duration_sec=5.0)
                continue
            # json.loads happily returns floats/ints/lists for lines like "2.5".
            # We only handle JSON objects; skip anything else.
            if not isinstance(data, dict):
                self.get_logger().warning(
                    f'Serial line is not a JSON object, ignoring: {line!r}',
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
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = SerialBridgeNode()
    except RuntimeError as exc:
        # Under `ros2 launch` a raw traceback scrolls past among the other
        # nodes' output and reads as "everything started fine". Make the one
        # line that matters greppable.
        rclpy.logging.get_logger('serial_bridge_node').fatal(
            f'SERIAL BRIDGE DID NOT START -- the vehicle will not move. {exc}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
