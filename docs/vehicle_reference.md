# 1811 Vehicle Reference

Start with the [README](../README.md) for setup and the [driving guide](gamepad_controls.md#how-to-run-the-car) for vehicle operation.

## Host setup and recovery

Run `./scripts/vehicle.sh --help` from the repository root for all setup, container, and Jetson commands. `setup-machine` prepares Docker and fetches vendor libraries. If it asks you to log out after adding your user to the Docker group, sign in again and rerun it.

Use `./scripts/vehicle.sh dev` to open a development shell. Run `rebuild` in that shell to rebuild and source the workspace after changing packages or compiled code.

After a failed build, stop the stack and run `./reload` on the Karbon. It keeps the old build and install outputs under `ros2_ws/.reload-backups/` and restores them if the rebuild fails. After launch it checks for a real message on `/ouster/points`. If the build passes and no cloud arrives, check LiDAR power, Ethernet, and UDP traffic.

### Windows and WSL

In Docker Desktop, enable **Settings → Resources → WSL Integration** for Ubuntu. Run the project's shell commands from Ubuntu. Graphical tools use the host's `DISPLAY`, which you can check in WSL before starting the container.

For a Windows-connected Arduino or gamepad, install `usbipd-win`, then run these commands in Windows PowerShell. Run `bind` as administrator.

```powershell
usbipd list
usbipd bind --busid <busid>
usbipd attach --wsl --busid <busid>
```

On unplug or reboot, attach the device again. Confirm it appears under `/dev/serial/by-id/` or `/dev/input/` in WSL before opening the development container.

## Power and hardware

Turn on the precharge switch and wait a few seconds before turning the key. The wait limits current into the motor controller when the contactor closes. Turn the contactor key off before leaving the vehicle. Follow the [vehicle checklist](gamepad_controls.md#how-to-run-the-car) before enabling motion.

| Component | Supply or connection |
| --- | --- |
| Karbon 800 | 24 V |
| Jetson Orin and expansion board | 12 V |
| Contactor coil | Separate 12 V battery, trickle-charged by the main pack |
| Arduino to VESC | Optocoupled UART to isolate the logic ground |

Vehicle notes report loosening steering collar set screws, asymmetric steering travel, and voltage sag during steering. Check mechanical play before compensating in software. The [route controller](../ros2_ws/src/control/config/pure_pursuit.yaml) uses a short 0.25 m lookahead and an estimated steering limit. Validate both on blocks before route following on the vehicle. The nominal top speed has yet to be measured.

## Arduino firmware

The [Arduino Uno sketch](../firmware/vehicle_1811/vehicle_1811.ino) runs the steering, brake, and drive outputs. To flash on the Karbon, stop the stack, disable traction power, and keep the Arduino connected to USB. Install Arduino CLI with the `arduino:avr` core and the `AltSoftSerial`, `VescUart`, and calibrated `ServoTimer2` libraries.

```bash
./g --flash-arduino
```

If several Arduinos are present, pass `port:=` with the correct path from `/dev/serial/by-id/`.

| Connection | Interface |
| --- | --- |
| Karbon commands | Hardware UART at 57600 baud |
| VESC | AltSoftSerial at 19200 baud, RX pin 8 and TX pin 9 |
| Axle steering, brake, steering wheel | ServoTimer2 on pins 10, 6, and 3 respectively |

Steering updates every 20 ms. The axle pulse is centered at 1150 µs with a span of about ±417 µs. The vehicle's local ServoTimer2 copy was modified to set `MIN_PULSE_WIDTH` to 500 µs. A stock reinstall clamps the 733 µs full-left command at 750 µs. Measure the actual left and right steering angles before tuning route following.

### Link staleness

After 250 ms without a valid Arduino command, the firmware zeros speed, steering, and braking. The vehicle can coast. After 1.5 seconds of qualifying stationary commands, all three servos can stop pulsing and the brake servo can release its grip. Park on level ground and keep a person at the kill switch during vehicle tests.

### Serial protocol

The bridge sends one JSON object per line. Speed is in mph, steering ranges from −1 to 1, and braking ranges from 0 to 1.

```json
{"speed": 2.5, "steering": -0.2, "braking": 0.0}
```

The Arduino applies all three fields from the same valid line. Telemetry is disabled because VESC polling delayed the loop and debug output interfered with the serial protocol.

## Radar

The Smartmicro driver lives in a submodule. Run `./scripts/vehicle.sh fetch-radar` after initializing it, or run `setup-machine` to fetch the vendor libraries during host setup.

The 1811-specific [radar configuration](../ros2_ws/src/obc_bringup/config/radar_1811.yaml) uses `enp1s0` at `192.168.11.17/24` and receives from `192.168.11.11` on UDP port 55555. Confirm traffic on the host first. Then, from a container with the radar driver built, run `ros2 launch obc_bringup radar.launch.py`.

## Troubleshooting

| Symptom | First check |
| --- | --- |
| Throttle stays at zero in MANUAL | Fully press and release the brake trigger, then check its rest value on `/joy` against the [gamepad mapping](gamepad_controls.md#sticks-and-trigger). |
| Commands appear but the car stays still | Run `ros2 topic info /vehicle_command --verbose`. Under `./g`, expect the mode manager as the only publisher and the serial bridge as a subscriber. Check the bridge log and `/dev/serial/by-id/`. |
| No LiDAR data | The default IP is `169.254.148.80`. Resolve `os-122316000219.local` on Karbon if the address changed. The standard bringup uses the `LEGACY` UDP profile for the documented 3.0.1 sensor firmware to avoid the missing `WINDOW` field error. |
| A launch or config edit appears stale | Rebuild and source the workspace inside the development container. For recovery from broken build outputs, use the [reload instructions](#host-setup-and-recovery). |
| Cameras or RViz fail to start | Check the [Karbon–Jetson diagnostics](camera_link_setup.md#troubleshooting). |
