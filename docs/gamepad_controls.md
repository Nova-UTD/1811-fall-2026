 # Running the car and gamepad controls

## How to run the car
Two kinds of prompt:

- `nova@karby:~$` is the Karbon itself. Run `git` commands here.
- `root@karby:/vehicle_1811/ros2_ws#` is inside the Docker container. Run
  `ros2` and `colcon` commands here. `./scripts/dev.sh` gets you in.

### 1. Update and build (after any `git pull`)

On the Karbon, outside the container:

```bash
cd ~/1811-fall-2026
git status                              # should say "working tree clean" -- if not, ask before continuing
git pull
git submodule update --init --recursive
./scripts/dev.sh                        # now inside the container
```

Inside the container:

```bash
colcon build --symlink-install --packages-ignore umrr_ros2_driver umrr_ros2_msgs smart_rviz_plugin
source install/setup.bash
```

It should end with `Summary: 15 packages finished` and no `failed` line. The
three ignored packages are the radar driver and its RViz plugin, which aren't
needed to drive (the plugin can't build on this ROS version at all). Use this
command instead of `rebuild`, which fails on them.

The first build after a fresh clone takes several minutes. Later builds only
recompile what changed.

### 2. Power on

1. **First test of anything new: wheels off the ground.**
2. Plug the gamepad into the Karbon with a **USB cable**.
3. **Precharge switch ON**, wait a few seconds, then **turn the key**. Skipping
   the wait can weld the contactor or blow the fuse.
4. Someone at the kill switch.

### 3. Terminal 1: start everything (leave it running)

```bash
cd ~/1811-fall-2026
./scripts/dev.sh
ros2 launch obc_bringup bringup.launch.py lidar_ip:=169.254.148.80 port:=/dev/serial/by-id/$(ls /dev/serial/by-id/ | grep Arduino)
```

- `169.254.148.80` is the Ouster lidar (hostname `os-122316000219.local`, status page
  at http://os-122316000219.local/). It's a self-assigned `169.254.x.x` address,
  so it can change after the lidar restarts. If terminal 1 shows `Error
  connecting to sensor`, find the current address on the Karbon (outside the
  container) with `ping -c 1 os-122316000219.local` and use the IP it prints.
- To drive manually without the lidar, use `lidar_ip:=0.0.0.0`: the lidar
  driver prints connection errors, but everything else works. Recording a route
  needs the real IP.
- The `port:=...` part finds the Arduino by itself. Paste it as-is rather than
  typing the Arduino's name, which is easy to get wrong (it contains double
  underscores).

Check the output for these lines before driving:

| Line | Meaning |
|---|---|
| `Opening serial port /dev/serial/by-id/usb-Arduino...` | Arduino found |
| `Opened joystick: Xbox One Controller` | Gamepad found |
| `MODE -> DISABLED (startup)` | Mode manager running; brake on |

If you see `SERIAL BRIDGE DID NOT START`, the car can't move: check the Arduino
USB cable and that the car is powered, then Ctrl-C and run the launch again.

**Don't close or restart terminal 1 between recording a route and repeating it.**
A restart resets the lidar's coordinate origin, and the saved route stops
matching where the car is.

### 4. Terminal 2: checks and monitoring

Open a second terminal on the Karbon:

```bash
cd ~/1811-fall-2026
./scripts/dev.sh
```

Useful commands here (Ctrl-C stops each one):

| Command | What it's for |
|---|---|
| `ros2 topic echo /joy` | Check the button and stick numbers (see [the mapping](#checking-and-changing-the-mapping)) |
| `ros2 topic echo /vehicle_command` | See exactly what's being sent to the Arduino |
| `ros2 topic hz /odometry` | Lidar positioning is working (steady rate, roughly 10 Hz) — needed before recording a route |
| `ros2 topic echo /cmd/auto` | Dry run: what pure pursuit would do, while you push the car by hand |
| `ros2 service call /route_recorder_node/save std_srvs/srv/Trigger` | Re-send the saved route for another lap |

### 5. Drive

The car starts in **DISABLED** (brake on, sticks ignored).

- **Manual:** press **A**, then drive with the sticks. **B** stops.
- **Teach and repeat:** see [Teach and repeat with the buttons](#teach-and-repeat-with-the-buttons)
  below.

### 6. Shut down

1. Press **B** (brake on).
2. Terminal 2: **Ctrl-C**, then `exit`.
3. Terminal 1: **Ctrl-C** once and wait for the nodes to stop (Ctrl-C again if
   it hangs), then `exit`.
4. Turn the **contactor key OFF** and confirm it's off before leaving the car.

The Docker container keeps running after `exit`; `./scripts/dev.sh` reconnects
to it next time.

## Quick reference

With `bringup.launch.py` (the default way to run the car):

| Control | What it does |
|---|---|
| **A** | MANUAL: the sticks drive the car |
| **B** | DISABLED: stop, brake on, sticks ignored |
| **Y** | Start recording a route; press again to stop and save it |
| **Start** | AUTONOMOUS: the car follows the saved route while RB is held |
| **RB** (hold) | Deadman: AUTONOMOUS only drives while this is held |
| **Left stick** up / down | Throttle forward / reverse |
| **Right stick** left / right | Steer left / right |
| **Left trigger** | Brake. In AUTONOMOUS, squeezing it takes over (switches to MANUAL) |

The car **starts in DISABLED**. Nothing moves until you press **A**.

## Modes

`mode_manager_node` decides what reaches the Arduino. Terminal 1 logs every
change as `MODE -> ...`.

| Mode | What drives the car | Brake | How you get there |
|---|---|---|---|
| **DISABLED** | Nothing. Sticks are ignored | On | **B**, and at startup |
| **MANUAL** | Your sticks and trigger | Only when you squeeze the trigger | **A**, or the brake trigger while AUTONOMOUS |
| **AUTONOMOUS** | Pure pursuit, **only while RB is held** | On whenever RB is released | **Start** |

## Buttons

| Button | Index in `/joy` `buttons[]` | Press | Notes |
|---|---|---|---|
| **A** | 0 | → MANUAL | Centre the sticks first: they take effect immediately |
| **B** | 1 | → DISABLED (brake on) | Works from any mode. If several mode buttons are pressed at once, B wins |
| **Y** | 3 | Start recording / stop and save | Saves to `/vehicle_1811/routes` and sends the route to pure pursuit. Ignored in AUTONOMOUS. Pressing it again later starts a **new** recording and clears the old one |
| **RB** | 5 | **Hold** to let AUTONOMOUS drive | Release = brake. The mode stays AUTONOMOUS, so holding RB again resumes |
| **Start** | 7 | → AUTONOMOUS | **Refused** (warning in terminal 1) if pure pursuit isn't running, the gamepad isn't publishing, or the brake trigger is pressed |

Buttons act once per press: holding A or Start doesn't repeat it. RB is the
only button that matters while held.

## Sticks and trigger

| Control | Index in `/joy` `axes[]` | Raw reading | Becomes |
|---|---|---|---|
| **Left stick**, up/down | 1 | up = **positive** | Throttle. Up = forward, down = reverse (negative speed to the motor controller; reverse hasn't been confirmed on the car) |
| **Right stick**, left/right | 3 | right = **negative** | Steering. Inverted in software so right stick = steer right |
| **Left trigger** | 2 | rests at **+1.0**, → −1.0 fully pressed | Brake, 0 (released) to 1 (fully pressed) |
| Left stick, left/right | 0 | right = negative | Not used |
| Right stick, up/down | 4 | up = positive | Not used |

- **Deadzone:** stick movements under 5% (0.05) count as zero.
- **Brake overrides throttle:** while the trigger is squeezed, throttle is
  forced to 0.
- **Speed scale:** full stick sends a 12.5 mph target (`MAX_SPEED_MPH` in
  `serial_bridge_node.py`). Small stick movements ask for very low speeds,
  which the motor may not turn at. Push further to get the wheels moving.
- Sticks and trigger only do anything in **MANUAL**. In AUTONOMOUS, the only
  stick-side control is the trigger takeover.

## Safety behaviour

| What happens | Result |
|---|---|
| RB released in AUTONOMOUS | Brake, wheels stop |
| Brake trigger squeezed in AUTONOMOUS | Switch to MANUAL; your brake reaches the car immediately |
| Gamepad unplugged or stops sending for 0.5 s | Brake, in any mode |
| Pure pursuit stops sending for 0.5 s in AUTONOMOUS | Brake |
| The whole stack dies (no messages for 250 ms) | The Arduino zeros speed and steering and **coasts**. It does not brake |

The kill switch is always the final backstop.

## Teach and repeat with the buttons

1. **A** → MANUAL. Drive to the start of the loop.
2. **Y** → start recording. Drive the loop and stop back where you started.
3. **Y** → stop and save.
4. **Dry run:** stay in MANUAL with the sticks centred, run
   `ros2 topic echo /cmd/auto` in a second terminal, and push the car by hand.
5. **Start**, then **hold RB** → the car drives the loop.
6. Release **RB**, squeeze the brake, or press **B** to stop.
7. **Another lap:** drive back to the start, run
   `ros2 service call /route_recorder_node/save std_srvs/srv/Trigger`, then
   **Start** + hold **RB**. Don't press Y for this: it starts a new recording.

Details: [`mode_manager`'s README](../ros2_ws/src/mode_manager/README.md).

## Driving without the mode manager

`ros2 launch teleop_bridge teleop_bridge.launch.py` (for bench tests without the
lidar) has **no modes and no buttons**: the sticks and trigger drive the car the
moment it starts, and B, Y, Start and RB do nothing. Never run it at the same
time as `bringup`.

## Checking and changing the mapping

The axes were measured on this pad. Of the buttons, A has been confirmed on the
car; the rest follow the standard Xbox layout the pad reports. Check them before
an autonomous run:

```bash
ros2 topic echo /joy
```

Press one control at a time and watch which index changes.

- **Buttons:** edit
  [`ros2_ws/src/mode_manager/config/mode_manager.yaml`](../ros2_ws/src/mode_manager/config/mode_manager.yaml),
  then restart bringup.
- **Sticks and trigger:** edit the constants at the top of
  [`gamepad_node.py`](../ros2_ws/src/teleop_bridge/teleop_bridge/gamepad_node.py)
  (`AXIS_*`, `INVERT_STEER`, `TRIGGER_RESTS_AT_PLUS_ONE`).

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Sticks do nothing after startup | The car starts in DISABLED | Press **A** |
| Throttle stays at 0 even in MANUAL | Trigger read as half-pressed (brake ≈ 0.5 at rest) | Squeeze the left trigger fully once and let go. If it persists, see [README troubleshooting](../README.md#troubleshooting) |
| Pressing Start does nothing | AUTONOMOUS was refused | Read the warning in terminal 1: no route loaded, brake pressed, or gamepad not publishing |
| Buttons do the wrong things | Pad is in a different mode, or on Bluetooth | Use the USB cable, then check `ros2 topic echo /joy` against the tables above |
| Steering goes the wrong way | `INVERT_STEER` doesn't match the pad | Flip `INVERT_STEER` in `gamepad_node.py` |
