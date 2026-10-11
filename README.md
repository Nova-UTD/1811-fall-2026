# 1811 Vehicle Software

ROS 2 Humble stack for the 1811 vehicle, running entirely in Docker.

> **Architecture:** 1811 runs on **two on-board computers** — the Karbon 800
> (Ouster lidar + Arduino) and the Jetson Orin (4× ZED X cameras + most
> fusion work), intended to be joined by Ethernet into one ROS 2 graph.
> **That link is built and connected but not currently in use** — see
> [Known issues](#known-issues). Read
> [`docs/compute_and_sensor_topology.md`](docs/compute_and_sensor_topology.md)
> before wiring up anything that consumes lidar and camera data together.

---

## Contents

- [1811 Vehicle Guide](#1811-vehicle-guide) — **start here** (hardware, how to drive, what's done / not done)
- [Quick reference](#quick-reference) — topics, nodes, what's built
- [Vehicle power & mechanical](#vehicle-power--mechanical) — hardware details
- [Setup](#setup) — first time, on any machine
- [Daily use](#daily-use) — enter the container, build
- [Workflows](#workflows) — copy-paste command blocks
  - [Manual teleop — gamepad](#manual-teleop--gamepad)
  - [Manual teleop — keyboard](#manual-teleop--keyboard)
  - [Lidar + odometry](#lidar--odometry)
  - [Teach and repeat](#teach-and-repeat)
- [Firmware](#firmware) — Arduino, timings, steering pipeline, flashing
- [Serial protocol](#serial-protocol-karbon--arduino)
- [Troubleshooting](#troubleshooting) — symptom → cause → fix
- [Safety](#safety) and [Known issues](#known-issues)
- [Machine-specific setup](#machine-specific-setup) — WSL, Karbon

---

## 1811 Vehicle Guide

In short: 1811 can drive by **gamepad** today, and it can **teach and repeat**.
That means you drive a loop by hand while the lidar records it, and then the car
drives the same loop itself. Everything else is still unbuilt: cameras, radar,
perception, sensor fusion, and a proper safety layer. There's also **no deadman
switch** yet, so a human at the kill switch is required whenever the car drives
itself.

### 1. What's on the vehicle

| Part | What it does |
|---|---|
| **Karbon 800** (24 V computer) | The main computer today. It runs the lidar driver, localization, control, and the link to the Arduino. |
| **Jetson Orin** (12 V computer) | Meant to run the cameras and heavy processing. **It isn't used yet.** Its Ethernet link to the Karbon is wired but not active. |
| **Ouster OS1 lidar** | 3D laser scanner, plugged into the Karbon. It's the only sensor the software uses. |
| **4× ZED X cameras** | Plugged into the Jetson. Mounted, but no software uses them. |
| **Smartmicro DRVEGRD 169 radar** | Driver vendored as submodule `smartmicro_ros2_radars`; hardware link/IP bring-up still open. |
| **Arduino Uno** | Turns computer commands into steering, brake, and motor signals. |
| **Steering servo** (Docyke S350) | Turns the wheels. Arduino pin 10. |
| **Brake servo** | Pulls the brake. Arduino pin 6. |
| **VESC** | Motor controller for the drive motor. The Arduino talks to it over serial. |

See also [`docs/compute_and_sensor_topology.md`](docs/compute_and_sensor_topology.md).

### 2. How a command becomes wheel motion

```
 You (gamepad)  ──┐
                  ├──► /vehicle_command ──► serial_bridge_node ──► USB ──► Arduino ──► steering servo
 pure_pursuit   ──┘     (steer, throttle,      (Karbon, Python)     JSON     (firmware)  brake servo
 (autonomy)              brake: -1..1)                                                  VESC → motor
        ▲
        │ /odometry  (where the car is)
 Ouster lidar ──► /ouster/points ──► KISS-ICP (localization) ──► route recorder (saves the loop)
```

1. Something publishes a command on `/vehicle_command`. During TEACH it comes
   from the gamepad, during REPEAT from pure pursuit.
2. `serial_bridge_node` is the **only** program that talks to the Arduino. It
   clamps the values to safe ranges and sends a JSON line such as
   `{"speed": 2.5, "steering": -0.2, "braking": 0.0}`.
3. The Arduino converts steering into a servo pulse (1150 µs = straight) and
   speed into a motor command.
4. If the Arduino hears nothing for **250 ms**, it zeros speed and steering and
   **coasts**. It does **not** brake.

### 3. How to operate it

#### Power on (order matters)

1. **Precharge switch ON**, then wait a few seconds.
2. **Turn the key.** Skipping the wait can weld the contactor or blow the fuse.

Details: [Vehicle power & mechanical](#vehicle-power--mechanical).

#### Start the software (on the Karbon)

3. In the `1811/` folder, run this once for each terminal you need:

```bash
./scripts/dev.sh
```

It starts the Docker container, builds the code the first time, and opens a
ready shell. Details: [Daily use](#daily-use).

#### Option A: manual driving

4. In one terminal:

```bash
ros2 launch teleop_bridge teleop_bridge.launch.py
```

Controls: left stick up/down is throttle, right stick left/right is steering,
left trigger is brake. Use the gamepad **wired** — Bluetooth changes the axis
numbers. Details: [Manual teleop — gamepad](#manual-teleop--gamepad).

#### Option B: the full stack: manual driving, then teach and repeat

4. **Terminal 1** starts everything. Leave it running until you're completely
   done:

```bash
ros2 launch obc_bringup bringup.launch.py lidar_ip:=169.254.148.80 port:=/dev/serial/by-id/$(ls /dev/serial/by-id/ | grep Arduino)
```

The car starts in **DISABLED** (brake on, sticks ignored). Everything else is on
the gamepad:

5. **A** → MANUAL: drive with the sticks.
6. **Y** → start recording. Drive the loop, stop back where you started, and
   press **Y** again to save. The route goes straight to pure pursuit.
7. **Dry run:** stay in MANUAL with the sticks centred, run
   `ros2 topic echo /cmd/auto` in a second terminal, and push the car by hand.
   Steering should stay smooth and well inside ±1.
8. **Real run:** **Start** → AUTONOMOUS, then **hold RB** to drive. Release RB,
   squeeze the brake, or press **B** to stop. Have a spotter and keep a hand on
   the kill switch.

**Don't restart terminal 1 between TEACH and REPEAT.** Restarting moves the
car's coordinate origin, and the saved route stops matching reality with no
warning. Full detail: [Teach and repeat](#teach-and-repeat).

#### Testing without the car

```bash
ros2 launch control pure_pursuit.launch.py use_sim:=true
```

This runs pure pursuit against a simulated car.

#### Power off

8. Turn the **key/contactor OFF**, and confirm it's off before walking away.

### 4. Software stacks: done vs. not done

#### Working

| Stack | Package / file | What it does |
|---|---|---|
| **Teleop** | `teleop_bridge` | Gamepad, keyboard, and the serial bridge to the Arduino |
| **Firmware** | `firmware/vehicle_1811/vehicle_1811.ino` | Steering (with smoothing), brake, motor, and a 250 ms dead-link cutoff |
| **Lidar driver** | `ouster-ros` (third-party submodule) | Publishes `/ouster/points` |
| **Localization** | `localization`, wraps `kiss-icp` (third-party submodule) | Works out where the car is from lidar alone → `/odometry` |
| **Vehicle model / TF** | `vehicle_1811_description` | URDF with the measured sensor positions |
| **Route recording** | `routing` (`route_recorder_node`) | Saves the driven loop as a CSV file |
| **Path following** | `control` (`pure_pursuit_node`, `bicycle_sim_node`) | Steers along the saved route, with a simulator for testing |
| **Messages** | `vehicle_msgs` | `VehicleCommand`, `VehicleState`, `Detection` |
| **One-command launch** | `obc_bringup` | `bringup` runs everything; the gamepad switches modes |

#### Not done

| Stack | Status | Why it matters |
|---|---|---|
| **`mode_manager` / deadman switch** | On by default in `bringup`. Manual driving tested on the car; teach and repeat tested in simulation only | Autonomy on the real car hasn't been run through it yet. |
| **`route_publisher`** | Partly: `route_recorder_node` publishes each save on `/planning/path` | Loading an *older* route file back onto the topic still needs it. |
| **Cameras** | ✅ `jetson_bringup` starts all four ZED X on the Jetson (driver in `~/zed_ws`); lidar + cameras show together in RViz on the Karbon — [how to run](docs/gamepad_controls.md#see-everything-at-once-lidar--4-cameras-in-one-rviz-window) | No team code *uses* camera data yet (perception is empty). |
| **`camera_perception`** | Empty skeleton | No object or lane detection. |
| **`lidar_perception`** | Empty skeleton | No obstacle detection. |
| **`sensor_fusion`** | Empty skeleton | Lidar and cameras aren't combined. |
| **`jetson_bringup`** | ✅ `zed_cameras.launch.py` | Cameras only; nothing else runs on the Jetson yet. |
| **Jetson ↔ Karbon link** | ✅ Set up on the car by `scripts/setup_link_*.sh`: static IPs, Fast DDS on the cable only, chrony with the Karbon as time server | [`docs/camera_link_setup.md`](docs/camera_link_setup.md) |
| **Radar** | Driver in repo; team config in `obc_bringup` | `ros2 launch obc_bringup radar.launch.py`. HW link/IP must work first. |
| **Arduino telemetry** | Disabled | The car can't report speed or state back (it corrupted the command link before). |
| **Obstacle stopping** | None | During REPEAT the car doesn't see or avoid anything. |

### 5. Known problems before trusting autonomy

These are summarized from [Known issues](#known-issues) — read that section
before an autonomous run:

1. **`lookahead_distance` is 0.25 m** in `control/config/pure_pursuit.yaml` —
   steering is about 16× too aggressive. About 1–1.5 m is sensible.
2. **The VESC's real top speed is unverified.** Pure pursuit and
   `serial_bridge_node` now both assume 12.5 mph at full throttle, so a 2 mph
   target is sent as 2 mph — but whether the VESC actually delivers that speed
   hasn't been measured.
3. **Steering angle unverified.** "±1 = ±25°" was never measured on the current
   firmware; `max_steer_angle: 0.35` is still a placeholder.
4. **Steering turns further left than right** (likely servo horn alignment).
5. **Steering collar screws work loose** — check them before blaming software.
6. **A lost link coasts instead of braking** — on a slope the car keeps rolling.
7. **`ServoTimer2` was edited locally** (min pulse 750 → 500); reinstalling the
   library silently undoes it.
8. **Ouster firmware 3.0.1** — every lidar launch needs
   `udp_profile_lidar:=LEGACY` (bringup already passes this).
9. **Camera positions in the URDF** are still rough tape-measure values.

### 6. Where to find things

| Want to... | Look at |
|---|---|
| Change how steering or brake respond | `firmware/vehicle_1811/vehicle_1811.ino`, then reflash (unplug the blue USB from the Karbon → laptop → flash → reconnect) |
| Run the car step by step, or see every gamepad control | [`docs/gamepad_controls.md`](docs/gamepad_controls.md) |
| Set up the Jetson link, or see the cameras and lidar in RViz | [`docs/camera_link_setup.md`](docs/camera_link_setup.md) |
| Change gamepad mapping | Sticks: `ros2_ws/src/teleop_bridge/teleop_bridge/gamepad_node.py`; buttons: `ros2_ws/src/mode_manager/config/mode_manager.yaml` |
| Tune path following | `ros2_ws/src/control/config/pure_pursuit.yaml` |
| Update sensor positions | `ros2_ws/src/vehicle_1811_description/urdf/vehicle_1811.urdf.xacro` |
| Change what starts together | `ros2_ws/src/obc_bringup/launch/` |
| Radar IP / NIC / model (1811) | `ros2_ws/src/obc_bringup/config/radar_1811.yaml` — launch with `ros2 launch obc_bringup radar.launch.py` |
| Understand ROS basics used here | [`docs/understanding_the_stack.md`](docs/understanding_the_stack.md) |
| See the autonomy roadmap | [`docs/teach_and_repeat_plan.md`](docs/teach_and_repeat_plan.md), [`docs/teach_and_repeat_guide.md`](docs/teach_and_repeat_guide.md) |
| Understand the two-computer plan | [`docs/compute_and_sensor_topology.md`](docs/compute_and_sensor_topology.md) |
| Troubleshoot | [Troubleshooting](#troubleshooting) |

---

## Quick reference

### Data flow

```
                gamepad_node ─┐
                              ├─> /vehicle_command ─> serial_bridge_node ─> Arduino
           pure_pursuit_node ─┘        (VehicleCommand)
                    ^
                    │ /odometry
  Ouster ─> /ouster/points ─> localization (KISS-ICP) ─┬─> route_recorder_node ─> route CSV
                                                       └─> pure_pursuit_node
```

**Only `serial_bridge_node` ever opens the serial port.** If the wheels don't
turn, it is the first thing to check — no other node can move the vehicle.

### Topics

| Topic | Type | Published by | Consumed by |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `gamepad_node`, `mode_manager_node` |
| `/vehicle_command` | `vehicle_msgs/VehicleCommand` | `mode_manager_node` under `bringup`; `gamepad_node` or `keyboard_teleop_node` under `teleop_bridge`; `pure_pursuit_node`\* | `serial_bridge_node` |
| `/cmd/manual` | `vehicle_msgs/VehicleCommand` | `gamepad_node` (remapped, under `bringup`) | `mode_manager_node` |
| `/guardian/mode` | `std_msgs/String` (latched) | `mode_manager_node` | — (for monitoring) |
| `/planning/path` | `nav_msgs/Path` (latched) | `route_recorder_node`, on each save | `pure_pursuit_node` when `path_file:=''` |
| `/vehicle_state` | `vehicle_msgs/VehicleState` | `serial_bridge_node` | — (firmware telemetry disabled) |
| `/cmd/auto` | `vehicle_msgs/VehicleCommand` | `pure_pursuit_node` (default) | `mode_manager_node` under `bringup`; otherwise nothing (dry run) |
| `/ouster/points` | `sensor_msgs/PointCloud2` | `ouster_ros` | `localization` |
| `/odometry` | `nav_msgs/Odometry` | `localization` (KISS-ICP) | `route_recorder_node`, `pure_pursuit_node` |

\* only when launched with `cmd_topic:=/vehicle_command` — see
[Teach and repeat](#teach-and-repeat).

### Packages

| Package | What it does | Status |
|---|---|---|
| `teleop_bridge` | `gamepad_node`, `keyboard_teleop_node`, `serial_bridge_node` | ✅ |
| `localization` | Lidar odometry, wraps KISS-ICP → `/odometry` | ✅ |
| `routing` | `route_recorder_node` — record a route while driving | ✅ |
| `control` | `pure_pursuit_node`, `bicycle_sim_node` | ✅ |
| `vehicle_msgs` | `VehicleCommand`, `VehicleState`, `Detection` | ✅ |
| `vehicle_1811_description` | URDF, frames (`base_link` → `os_sensor` → `os_lidar`) | ✅ |
| `obc_bringup` | One-command launches: `bringup`, `radar` | ✅ |
| `ouster-ros` | Vendored Ouster driver (git submodule) | ✅ |
| `kiss-icp` | Lidar odometry algorithm (git submodule) | ✅ |
| `smartmicro_ros2_radars` (`umrr_ros2_driver`, `umrr_ros2_msgs`) | Smartmicro radar driver (git submodule). Needs `scripts/fetch_radar_deps.sh` once after submodule init. | 🚧 in repo; HW bring-up open |
| `routing` → `route_publisher` | Saved route → live `/planning/path` | 🟡 `route_recorder_node` publishes each save; no load-from-file yet |
| `mode_manager` | Manual/auto arbitration + deadman | ✅ manual driving tested on the car; 🧪 autonomy sim-tested only |
| `lidar_perception`, `camera_perception`, `sensor_fusion` | — | ❌ empty skeletons |

Each package has its own README with the details:
[`control`](ros2_ws/src/control/README.md),
[`routing`](ros2_ws/src/routing/README.md),
[`mode_manager`](ros2_ws/src/mode_manager/README.md),
[`localization`](ros2_ws/src/localization/README.md),
[`vehicle_1811_description`](ros2_ws/src/vehicle_1811_description/README.md).

---

## Vehicle power & mechanical

### Powering on — order matters

**Do not skip the precharge step.**

1. **Precharge switch (the fuse switch) ON first.** This bleeds current into the
   motor controller's capacitor bank through a current-limiting path. Give it a
   few seconds.
2. **Then turn the key.** This closes the contactor and connects the main pack.

Closing the contactor before the caps are precharged dumps the full pack into an
empty capacitor bank. The inrush can weld the contactor contacts shut or blow the
fuse. The precharge switch exists specifically to prevent that, and **the delay
between the two steps is the entire point** — not a formality.

### Powering off

**Turn the contactor key switch OFF.** Confirm it's off before leaving the
vehicle — an energized contactor keeps the traction bus live.

### Power distribution

| Component | Supply |
|---|---|
| Jetson Orin AGX | 12 V |
| Jetson expansion board | 12 V |
| Karbon 800 | 24 V |
| Contactor coil | 12 V battery, trickle-charged from the main pack |

The 12 V contactor battery recharges from the main battery, so it needs no
separate charging under normal use.

### Sensors

- **Ouster lidar → Karbon** — currently `169.254.148.80`, hostname `os-122316000219.local`
  (status page: http://os-122316000219.local/). The `169.254.x.x` address is
  self-assigned and can change after the lidar restarts; find the current one
  with `ping -c 1 os-122316000219.local` on the Karbon.
- **4× ZED X cameras → Jetson**

### Arduino ↔ motor controller

The Arduino talks to the motor controller over **UART through an optocoupler**,
keeping logic ground isolated from the high-current motor ground. This is
deliberate — bonding them would inject traction-domain switching noise straight
into the control link.

The Arduino can also send messages back to the Karbon, but that is currently
**disabled** — see [Firmware](#firmware).

### Manual controls

The car's manual throttle and brake should be configured and functional. Verify
before relying on them as a fallback.

### Known mechanical issues

- **Steering shaft collar works loose.** The two set screws holding the motor
  attachment to the steering shaft loosen under repeated high-friction load. When
  loose, the wheel develops slack and won't return cleanly to center — this
  presented as ~2° of apparent backlash and was initially mistaken for gearbox
  lash, and a software compensation was written for it before the real cause was
  found. **Check these screws first** before chasing steering play in software.
  *Fix:* correctly sized bolts plus threadlocker.
- **Steering throw is asymmetric** — the motor travels noticeably further left
  than right, even though the pulse widths are symmetric about center
  (1150 ± 417 µs). Most likely servo horn clocking: a horn/pushrod linkage is a
  crank-slider, so throw is only symmetric when the horn sits perpendicular to
  the pushrod at neutral. **This matters for autonomy** — `pure_pursuit` has a
  single `max_steer_angle` and assumes the vehicle turns equally both ways, so it
  will systematically over-steer one direction and under-steer the other, and on
  a closed loop that error accumulates rather than cancelling. Fix mechanically
  if possible; otherwise split `STEER_SPAN_US` into separate left/right constants
  in firmware.
- **Front grill lights disconnected** — not soldered properly. Left unfixed, low
  priority.
- **Some wires hang near the ground.** Needs zip-tying. Low effort, real
  snag/abrasion risk.

---

## Setup

Everything runs inside a Docker container built from `ros:humble`. **No native
ROS 2 install is needed on any machine** — not the Karbon (Ubuntu 24.04), not a
Windows laptop (via WSL). The container brings its own Ubuntu 22.04 / Humble
userspace; the host just runs Docker.

```bash
git clone git@github.com:yourorg/1811.git
cd 1811
git submodule update --init --recursive     # ouster-ros, kiss-icp, smartmicro_ros2_radars
bash scripts/fetch_radar_deps.sh            # Smart Access libs (skipped if already present)
bash scripts/setup_machine.sh
```

`setup_machine.sh` handles host-level prerequisites (Docker itself), runs
`fetch_radar_deps.sh` if needed, and builds the image. The submodule step
fetches `ouster-ros`, `kiss-icp`, and `smartmicro_ros2_radars`. The radar
driver's Smart Access binaries are **gitignored** inside that submodule, so
every machine must run `fetch_radar_deps.sh` once (or rely on `setup_machine.sh`).

On WSL, do the [usbipd steps](#wsl-windows-laptop) before expecting any USB
device to show up.

---

## Daily use

From the repo root on the host, **once per terminal you need**:

```bash
./scripts/dev.sh
```

It starts the container if it isn't up (`docker compose up -d dev`), builds the
workspace the first time, and opens a shell in it (`docker compose exec dev bash`).

> **Prefer `exec` over `run` for extra terminals.** Each `docker compose run`
> creates a *separate container*, and separate containers are what make DDS
> participant GUIDs collide (see [Known issues](#known-issues)). One container
> with many `exec` shells has one PID space and one DDS participant pool — and
> it starts faster. `docker compose run --rm dev bash` still works for a
> throwaway one-off.

That's the whole thing — **no `cd`, no `source`.** Every shell opens already
`cd`'d into `/vehicle_1811/ros2_ws` with ROS 2 *and* the built workspace overlay
sourced, so `ros2 launch ...` works on the first line you type. The repo is
bind-mounted at `/vehicle_1811`, `/dev` is passed through, and `DISPLAY` is
forwarded.

Rebuild after code changes — there's an alias for it (this is `colcon`, not
`docker build`):

```bash
rebuild
```

That expands to
`colcon build --symlink-install --packages-skip smart_rviz_plugin && source install/setup.bash`
(Humble does not support smartmicro's RViz plugins; the radar **driver** still
builds). When you only touched one or two packages, skip the alias and select them:

```bash
colcon build --symlink-install --packages-select control routing teleop_bridge && source install/setup.bash
```

- Pure-Python edits take effect immediately with `--symlink-install` — no rebuild.
- Rebuild for C++ changes, new/removed packages, or `package.xml` / `CMakeLists.txt` edits.
- Rebuild the **Docker image** only for `Dockerfile` changes (apt/pip deps, or the
  shell setup above):
  ```bash
  docker compose build
  ```

> **Config edits not taking effect?** If a package was ever built *without*
> `--symlink-install`, `install/` holds real copies and later symlink builds
> won't replace them — you edit `src/` forever while the node reads a stale
> file. See [Troubleshooting](#a-yaml-edit-doesnt-reach-the-node).

---

## Workflows

Every command below runs **inside the container**. Each numbered terminal is its
own `docker compose exec dev bash` — see [Daily use](#daily-use) for why `exec`
rather than a fresh `run` container per terminal.

### Manual teleop — gamepad

One command starts `joy_node`, `gamepad_node`, and `serial_bridge_node`:

```bash
ros2 launch teleop_bridge teleop_bridge.launch.py
```

**The launch file's serial port default is `/dev/ttyACM0`.** `ttyACM*` numbers
are assigned in plug order and shift across replugs and reboots — that is how the
bridge once ended up writing JSON at a Linux serial console. For a stable path,
find the by-id link and pass it explicitly:

```bash
ls -l /dev/serial/by-id/
```

```bash
ros2 launch teleop_bridge teleop_bridge.launch.py port:=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_XXXX-if00
```

`serial_bridge_node` also supports `port:=auto`, which resolves the Arduino
through `/dev/serial/by-id/` by itself. That path is currently commented out in
the launch file, so it only applies if you set it explicitly or run the node
directly.

Controls — 8BitDo SN30 Pro, **wired**, measured on this pad:

| Axis | Control | Sign | Used as |
|---|---|---|---|
| `axes[0]` | left stick X | right = **negative** | — |
| `axes[1]` | left stick Y | up = **positive** | throttle |
| `axes[2]` | left trigger | rests **+1.0**, → −1.0 pressed | brake |
| `axes[3]` | right stick X | right = **negative** | steering |
| `axes[4]` | right stick Y | up = **positive** | — |

`INVERT_STEER = True` because right reads negative on `axes[3]`; `INVERT_THROTTLE
= False` because up already reads positive on `axes[1]`.

**Axis numbers depend on the pad's mode, which is selected by a button combo at
power-on — not by the cable.** Over Bluetooth this pad reported steering on
`axes[2]` and the trigger on `axes[5]`. Re-verify after any power-cycle into
another mode, a pad swap, or a repair:

```bash
ros2 topic echo /joy
```

Move one control at a time and confirm against the table above and the constants
in [`gamepad_node.py`](ros2_ws/src/teleop_bridge/teleop_bridge/gamepad_node.py).
`gamepad_node` refuses to publish (and logs why) if `/joy` arrives with fewer
axes than it reads, rather than dying mid-drive.

### Manual teleop — keyboard

**Terminal 1** — serial bridge:

```bash
ros2 launch teleop_bridge teleop_bridge.launch.py use_gamepad:=false
```

**Terminal 2** — keyboard teleop:

```bash
ros2 run teleop_bridge keyboard_teleop_node
```

Click into the **pygame window** (not the terminal) for it to receive keys.
Arrows drive, Shift brakes (overrides throttle), `+`/`-` adjust speed scale,
`q`/`Esc` quits. Requires a display — see [GUI apps](#gui-apps-pygame-window).

### Lidar + odometry

**Terminal 1** — URDF + TF tree. **Required before odometry**, not optional:

```bash
ros2 launch vehicle_1811_description description.launch.py
```

`localization.launch.py` asks KISS-ICP for `base_frame:=base_link` so `/odometry`
is the *vehicle's* pose, not the sensor's. That conversion needs the
`base_link → os_sensor → os_lidar` chain, which only exists while
`robot_state_publisher` is running. Without it the TF tree is just
`os_sensor → os_lidar/os_imu` with **no `base_link` at all**, and the odometry
you record is wrong or absent. Confirm the tree before trusting a route:

```bash
ros2 run tf2_tools view_frames
```

You want `base_footprint → base_link → os_sensor → os_lidar` plus the wheel and
camera frames. If you only see `os_sensor` and its children, this launch isn't
running.

**Terminal 2** — Ouster driver. `viz:=false` matters on the Karbon (no monitor;
`rviz2` crashes without X11). `udp_profile_lidar:=LEGACY` works around the
firmware 3.0.1 bug — see [Known issues](#known-issues):

```bash
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=169.254.148.80 viz:=false udp_profile_lidar:=LEGACY
```

**Terminal 3** — odometry:

```bash
ros2 launch localization localization.launch.py
```

Verify:

```bash
ros2 topic hz /odometry
```

### Teach and repeat

Drive a loop by hand while lidar odometry records it, then drive it back
autonomously. One launch, driven from the gamepad by `mode_manager_node`; full
detail in [`mode_manager`'s README](ros2_ws/src/mode_manager/README.md).

**Terminal 1** — the whole stack: URDF + TF tree, Ouster driver, KISS-ICP
odometry, route recorder, serial bridge, gamepad, pure pursuit, and the mode
manager. **Leave it running until REPEAT is completely done.** Restarting it
restarts odometry, which moves the `odom` frame origin — the recorded route then
silently stops matching reality, with no error:

```bash
ros2 launch obc_bringup bringup.launch.py lidar_ip:=169.254.148.80 port:=/dev/serial/by-id/$(ls /dev/serial/by-id/ | grep Arduino)
```

The car starts in DISABLED (brake on). Then, on the gamepad:

1. **A** → MANUAL. Drive to the start of the loop.
2. **Y** → start recording. Drive the loop back to your starting spot.
3. **Y** again → stop and save. The take goes to `/vehicle_1811/routes` and
   straight to pure pursuit.
4. **Dry run:** stay in MANUAL with the sticks centred, and in **terminal 2**
   run `ros2 topic echo /cmd/auto`. Push the car by hand along the route.
   Steering should move **smoothly and stay well inside ±1**, touching the
   limits only on genuinely sharp sections. Pinned at ±1, or flipping sign
   rapidly, means `lookahead_distance` is too small — see
   [Known issues](#known-issues).
5. **REPEAT for real**, only once the above looks right *and* you've read
   [Safety](#safety): **Start** → AUTONOMOUS, then **hold RB**. Release RB,
   squeeze the brake, or press **B** to stop.
6. **Another lap:** drive back to the start in MANUAL, then in terminal 2 run
   `ros2 service call /route_recorder_node/save std_srvs/srv/Trigger` to re-send
   the route. (Not **Y** — that starts a new recording.)

Only the route recorded in the current session can be repeated: a route from
an earlier session was recorded in a different odometry frame. Replaying a
saved file is `route_publisher`'s job, which isn't built.

See [`control`'s README](ros2_ws/src/control/README.md), "Bench test through
serial_bridge," for the full pre-flight checklist.

**No hardware at all?** `pure_pursuit_node` runs closed-loop against a
simulated bicycle model, using the bundled sample route:

```bash
ros2 launch control pure_pursuit.launch.py use_sim:=true
```

---

## Firmware

Source: [`firmware/vehicle_1811/vehicle_1811.ino`](firmware/vehicle_1811/vehicle_1811.ino).
Arduino Uno.

### Flashing

Disconnect the **blue USB cable** from the Karbon and plug it into your laptop.
Flash, then reconnect it to the Karbon.

### Pin and peripheral map

| Function | Pin / peripheral | Notes |
|---|---|---|
| Command link ← Karbon | hardware `Serial`, 57600 | the Uno's **only** hardware UART |
| VESC link | `AltSoftSerial`, pins **8 (RX) / 9 (TX)**, 19200 | pins fixed by the library; uses **Timer1** |
| Steering servo | pin 10, `ServoTimer2` | **Timer2** |
| Brake servo | pin 6, `ServoTimer2` | moved off pin 9, which AltSoftSerial owns |

Only one hardware UART exists, so the VESC link is bit-banged. That single fact
drives most of the timing constraints below.

### Loop structure and timing

`loop()` free-runs, but **nothing touches hardware at loop rate.** Two `millis()`
gates control everything:

| Stage | Rate | Notes |
|---|---|---|
| `readAndParseSerial()` | every iteration | may block ~10 ms waiting for a newline |
| `checkStaleness()` | every iteration | cheap |
| `updateActuators()` | **50 Hz** | steering + brake; matches the servo's latch rate |
| `updateVesc()` | **50 Hz** | `setRPM`, ~26% TX duty on the VESC link |

**Why the gates exist.** `getVescValues()` previously ran every iteration — a
~78-byte reply at 19200 baud is **40.6 ms** of wire time, which pinned the loop
to ~17 Hz while ROS published at 30 Hz. Half the commands were dropped, and the
larger per-update steering steps that resulted made current transients worse.
Separately, `setRPM` every iteration saturated the VESC link (~100% TX duty),
keeping AltSoftSerial's Timer1 ISRs firing continuously and jittering the Timer2
servo pulses by ~0.2°.

Note that `steer.write()` does **not** generate a pulse — ServoTimer2's Timer2
ISR produces the 50 Hz train continuously in the background, and `write()` only
updates the value that ISR uses. So a blocked `loop()` costs command *freshness*,
never signal integrity.

### Steering pipeline

Runs once per 20 ms tick:

```
target → deadband → slew limit → direction → backlash bias → clamp → µs
```

| Constant | Value | Purpose |
|---|---|---|
| `SERVO_UPDATE_MS` | 20 | 50 Hz — the S350's latch rate; faster writes do nothing |
| `STEER_SLEW_PER_UPDATE` | 0.08 | 2°/tick = **100 °/s**. Bounds peak current. |
| `STEER_DEADBAND` | 0.02 | stops 30 Hz jitter driving the linkage back and forth |
| `BACKLASH_MOVING_POS` / `_NEG` | **0.00** / **0.00** | disabled — the play was a loose screw, not lash |

`steerApplied` is the rate-limited integrator. The backlash bias is computed into
a local and **never written back**, so direction detection cannot react to its own
output. Direction is derived from the slew-limited signal (not the raw 30 Hz
target, which would chatter near center) and is **held while stationary**.

The slew limiter is the voltage-sag fix. A step command asks a 34 N·m actuator
for maximum acceleration, and a *reversal* asks it to brake that inertia and
re-accelerate — two near-stall current events back to back. Full lock to full
lock now takes 500 ms instead of being instantaneous. Halve
`STEER_SLEW_PER_UPDATE` if sag persists; raise it if steering feels sluggish.

### Steering calibration

```
us = 1150.00 + 416.667 × steer          (16.667 µs per degree)
```

| `steer` | pulse | angle |
|---|---|---|
| −1.0 | 733 µs | −25° \* |
| 0.0 | **1150 µs** | 0° — confirmed centered |
| +1.0 | 1567 µs | +25° \* |

\* **Unverified on this firmware.** The ±25° figure may date from an older
`105 + 90·s` build. Every normalized constant in the pipeline depends on it —
measure axle angle at `steer = ±1.0` before tuning anything.

The Docyke S350 accepts **0.5–2.5 ms @ 50 Hz**, so 733 µs is in spec.
`ServoTimer2`'s stock `MIN_PULSE_WIDTH` is **750** — *above* full left lock — so
**the local library copy has been edited to 500.** Without that edit full left
silently clamps, and **the edit does not survive a library reinstall.**

### Link staleness

No valid message for **250 ms** → `speed = 0`, `steering = 0`, `braking = 0`.

**Deliberately coast, not brake.** A dead link cuts drive and centers the
steering, but does not apply the brake. See [Safety](#safety).

### Telemetry — disabled

`readVescData()` is commented out for two reasons:

1. Its `Serial.print()` calls wrote to the **same UART that receives commands**,
   corrupting the JSON stream the host parses. This was found and fixed once,
   then reintroduced — hence the comment block guarding it now.
2. `getVescValues()` is a blocking 40 ms request/response that dominated the loop.

To re-enable safely: put it on its own ~10 Hz timer, and emit a parseable JSON
line rather than free text. `serial_bridge_node` already parses replies
defensively, so the ROS side needs no changes.

---

## Serial protocol (Karbon ↔ Arduino)

JSON, newline-terminated, **57600 baud**.

**Karbon → Arduino:**

```json
{"speed": 2.500, "steering": -0.200, "braking": 0.000}
```

- `speed` — target speed in **mph** (an actual speed, not normalized).
  `serial_bridge_node` computes it as `throttle × MAX_SPEED_MPH`.
- `steering` — `-1.0` … `1.0`
- `braking` — `0.0` … `1.0`

`serial_bridge_node` clamps `throttle`/`steer` to ±1 and `brake` to 0…1 before
writing, and logs a throttled warning naming the offending value when it has to.
It is the last thing between a bad command and the hardware, so these ranges are
enforced there rather than trusted from upstream.

The firmware commits all three fields only if they parsed from the **same line**,
so a partial parse can't pair `speed` from one message with `steering` from an
older one.

The Arduino currently sends **nothing back** — telemetry is disabled, see
[Firmware](#firmware).

---

## Troubleshooting

### The vehicle doesn't move, but `/vehicle_command` echoes fine

The graph is healthy and the break is at the serial leg. Check that anything is
actually subscribed:

```bash
ros2 topic info /vehicle_command --verbose
```

Subscriber count `0` means `serial_bridge_node` is dead or never started. Under
`ros2 launch`, a node that dies at startup prints one `process has died` line
that scrolls past while the others keep running — everything *looks* alive.
Confirm:

```bash
ros2 node list
```

`serial_bridge_node` logs the port it opened at startup, and logs
`SERIAL BRIDGE DID NOT START` (with the available devices listed) if it
couldn't. Check the host's view of the devices:

```bash
ls -l /dev/serial/by-id/
```

**Publisher count `2`** is the other failure: two nodes publishing means
`serial_bridge_node` receives them interleaved and the Arduino acts on whichever
landed last. Under `bringup` the only publisher should be `mode_manager_node` —
something else (a `teleop_bridge` launch, a remapped `pure_pursuit_node`) is
running alongside it.

### Nothing at all on `/vehicle_command` while pure pursuit is running

`pure_pursuit_node` publishes on `/cmd/auto`, never on `/vehicle_command`. Under
`bringup`, `mode_manager_node` forwards it only in AUTONOMOUS with RB held. Check
the mode in terminal 1's log (`MODE -> ...`). Standalone, `/cmd/auto` has no
subscriber at all — the intended dry-run default.

### Bench-testing a fixed steering command

Drives the topic directly with no gamepad and no pure pursuit. Use this rather
than the Arduino Serial Monitor — a hand-typed command gets zeroed 250 ms later
by the firmware's staleness timeout, and `-r 30` keeps the link alive:

```bash
ros2 topic pub /vehicle_command vehicle_msgs/msg/VehicleCommand "{steer: -1.0, throttle: 0.0, brake: 0.0}" -r 30
```

### A yaml edit doesn't reach the node

Nodes read from `install/`, not `src/`. `colcon build` **copies** config files
across; `--symlink-install` replaces those copies with symlinks so `src/` edits
take effect live. But if the package was ever built *without* the flag, the
existing copy is left in place and later symlink builds won't convert it.

Confirm what the running node actually has — this is the only ground truth:

```bash
ros2 param get /pure_pursuit_node lookahead_distance
```

Verify the install artifact is a symlink into `src/`:

```bash
ls -l install/control/share/control/config/pure_pursuit.yaml
```

To force it, clear the stale artifacts **including the egg-info in `src/`**, then
rebuild and re-source:

```bash
rm -rf build/control install/control src/control/control.egg-info && colcon build --symlink-install --packages-select control && source install/setup.bash
```

Two related traps:

- **`ros2 run control pure_pursuit_node` loads no yaml at all** — it falls back
  to `declare_parameter` defaults. Pass `--ros-args --params-file <path>`.
- **Undeclared launch arguments are silently ignored.**
  `ros2 launch control pure_pursuit.launch.py lookahead_distance:=1.0` does
  **nothing** — that launch file only declares `path_file`, `cmd_topic`, and
  `use_sim`. No error, no warning, no effect.

### `PackageNotFoundError: No package metadata was found for <pkg>`

The `.egg-info` in `src/<pkg>/` is missing or stale — the generated console-script
wrapper needs it to resolve `load_entry_point`. Use the `rm -rf` command above
(it clears the egg-info), then **re-source**, or just open a fresh
`docker compose exec dev bash`.

### Throttle is always 0 in `/vehicle_command`

The brake trigger is being read as pressed. If `TRIGGER_RESTS_AT_PLUS_ONE` in
[`gamepad_node.py`](ros2_ws/src/teleop_bridge/teleop_bridge/gamepad_node.py) is
wrong for your connection, brake sits at ~0.5 at rest and throttle is forced to
0 forever — the topic still publishes, so it looks fine. Echo `/joy`, read the
trigger axis **at rest**, and set the constant to match.

### Nodes can't see each other across containers

All containers must agree on `ROS_DOMAIN_ID` (hardcoded to `0` in
`docker-compose.yml`) and use `network_mode: host`. Check:

```bash
ros2 topic list
```

If topics intermittently fail to cross containers, see the DDS GUID entry in
[Known issues](#known-issues) — and prefer one container with `exec` shells.

### A device doesn't appear inside the container

The `/dev` bind-mount reflects the host **live** — devices that appear after the
container started are picked up automatically, no restart needed. So if it's not
there, it wasn't on the host either. Check on the host first, and on WSL re-run
`usbipd attach` (it does not survive a replug or reboot).

---

## Safety

`bringup` runs `mode_manager_node` as the only publisher on `/vehicle_command`,
and autonomy only drives while RB is held — see
[`mode_manager`'s README](ros2_ws/src/mode_manager/README.md). It is the newest
part of the stack and autonomy hasn't been run through it on the car yet, so:

- **The deadman is only as good as the button mapping.** Verify the button
  indices with `ros2 topic echo /joy` before an autonomous run. Anything that
  publishes `/vehicle_command` outside `bringup` (`teleop_bridge`, a remapped
  `pure_pursuit_node`) bypasses the mode manager entirely.
- **The firmware watchdog is enabled, but it coasts.** `checkStaleness()` fires
  after 250 ms without a valid message and sets `speed = 0`, `steering = 0`,
  `braking = 0`. So a dead link — a crashed node, a pulled cable — cuts drive and
  centers the steering. It does **not** brake. The vehicle coasts to a stop on
  its own, and on any gradient it keeps rolling.
- `pure_pursuit_node`'s internal safety net (zero throttle if the path hasn't
  loaded or `/odometry` goes stale) covers *only those two failures*, and only
  while the node is still alive. It cannot help if the process dies outright.

**A human at the kill switch is still the backstop.** Treat every REPEAT run as
"manual driving with a robot doing the steering," never as something to walk away
from. Wheels off the ground for the first run of anything new; spotter present;
start slow.

---

## Known issues

### Software / configuration

- **⚠️ `lookahead_distance` is `0.25` in the committed yaml.** Pure pursuit's
  curvature is `κ = 2·y_local / Ld²` — **quadratic** in `Ld` — so cutting `Ld`
  from 1.0 m to 0.25 m multiplies steering gain by **16×**. With
  `max_steer_angle: 0.35` and `wheelbase: 0.937`, saturation (`|steer| = 1.0`)
  happens at:

  | `Ld` | lateral offset to saturate | heading error to saturate |
  |---|---|---|
  | 1.0 m | 19.5 cm | 11.2° |
  | **0.25 m** | **1.2 cm** | **2.8°** |

  1.2 cm is far below lidar-odometry noise, and recorded routes wander ~17 cm
  laterally — **14× the threshold**. So full lock is the *normal* output at this
  setting, not a fault, and the sign flips constantly as sub-centimetre error
  crosses zero. `Ld` well under the 0.937 m wheelbase is pathological for pure
  pursuit; use roughly 1–1.5× wheelbase at low speed.
  **Confirm which value the successful run actually used and commit it to
  [`pure_pursuit.yaml`](ros2_ws/src/control/config/pure_pursuit.yaml).**
- **`max_steer_angle: 0.35` is still a `TODO-MEASURE` placeholder**, and
  `±1.0 = ±25°` was never confirmed on the current `117 + 75·s` firmware. Both
  feed the normalized constants in the firmware steering pipeline.
- **Speed limits not accurate.** The speed configuration (mph) on the VESC needs
  tuning/configuration. (The software side is reconciled:
  [`pure_pursuit.yaml`](ros2_ws/src/control/config/pure_pursuit.yaml)
  `max_speed_mps: 5.588` matches `serial_bridge_node`'s `MAX_SPEED_MPH = 12.5`.
  If you change one, change the other — throttle scaling depends on it.)
- **Voltage sag under steering load.** Sag was observed while the steering motor
  was moving, with no mechanical obstruction. Cause is current draw from
  acceleration and direction reversals, not stalling: a step command asks a
  34 N·m actuator for maximum acceleration, and a reversal demands brake-then-
  reaccelerate. Mitigated by the firmware slew limiter. Still worth checking the
  servo rail's DC-DC current rating against the S350's stall current (not
  published on the vendor page) and adding bulk capacitance near the servo.
- **DDS participant GUID collisions across containers.** Fast DDS derives a
  participant's GUID prefix from a host identifier plus the process id.
  `network_mode: host` and `ipc: host` already make the host part identical
  across containers, and without a shared PID namespace each `docker compose run`
  container numbers processes from 1 — so two containers readily produce the same
  pid, hence the same GUID prefix. Duplicate GUIDs are undefined behavior in DDS:
  discovery mis-attributes endpoints, and a reader can be matched to a writer on
  a different topic — delivering bytes that were never a `VehicleCommand`,
  i.e. arbitrary floats easily outside ±1. **Mitigated** by `pid: "host"` in
  `docker-compose.yml` and by using one container with `docker compose exec`
  shells. Unconfirmed as a root cause.
- **Ouster driver crashes on sensor firmware < 3.2.0** with `std::out_of_range` /
  `Field 'WINDOW' not found in LidarScan`. The driver's field layout for
  `RNG19_RFL8_SIG16_NIR16` always includes a `WINDOW` field that doesn't exist
  below firmware 3.2 — regardless of `point_type`, since the layout follows
  `udp_profile_lidar`. Our unit is on 3.0.1. Either upgrade the sensor firmware
  (real fix) or pass `udp_profile_lidar:=LEGACY` (workaround, costs the newer
  profile's ambient/reflectivity encoding).
- **`Failed to set desired SO_RCVBUF size`** from the Ouster driver means the
  host's UDP buffer ceiling is under 1 MB. Harmless at low rates, risks dropped
  packets under load. Fix on the **host**, not in the container:
  ```bash
  echo -e "net.core.rmem_max=1048576\nnet.core.rmem_default=1048576" | sudo tee /etc/sysctl.d/99-ouster.conf && sudo sysctl --system
  ```

### Hardware / integration

- **Radar bring-up.** Driver submodule: `ros2_ws/src/smartmicro_ros2_radars`
  (run `scripts/fetch_radar_deps.sh` after submodule init). **1811 params and
  launch live in this repo**, not in the submodule:
  `ros2 launch obc_bringup radar.launch.py` (config:
  `obc_bringup/config/radar_1811.yaml`). Before ROS: put `enp1s0` on
  `192.168.11.17/24`, confirm traffic with `tcpdump` (`udp port 55555`). Do not
  edit the vendored `radar.params.template.yaml`. A media converter has no ROS
  driver of its own. If build fails on `point_cloud_msg_wrapper`, rebuild the
  Docker image (`docker compose build`).
- **The Jetson ↔ Karbon link is not currently in use.** It has been built and
  connected, but is not active, so the two computers are not sharing one ROS
  graph today.
- **`ServoTimer2`'s `MIN_PULSE_WIDTH` has been edited locally** from 750 to 500,
  because full left lock is 733 µs. This is not tracked by version control and
  **will be lost on a library reinstall** — full left would then silently clamp.
- Mechanical issues are listed under
  [Known mechanical issues](#known-mechanical-issues).

### Historical — fixed, kept for context

- The firmware's `Serial.print()` debug output collided with the command channel
  twice. It is currently disabled along with `readVescData()`; see
  [Firmware](#firmware).
- Earlier firmware drained only one serial line per `loop()`. If commands start
  lagging or backing up, check the drain-all-buffered-lines fix is still there.
- `pure_pursuit_node` used to report "Goal reached" on the first control tick of
  any route ending near its start, because the check was pure distance to the
  final waypoint. Fixed by gating on path progress (`is_near_path_end`).

---

## Machine-specific setup

### GUI apps (pygame window)

- **Karbon:** works via the X11 `DISPLAY` + `/tmp/.X11-unix` bind already in
  `docker-compose.yml`, once the desktop lets the container's `root` user draw
  on it. Run this once per boot in a terminal on the Karbon's own screen (not in
  Docker), or RViz fails with `Authorization required` /
  `could not connect to display`:
  ```bash
  xhost +SI:localuser:root
  ```
- **WSL:** requires WSLg. Confirm `echo $DISPLAY` is non-empty in a **plain WSL
  shell** first — the container inherits whatever `$DISPLAY` the host shell had
  when the container was started. If it's empty there, it's empty inside.

### WSL (Windows laptop)

USB devices plugged into Windows aren't visible to WSL — or to Docker Desktop,
which runs on top of WSL — by default. Docker Desktop's WSL2 backend only sees
what WSL itself sees; it does not talk to Windows USB directly.

**Windows PowerShell (as Administrator):**

```powershell
winget install usbipd
usbipd list
usbipd bind --busid <busid>
usbipd attach --wsl --busid <busid>
```

Re-run `attach` after **every** unplug/replug or reboot.

**In a plain WSL shell**, confirm before entering the container:

```bash
ls -l /dev/serial/by-id/ /dev/input/js*
```

Also enable **Docker Desktop → Settings → Resources → WSL Integration** for your
distro, then **Apply & Restart**.

### docker-compose services

- **`dev`** — generic development container. Used everywhere.
- **`obc`** — same image and setup, intended for the Karbon as the actual
  on-board computer. Exists as its own service so Karbon-specific overrides
  (fixed serial path, autostart) have somewhere to live without touching `dev`.

`privileged: true` gives full device access, which is why you don't need to fuss
with `dialout` group permissions inside the container. `pid: "host"` shares the
host PID namespace so DDS participant GUIDs stay distinct across containers.
