# Teach-and-Repeat — Step-by-Step Build Guide

The execution companion to [`teach_and_repeat_plan.md`](teach_and_repeat_plan.md).
That doc explains *why* and *how the pieces work* (KISS-ICP internals, drift,
bags, the fork comparison). **This doc is the how-to:** the concrete packages,
commands, configs, and done-when checks to build the milestone in order.

**Goal:** drive a route once by hand while lidar odometry records it, then drive
it back autonomously. Lidar-only, single session.

---

## 0. Architecture — everything on the Karbon

**Decision: the entire real-time loop runs on the Karbon.** The lidar and the
Arduino are both on the Karbon, so keeping sensing → estimation → control →
actuation local means **no network in the safety-critical control loop**. The
Jetson is used only for **monitoring** (RViz over the cable) and, later, camera
work — never in the drive loop.

```
KARBON (everything real-time)
┌────────────────────────────────────────────────────────────────────────────┐
│  Ouster ─▶ ouster-ros ─▶ /ouster/points ─▶ KISS-ICP ─▶ /odometry + TF        │
│                                                          │                    │
│   TEACH:  route_recorder  ◀──────────────────────────────┤                   │
│           (writes route file)                            │                    │
│   REPEAT: route_publisher ─▶ /planning/path ─▶ pure_pursuit ─▶ /cmd/auto      │
│                                                          ▲                    │
│   gamepad ─▶ joy_node ─▶ /joy ─▶ gamepad_node ─▶ /cmd/manual                  │
│                              │                                                │
│                              └─▶ mode_manager ─(selects)─▶ /vehicle_command   │
│                                       ▲                          │            │
│                                  reads mode + deadman            ▼            │
│                                  from /joy buttons        serial_bridge ─▶ Arduino
└────────────────────────────────────────────────────────────────────────────┘

JETSON (monitoring only, over the cable)
┌──────────────────────────────┐
│  RViz  (watch map/route/pose) │   ← does NOT participate in control
└──────────────────────────────┘
```

You already have: `ouster-ros`, `joy_node`, `gamepad_node`, `serial_bridge`.
New work: **KISS-ICP (configure), route_recorder, route_publisher, pure_pursuit,
mode_manager, eval harness.**

---

## 1. Prerequisites — verify these before starting

Run all vehicle work in the container on the Karbon (`./scripts/dev.sh`).

- [ ] **Ouster driver publishes** — `ros2 topic hz /ouster/points` shows ~10–20 Hz.
      (Use the `udp_profile_lidar:=LEGACY` workaround if still on firmware 3.0.1.)
- [ ] **URDF TF is up** — `ros2 run tf2_tools view_frames` shows
      `base_link → os_sensor → os_lidar`. KISS-ICP needs this to report the
      *vehicle's* pose, not the sensor's.
- [ ] **Serial control works** — gamepad → `/vehicle_command` → Arduino moves
      (wheels off the ground).
- [ ] **Watchdog confirmed** — firmware brakes on stale serial input.
- [ ] **Karbon CPU headroom** — run `htop` while the Ouster driver streams;
      confirm there's room for KISS-ICP (it's light, but check).

If any fail, fix them first — every stage below assumes these hold.

---

## 2. Repo layout — packages to create

All under `ros2_ws/src/`. Suggested names (functional, matching
`camera_perception` / `sensor_fusion` style — rename if you prefer the
`vehicle_1811_*` convention):

```
ros2_ws/src/
  kiss-icp/            # cloned PRBonn upstream (provides the `kiss_icp` ROS node)
  localization/        # your launch + config wrapping KISS-ICP
  routing/             # route_recorder + route_publisher
  control/             # pure_pursuit
  guardian/            # mode_manager (or fold into teleop_bridge)
  eval/                # drift/eval scripts (pure Python)
  teleop_bridge/       # existing — gamepad + serial bridge
  vehicle_1811_description/  # existing — URDF/TF
  vehicle_msgs/        # existing
  ouster-ros/          # existing submodule
```

Create a ROS 2 Python package skeleton with:
```bash
cd /vehicle_1811/ros2_ws/src
ros2 pkg create --build-type ament_python routing --dependencies rclpy nav_msgs geometry_msgs vehicle_msgs
ros2 pkg create --build-type ament_python control --dependencies rclpy nav_msgs geometry_msgs vehicle_msgs tf_transformations
ros2 pkg create --build-type ament_python guardian --dependencies rclpy sensor_msgs vehicle_msgs
ros2 pkg create --build-type ament_python eval --dependencies rclpy nav_msgs
```

Build only what you need (don't `colcon build` everything):
```bash
cd /vehicle_1811/ros2_ws
colcon build --packages-select kiss_icp localization routing control guardian eval
source install/setup.bash
```

---

## 3. Interface contract — topics, frames, units

Adopt Nova's topic names where they exist, so work is portable.

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/ouster/points` | `sensor_msgs/PointCloud2` | ouster-ros → KISS-ICP |
| `/odometry` | `nav_msgs/Odometry` | KISS-ICP → routing, control |
| `/planning/path` | `nav_msgs/Path` | route_publisher → pure_pursuit |
| `/cmd/manual` | `vehicle_msgs/VehicleCommand` | gamepad_node → mode_manager |
| `/cmd/auto` | `vehicle_msgs/VehicleCommand` | pure_pursuit → mode_manager |
| `/vehicle_command` | `vehicle_msgs/VehicleCommand` | mode_manager → serial_bridge |
| `/guardian/mode` | `std_msgs/String` (or custom) | mode_manager (for logging/RViz) |

**Frames:** `odom → base_link` (KISS-ICP), `base_link → os_sensor → os_lidar`
(URDF + driver). One odometry source only — nothing else may publish `odom`.

**Units:** work in **SI internally** (m/s, radians) inside pure_pursuit, and
convert to the normalized `VehicleCommand` (`throttle`/`steer` −1..1, `brake`
0..1) *only* at the boundary. `serial_bridge` already turns `throttle` into mph.
The steering conversion needs `MAX_TURN_ANGLE` (measure off the chassis):
`steer = steering_angle_rad / MAX_TURN_ANGLE`, clamped to ±1.
*(Optional cleaner refactor for later: fork `VehicleCommand` to honest SI fields
`target_speed_mps` / `steering_angle_rad` and convert only in the serial bridge.
Skip for the week.)*

---

## Stage B1 — KISS-ICP live → `/odometry`  *(day 1)*

**Goal:** a live, correct pose stream from the lidar.

1. **Vendor upstream KISS-ICP** (PRBonn, *not* Nova's fork — you want odometry,
   not relocalization):
   ```bash
   cd /vehicle_1811/ros2_ws/src
   git clone https://github.com/PRBonn/kiss-icp.git
   # (or add as a submodule / .repos entry to pin the commit, like ouster-ros)
   ```

2. **Build it:**
   ```bash
   cd /vehicle_1811/ros2_ws
   colcon build --packages-select kiss_icp
   source install/setup.bash
   ```

3. **Launch against the live lidar.** KISS-ICP ships a ROS 2 launch; the key
   args are the input topic and the frames (check exact param names for the
   commit you cloned):
   ```bash
   ros2 launch kiss_icp odometry.launch.py \
       topic:=/ouster/points \
       base_frame:=base_link \
       odom_frame:=odom \
       publish_odom_tf:=true \
       visualize:=false
   ```
   Then remap/confirm its odometry output is on `/odometry` (add a remap in your
   `localization` launch wrapper so the whole stack shares one name).

4. **Put the launch + config in `localization/`** so it's reproducible — a
   `localization.launch.py` that includes the KISS-ICP launch with your args, and
   a `kiss_icp.yaml` for `max_range`, `min_range`, `deskew`, voxel size, etc.

**Done when:** in RViz (fixed frame `odom`), driving under teleop keeps the
accumulated point cloud **crisp** (walls stay thin lines, not smeared), and
`base_link` moves correctly through the map. Smearing = bad extrinsics or config.

---

## Stage B2 — Bags + eval harness (the floor)  *(day 1–2)*

**Goal:** a *number* for drift. This is your guaranteed deliverable.

1. **Record bags** while driving under teleop — include at least one **closed
   loop** (return to the exact start spot):
   ```bash
   ros2 bag record -o loop1 /ouster/points /odometry /tf /tf_static /vehicle_command
   ```
   Record 3–5 runs of your test area.

2. **Write the eval script** in `eval/` (~250 lines Python, runs on a bag):
   - **Closed-loop drift:** read `/odometry` at the start and at the end (when
     physically back on the start mark); report translation gap (m) and heading
     gap (deg).
   - **Map self-overlay:** accumulate `/ouster/points` transformed by `/odometry`;
     overlay the start-area cloud vs. the end-area cloud; misalignment = drift.
   - **Velocity continuity:** differentiate `/odometry` position; plot speed —
     sudden jumps = KISS-ICP losing tracking.

3. **Run it on each bag**, save the plots.

**Done when:** you can state, e.g., *"0.4 m / 3° drift over a 30 m closed loop."*

---

## 🚦 Gate  *(end of day 2)*

Is drift acceptable for your route length? **Yes →** continue to A. **No** (bare,
featureless lot starving the geometry) **→** ship B as the deliverable, and either
scope autonomous drives to short distances where drift stays small, or evaluate
FAST-LIO2 as a drop-in odometry swap (see the plan doc's upgrade axes). Mitigate
by testing near vertical structure — buildings, curbs, poles, parked cars.

---

## Stage A1 — Record & replay the route  *(day 3)*

**Goal:** capture the driven path and play it back as a `nav_msgs/Path`.

**`routing/route_recorder`** (node):
- Subscribe `/odometry`.
- When mode is TEACH (see A3), append `(stamp, x, y, yaw)` to a file, downsampled
  to ~every 0.1–0.2 m of travel (skip near-duplicate poses).
- Write on shutdown / on a "save" trigger. Format: CSV or a small YAML.

**`routing/route_publisher`** (node):
- Load the route file, build a `nav_msgs/Path` in the `odom` frame.
- Publish it **latched** on `/planning/path`.

**Done when:** drive a route, then `route_publisher` shows the exact path in RViz
overlaid on where you drove.

---

## Stage A2 — Pure pursuit controller  *(day 4)*

**Goal:** follow `/planning/path` given live `/odometry`.

**`control/pure_pursuit`** (node), each cycle (~10–20 Hz):
1. Get current pose from `/odometry`.
2. Find the **lookahead point**: the point on the path a fixed distance `Ld`
   ahead of the car (start `Ld` ≈ 1.0 m at low speed; tune).
3. Transform it into `base_link`; let `α` = angle to it.
4. Curvature `κ = 2·sin(α) / Ld`; steering `δ = atan(wheel_base · κ)`
   (`wheel_base` from the URDF).
5. Convert to the normalized command:
   `steer = clamp(δ / MAX_TURN_ANGLE, -1, 1)`;
   `throttle = target_speed_mph / MAX_SPEED_MPH` (constant low speed to start,
   e.g. 2 mph; optionally slow on high curvature).
6. Publish `VehicleCommand` on `/cmd/auto`.
7. **End condition:** when within a threshold of the last path point, publish
   zero throttle + brake.

**The calibration that bites:** getting the steering **sign** and **scale**
right. Test on blocks first — push the car along the path by hand and watch the
wheels steer *toward* it. Too-small `Ld` → snaking; too-large → cutting corners.

**Done when:** on blocks, the wheels steer correctly toward the path as you move
the car along it.

---

## Stage A3 — Mode manager, integration, drive  *(day 5–6)*

**Goal:** safe switching between manual (teach) and autonomous (repeat).

**`guardian/mode_manager`** (node):
- Subscribe `/joy` (for mode + deadman buttons), `/cmd/manual`, `/cmd/auto`.
- Maintain a mode: **DISABLED / MANUAL / AUTONOMOUS** (toggle on gamepad buttons).
- Publish the selected command to `/vehicle_command`:
  - DISABLED → zero throttle, `brake = 1.0`.
  - MANUAL → pass `/cmd/manual` (gamepad).
  - AUTONOMOUS → pass `/cmd/auto` (pure pursuit) **only while the deadman button
    is held**; release → brake.
- Publish the current mode on `/guardian/mode`.

**Wire the graph** (remap `gamepad_node` output to `/cmd/manual`):
```bash
ros2 run teleop_bridge gamepad_node --ros-args -r /vehicle_command:=/cmd/manual
```
Build a `bringup` launch that starts: ouster-ros, KISS-ICP (localization),
route_publisher, pure_pursuit, mode_manager, serial_bridge, joy_node, gamepad_node.

**The run:**
1. Place the car on a marked start spot; mode DISABLED.
2. MANUAL: drive a **loop** back to the start (route_recorder in TEACH captures it).
3. Save the route; load it in route_publisher.
4. **Without restarting KISS-ICP** (same session = same frame), switch to
   AUTONOMOUS, hold the deadman → the car drives the loop.

**Done when:** teach a loop, flip to autonomous, the car drives it.

---

## Safety checklist (every test)

- [ ] Wheels **off the ground** for the first run of any new code.
- [ ] Spotter present; hand on the **kill switch**.
- [ ] Watchdog active (firmware brakes on stale serial).
- [ ] Deadman required for AUTONOMOUS — release = brake.
- [ ] Start speed low (≈ 2 mph); raise only after clean runs.
- [ ] Known, clear area; no bystanders in the path.

---

## Demo & write-up  *(day 7)*

- Video of the autonomous loop.
- README with the eval plots and the **drift number**.
- Write the failure modes down (where KISS-ICP degraded, how you gated on it) —
  this is the highest-signal part for a résumé.

---

## Team split (after bags exist, day 2)

- **Localization + eval** (B1/B2) — front-loaded; unblocks the gate.
- **Routing + control** (A1/A2) — develops against **recorded bags**, off-vehicle.
- **Integration + safety** (A3) — owns the mode manager, deadman, watchdog, and
  test-drive choreography.

Bags are the shared substrate: one recorded loop lets control development run in
parallel with everything else.

---

## Risks & fallbacks

| Risk | Mitigation / fallback |
|---|---|
| KISS-ICP drift in a featureless lot | Measure day 2; test near structure; FAST-LIO2 swap if needed |
| Pure-pursuit steering sign/scale wrong | Calibrate on blocks first; budget a half-day |
| Karbon CPU can't keep up | Lower Ouster resolution/rate; last resort, run KISS-ICP on Jetson over the cable |
| Route meaningless on replay | Never restart KISS-ICP between teach and repeat (same session) |
| Link/among-node hiccup mid-drive | Watchdog brakes; control loop is Karbon-local so the cable isn't in it |

---

## Command cheat-sheet

```bash
# enter container (Karbon), once per terminal -- builds the workspace on first use
./scripts/dev.sh

# rebuild after code changes
rebuild

# full stack (URDF, lidar, odometry, recorder, serial bridge, gamepad, pure
# pursuit, mode_manager) -- then teach / repeat from the gamepad:
# A = MANUAL, Y = record / save, Start + hold RB = repeat, B = stop
ros2 launch obc_bringup bringup.launch.py lidar_ip:=<sensor-ip> port:=/dev/serial/by-id/$(ls /dev/serial/by-id/ | grep Arduino)

# record a loop
ros2 bag record -o loop1 /ouster/points /odometry /tf /tf_static /vehicle_command

# inspect
ros2 topic hz /ouster/points
ros2 topic echo /odometry
ros2 run tf2_tools view_frames
```
