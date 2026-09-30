# control

`pure_pursuit_node`: follows a path against live odometry and publishes
steering + speed. This is **step A2** of the teach-and-repeat milestone — see
[`docs/teach_and_repeat_guide.md`](../../../docs/teach_and_repeat_guide.md#stage-a2--pure-pursuit-controller--day-4).

Also ships `bicycle_sim_node`, a closed-loop kinematic-bicycle simulator so
pure_pursuit_node can be exercised end-to-end with no vehicle, no KISS-ICP,
and no route_publisher.

## Contract

| | |
|---|---|
| **Subscribes** | `/odometry` (`nav_msgs/Odometry`) — live pose |
| **Subscribes** | `/planning/path` (`nav_msgs/Path`, latched/TRANSIENT_LOCAL) — *or* loads `path_file` (CSV) once at startup instead |
| **Publishes** | `/cmd/auto` (`vehicle_msgs/VehicleCommand`) at `control_rate` Hz |
| **Does not publish** | `/vehicle_command` directly — `mode_manager` (not yet built) gates `/cmd/auto` with a deadman switch before it reaches `serial_bridge` |

## Build + test

```bash
./scripts/dev.sh     # from the repo root; lands in ros2_ws with ROS sourced
colcon build --symlink-install --packages-select control && source install/setup.bash
colcon test --packages-select control --event-handlers console_direct+
```

`test/test_pure_pursuit_core.py` unit-tests the geometry (closest-index
search, lookahead interpolation, curvature/steering, goal detection) against
synthetic straight and curved paths — no ROS, no hardware, no odometry
source required. It's plain pytest, so it also runs directly:
```bash
pytest ros2_ws/src/control/test/test_pure_pursuit_core.py
```

## Run standalone (no hardware, no other nodes)

```bash
ros2 launch control pure_pursuit.launch.py use_sim:=true
```

This loads `config/sample_loop.csv` (a rounded-rectangle bench path) and
launches `bicycle_sim_node` alongside it, closing the loop entirely in
software: pure_pursuit_node's `/cmd/auto` drives the simulated bicycle model,
whose `/odometry` feeds back into pure_pursuit_node. Watch it converge:

```bash
ros2 topic echo /odometry     # pose should settle onto the path and loop it
ros2 topic echo /cmd/auto     # throttle/steer being computed
```

Swap in your own path before hardware exists:
```bash
ros2 launch control pure_pursuit.launch.py use_sim:=true path_file:=/path/to/your.csv
```
CSV format: `x,y` or `x,y,yaw` per row (meters, radians), optional header row,
`#` comments and blank lines ignored. If `yaw` is omitted it's inferred from
bearing to the next waypoint.

Once `route_publisher` exists, run without `path_file` (or override it to
`''`) to subscribe `/planning/path` instead — no code changes needed.

## Bench test through serial_bridge (real Arduino, no mode_manager)

`serial_bridge_node` already subscribes `vehicle_msgs/VehicleCommand` and
writes the `{"speed", "steering", "braking"}` JSON line to the Arduino —
that's the same code path `gamepad_node` drives today. Pointing
pure_pursuit_node's output at that topic instead of `/cmd/auto` reuses it
directly, no new code:

```bash
ros2 launch control pure_pursuit.launch.py cmd_topic:=/vehicle_command \
    path_file:=/path/to/your.csv
```

>>> **This bypasses `mode_manager`'s deadman switch entirely** (it isn't
built yet) — **and the repo root [README](../../../README.md#known-issues--things-to-watch)
confirms the Arduino has no working watchdog either**: `checkStaleness()`
exists in the firmware but isn't enabled, so if the serial link goes stale
for *any* reason (this node crashes, `serial_bridge_node` dies, a network
hiccup), the firmware keeps executing the last command it received
**indefinitely — it does not brake on its own**. There is currently no
automatic backstop of any kind between "something goes wrong" and the car
stopping. A human at the kill switch is the only thing standing in for both
the missing deadman and the missing watchdog. Before running this against
real hardware:
- [ ] Wheels **off the ground** for the first run.
- [ ] Spotter present; hand on the **kill switch**, watching the whole time
      — not just at the start.
- [ ] `serial_bridge_node` must be the *only* thing opening the port — don't
      also run `gamepad_node` remapped to `/vehicle_command` at the same
      time, and don't run this alongside a second serial writer.
- [ ] Low `target_speed_mps` (the config default, ~2 mph) until you trust it.

This is a deliberately temporary shortcut for bench/blocks testing before
`mode_manager` exists — not something to leave wired for an unattended or
routine drive. See the [risks table](../../../docs/teach_and_repeat_guide.md#risks--fallbacks)
in the guide for why the deadman exists.

## Calibration TODOs before this touches the real vehicle

These are placeholders, not measurements. `config/pure_pursuit.yaml` flags
each with a comment; don't trust the defaults on hardware:

- **`max_steer_angle`** (default `0.35` rad / 20°) — measure the actual max
  wheel-turn angle off the chassis. Determines the minimum turning radius
  (`wheelbase / tan(max_steer_angle)`, ~2.6 m at the default) — a taught
  route with tighter corners than that is physically undrivable regardless
  of tuning.
- **`max_speed_mps`** (default `2.2352` = 5 mph) — must match
  `serial_bridge_node`'s `MAX_SPEED_MPH` constant
  ([`teleop_bridge/serial_bridge_node.py`](../teleop_bridge/teleop_bridge/serial_bridge_node.py)),
  or throttle normalization will be scaled wrong even though it's still
  clamped to ±1. That constant isn't a shared param today — if you change
  one, change the other.
- **Steering sign** — `steer_sign` (default `+1.0`) may need to flip to
  `-1.0`, mirroring `INVERT_STEER` in `gamepad_node.py`. Test on blocks:
  push the car by hand along the path and confirm the wheels steer *toward*
  it, not away.
- **`lookahead_distance`** — too short snakes, too long cuts corners. Start
  at `1.0` m and tune on blocks before a real drive.

## Safety

If the path hasn't loaded, or `/odometry` hasn't published within
`odom_timeout` (default 0.5 s), every control tick publishes an all-zero
`VehicleCommand` — it never repeats the last command on stale data.
