# mode_manager

`mode_manager_node` decides which command source is allowed to drive the car.
It is the only node that publishes `/vehicle_command` under
`bringup.launch.py`, which starts it by default. This is step **A3** of
[`docs/teach_and_repeat_plan.md`](../../../docs/teach_and_repeat_plan.md).

> **Status: manual driving tested on the car; teach and repeat tested in
> simulation only.** Before the first autonomous run: verify the button indices
> (below), then test with the **wheels off the ground**, spotter present, hand
> on the kill switch. `use_mode_manager:=false` starts bringup without the
> gamepad, pure pursuit, or this node.

## Modes

| Mode | What reaches `/vehicle_command` | Enter with |
|---|---|---|
| **DISABLED** | zero throttle and steering, **brake on** | **B**, and at startup |
| **MANUAL** | the gamepad sticks | **A**, or squeeze the brake while AUTONOMOUS |
| **AUTONOMOUS** | pure pursuit — **only while RB is held**; released RB = brake | **Start** |

- **Deadman (RB):** in AUTONOMOUS, the car only drives while RB is held.
  Releasing it sends a full-brake stop; the mode stays AUTONOMOUS so holding RB
  again resumes.
- **Takeover:** squeezing the brake trigger while AUTONOMOUS switches straight
  to MANUAL, so your brake reaches the car immediately.
- **AUTONOMOUS is refused** (logged as a warning) if pure pursuit isn't
  publishing, the gamepad isn't publishing, or the brake is pressed.
- **Lost input stops the car.** If the active source goes quiet for
  `input_timeout` (0.5 s), the output becomes a brake-and-stop command, not
  the last command it saw. This also covers the gamepad unplugging mid-drive.
- **B always wins.** If several mode buttons are pressed at once, DISABLED
  takes priority.

## Teach and repeat with one launch

```bash
ros2 launch obc_bringup bringup.launch.py lidar_ip:=<sensor-ip> port:=/dev/serial/by-id/$(ls /dev/serial/by-id/ | grep Arduino)
```

1. **A** → MANUAL. Drive to the start of the loop.
2. **Y** → start recording. Drive the loop and stop back where you started.
3. **Y** again → stop and save. The route is written to `/vehicle_1811/routes`
   **and** handed straight to pure pursuit (no restart).
4. **Dry run:** stay in MANUAL with the sticks centred (the car coasts, brake
   off), run `ros2 topic echo /cmd/auto` in a second shell, and push the car by
   hand. Steering should track the path smoothly and stay well inside ±1.
   (DISABLED holds the brake on, so the car can't be pushed in it.)
5. **Start** → AUTONOMOUS, then **hold RB** to drive. Let go of RB, or squeeze
   the brake, to stop.
6. Pure pursuit stops by itself at the end of the route. **For another lap**,
   drive back to the start and re-send the same route:
   `ros2 service call /route_recorder_node/save std_srvs/srv/Trigger`
   (don't press Y — that starts a new recording and clears the route).

Don't run `teleop_bridge.launch.py` alongside it: it would add a second
publisher on `/vehicle_command`.

## Verify the buttons first

The axis mapping (brake on `axes[2]`) matches `gamepad_node.py` and was
measured on the pad. The **button** indices are the standard Linux xpad layout
and have **not** been checked on the 8BitDo pad. Run `ros2 topic echo /joy`,
press each button, and fix
[`config/mode_manager.yaml`](config/mode_manager.yaml) if any differ:

| Param | Default | Button |
|---|---|---|
| `manual_button` | 0 | A |
| `disable_button` | 1 | B |
| `record_button` | 3 | Y |
| `deadman_button` | 5 | RB |
| `autonomous_button` | 7 | Start |

## Contract

| | |
|---|---|
| **Subscribes** | `/joy` (`sensor_msgs/Joy`), `/cmd/manual` and `/cmd/auto` (`vehicle_msgs/VehicleCommand`) |
| **Publishes** | `/vehicle_command` at `publish_rate` (20 Hz, inside the Arduino's 250 ms cutoff); `/guardian/mode` (`std_msgs/String`, latched) |
| **Calls** | `/route_recorder_node/start_recording`, `stop_recording`, `save` (record button) |

In one-launch mode, `gamepad_node` is remapped to publish `/cmd/manual`, and
`pure_pursuit_node` runs with `path_file:=''` so it follows whatever route
`route_recorder_node` publishes on `/planning/path` when it saves.

## Tests

The mode logic lives in [`mode_manager_core.py`](mode_manager/mode_manager_core.py)
with no ROS imports, so it's unit-tested directly:

```bash
colcon test --packages-select mode_manager && colcon test-result --verbose
```
