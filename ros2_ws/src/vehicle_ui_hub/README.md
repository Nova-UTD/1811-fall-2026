# 1811 Vehicle UI Hub

The hub is a separate Qt application with an embedded RViz 3D view. It is
independent of the LiDAR-camera registration implementation so other vehicle
pages can be added later.

On the Karbon, start the full stack and the hub with:

```bash
cd ~/1811-fall-2026
./g
```

With a desktop display, this opens the hub full screen and starts registration
in interactive mode on the Jetson. Use the **Window** control, F11, or Escape to
leave full screen. Use **Full screen** or F11 to return. To start in a window
when launching the hub executable directly, pass `--windowed`.

`./g hub:=false` uses the separate RViz window and automatic registration;
`./g rviz:=false` starts without a graphical window. With no `DISPLAY`, `./g`
starts headless and uses automatic registration. An explicit `hub:=true` still
requests the hub even without a display. The shortcut syncs the committed,
pushed Karbon revision to the Jetson; local uncommitted Jetson source changes
are not copied. The Jetson checkout must also be clean for that update.

The overview shows the full `/ouster/points` cloud and full-resolution ZED cloud
messages, with a separate color and layer toggle for each sensor. Drag with the
left mouse button to orbit the RViz scene and use the scroll wheel to zoom.
Before a camera has an alignment estimate its cloud cannot be placed in the
LiDAR frame and is not drawn; it appears as soon as a valid estimate is available.
An absent camera or disabled depth stream remains unavailable.

Open **Calibration**, then select **Begin calibration**. Each camera continues
to be estimated until you select it for review and accept it. A successful
RANSAC/ICP result can be reviewed immediately; the independent confirmation
count is displayed as additional evidence and continues to update. The pose in
the review is held fixed while live clouds and newer estimates keep updating,
so **Accept calibration** applies the exact pose being reviewed. **Decline and
recompute** discards that proposal and starts a fresh search for that camera.

After acceptance, computation stops for that camera. Select its button again
to recalibrate it; the previously accepted vehicle transform remains active
until a replacement is explicitly accepted. Other cameras continue computing.
The accepted camera mount is sent to `robot_state_publisher` and saved by the
registration node; preview transforms use separate frames and do not replace the
vehicle's TF before approval. The saved URDF is a record of the result and is
not automatically restored on the next launch, so calibration starts over when
the stack restarts.

For a layout-only preview with simulated 3D points and no ROS connection:

```bash
ros2 run vehicle_ui_hub vehicle_ui_hub --preview --calibration --windowed
```

The hub uses `/lidar_perception/hub/status` for state, the
`/lidar_perception/hub/<camera>/{review,accept,recompute,release}` services for
decisions, and `/lidar_perception/hub/clouds/<camera>` for the full camera-cloud
previews. The initial view configuration is in `config/hub.rviz`.
