# vehicle_1811_description

The xacro describes the 1811 vehicle, wheels, measured Ouster mount and four
ZED X camera bodies. `base_link` is at the rear axle on the ground; x points
forward, y left and z up. Distances are meters and angles are radians.

The LiDAR joint is fixed from `base_link` to `os_sensor`. The Ouster driver
adds the sensor's internal `os_lidar` and `os_imu` frames from its metadata.
Keep the LiDAR measurements accurate: they define the reference used by
localization and camera calibration. The xacro marks the LiDAR optical-origin
height as needing verification.

Camera joints start as floating, with no camera mount TF. The previous manual
camera measurements are retained as comments in the xacro for reference only.
When LiDAR-camera registration succeeds on the Jetson, it updates the live
`robot_description` with each estimated fixed camera mount. The vehicle's
`robot_state_publisher` publishes those mounts, and the ZED wrapper supplies
its internal lens frames. Cameras that have not calibrated remain disconnected
from `base_link`.

The Jetson also saves the latest expanded model to
`~/.ros/lidar_camera_calibration/vehicle_1811_calibrated.urdf`; it does not
change the source xacro. Registration reloads the saved camera mounts on the
next launch and applies them to the current vehicle model. The Hub then shows
those cameras as accepted; **Recompute** starts a replacement calibration when
a sensor has moved. See [LiDAR-camera registration](../lidar_perception/README.md)
for the launch, calibration interval, limitations and troubleshooting.

For the vehicle TF tree, bringup includes `description.launch.py`. To inspect
the source model with RViz and joint sliders, run:

```bash
ros2 launch vehicle_1811_description display.launch.py
```

Without a running registration node, the camera mounts remain uncalibrated in
this display. Check the raw LiDAR against the vehicle model separately.
