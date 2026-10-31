# Jetson lidar edge detection

Vehicle bringup starts `corner_detector.py` on the Jetson over the existing
SSH connection. The Jetson subscribes to `/ouster/points`, runs Open3D planar
patch and edge detection, and publishes colored patches, black edge points, and
gray unassigned points on `/lidar_perception/edge_cloud`. The Karbon's existing
`sensors.rviz` view shows the processed cloud using the vehicle's TF tree, with
`base_link` as its fixed frame. Detection waits two seconds after each scan, so
updates are at most 0.5 Hz. Open3D's geometry operations use the Jetson CPU in
this program; the code does not use CUDA.

The detector checks for the Jetson's configured cable address, `192.168.100.2`,
before loading Open3D and refuses to run without it. If the Jetson or the link
is unavailable, the rest of vehicle bringup continues; there is no local
detection fallback. If SSH startup fails or its session ends, restart bringup
to start the detector again.

## Setup

The Karbon and Jetson need the same updated repository and the existing cable,
ROS domain, Fast DDS and passwordless SSH setup described in
[`camera_link_setup.md`](../../../docs/camera_link_setup.md). Pull the changes
on **both** computers. On the Karbon, rebuild inside the ROS container:

```bash
cd /vehicle_1811/ros2_ws
colcon build --symlink-install --packages-select lidar_perception obc_bringup vehicle_1811_description
source install/setup.bash
```

Rebuild before running vehicle bringup: an older installed launch file can
still start the previous detector on the Karbon.

The Jetson runs ROS natively. It needs Open3D, NumPy and ROS Python packages in
the Python environment used by `python3`. Check on the Jetson:

```bash
python3 -c 'import numpy, open3d as o, rclpy, sensor_msgs_py; print(o.__version__, hasattr(o.geometry.PointCloud, "detect_planar_patches"))'
```

If they are missing, install versions compatible with the Jetson's ROS and
Python environment before starting detection; see the
[Open3D ARM guide](https://github.com/isl-org/Open3D/blob/main/docs/arm.rst).
The Karbon Docker image's
Open3D installation does not install Open3D on the Jetson. The startup script
builds `lidar_perception` on the Jetson when its installed detector or launch
file is missing or out of date; build errors are reported with a log path.

Run the usual vehicle bringup on the Karbon. The detector is enabled by default;
add `corners_on_jetson:=false` to disable it. Start the existing RViz sensor
view from another Karbon container terminal:

```bash
rviz2 -d $(ros2 pkg prefix vehicle_1811_description)/share/vehicle_1811_description/rviz/sensors.rviz
```

RViz needs the normal host-side `xhost +SI:localuser:root` permission. The
`Lidar edges` display is enabled by default. The raw `Lidar` display is off to
avoid obscuring the colored points; enable it in the Displays panel when raw
lidar is needed, for example if the Jetson is unavailable. Disable
`Lidar edges` when comparing the raw cloud alone. RViz transforms the cloud
for display; the published message remains in its original frame.

The detector downsamples the point cloud and uses a 5 cm edge voxel comparison.
Colors use the same patch bounding-box membership as edge detection and can
change between scans. Its overlapping-box heuristic can miss real corners if
detected patch boxes do not overlap. Jetson processing time depends on the
number of points and patches, and RViz still uses graphics resources on the
Karbon.

## Diagnosis

If no processed cloud appears, check that the Jetson started the detector, can
see `/ouster/points`, and can publish `/lidar_perception/edge_cloud`. Its full
log is on the Jetson at `~/.ros/jetson_cameras/corner_detector_latest.log`.
Check the `Lidar edges` status in RViz for topic or TF errors. The cloud's
source frame must have a transform to `base_link`.

The earlier Karbon reboot has not been explained by the available logs. They
show an Ouster driver segfault and i915 warnings, but later entries follow the
driver fault, and neither proves why the host rebooted. Offloading detection
does not establish or fix that cause.
