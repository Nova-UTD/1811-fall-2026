# Jetson lidar edge detection

Vehicle bringup starts `corner_detector.py` on the Jetson over the existing
SSH connection. The Jetson subscribes to `/ouster/points`, runs Open3D planar
patch and edge detection, and publishes a gray cloud with blue edge points on
`/lidar_perception/edge_cloud`. The Karbon subscribes to the processed cloud
and opens the Open3D viewer when a result arrives. Detection waits two seconds
after each scan, so updates are at most 0.5 Hz. Open3D's geometry operations
use the Jetson CPU in this program; the code does not use CUDA.

The detector checks for the Jetson's configured cable address, `192.168.100.2`,
before loading Open3D and refuses to run without it. If the Jetson or the link
is unavailable at startup, the Karbon's small viewer node waits without loading
Open3D or processing raw lidar; there is no local detection fallback. After a
connection loss, if results stop for 30 seconds and the ROS publisher is gone,
the viewer exits and releases its Open3D resources. The rest of vehicle bringup
continues. If SSH startup fails or its session ends, restart bringup to start
the detector and viewer again.

## Setup

The Karbon and Jetson need the same updated repository and the existing cable,
ROS domain, Fast DDS and passwordless SSH setup described in
[`camera_link_setup.md`](../../../docs/camera_link_setup.md). Pull the changes
on **both** computers. On the Karbon, rebuild inside the ROS container:

```bash
cd /vehicle_1811/ros2_ws
colcon build --symlink-install --packages-select lidar_perception obc_bringup
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

Run the usual vehicle bringup on the Karbon. The detector and viewer are
enabled by default; add `corners_on_jetson:=false` to disable both. The viewer
needs access to the Karbon desktop, including the normal host-side
`xhost +SI:localuser:root` permission used for RViz.

The detector keeps the existing 3 cm point-cloud downsampling and 5 cm edge
voxel comparison. Its overlapping-box heuristic can miss real corners if
detected patch boxes do not overlap. Jetson processing time depends on the
number of points and patches, and the viewer still uses graphics resources on
the Karbon.

## Diagnosis

If no window appears, check that the Jetson started the detector, can see
`/ouster/points`, and can publish `/lidar_perception/edge_cloud`. Its full log
is on the Jetson at `~/.ros/jetson_cameras/corner_detector_latest.log`. Check
the Karbon viewer's logs for display errors. A running ROS viewer without
processed clouds does not mean detection is running locally.

The earlier Karbon reboot has not been explained by the available logs. They
show an Ouster driver segfault and i915 warnings, but later entries follow the
driver fault, and neither proves why the host rebooted. Offloading detection
does not establish or fix that cause.
