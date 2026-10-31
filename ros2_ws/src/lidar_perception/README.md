# Jetson lidar edge detection

Vehicle bringup starts `corner_detector.py` on the Jetson over the existing
SSH connection. The Jetson subscribes to `/ouster/points`, runs Open3D planar
patch and edge detection, and publishes black points with neon-blue detected edges
on `/lidar_perception/edge_cloud`. The Karbon's existing `sensors.rviz` view
shows the processed cloud using the vehicle's TF tree, with
`base_link` as its fixed frame. The detector processes scans without an added
delay and keeps only the latest pending scan when it is busy. Its update rate
depends on the lidar and the time needed to process each scan. Open3D's geometry
operations use the Jetson CPU in this program; the code does not use CUDA.

When the panorama is enabled, a separate Jetson process running the camera node
in `corner_detector.py` subscribes to `/panorama/image`, detects image edges
with OpenCV, and publishes the original picture with neon-blue edges on
`/panorama/edges`. The Karbon's RViz displays that topic; it does not run the
detector. The separate process uses the same system NumPy/OpenCV environment as
the panorama node and does not wait for LiDAR processing.

Both detectors check for the Jetson's configured cable address, `192.168.100.2`,
before loading their processing libraries and refuse to run without it. If the
Jetson or the link is unavailable, the rest of vehicle bringup continues; there
is no local detection fallback. If SSH startup fails or its session ends,
restart bringup to start the detectors again.

## Setup

The Karbon and Jetson need the existing cable, ROS domain, Fast DDS and
passwordless SSH setup described in
[`camera_link_setup.md`](../../../docs/camera_link_setup.md). Update the Karbon
checkout, then on the Karbon host run:

```bash
cd ~/1811-fall-2026
./g
```

The shortcut builds the Karbon workspace, updates the Jetson to the Karbon's
committed branch and revision, builds the Jetson packages, starts its processes,
and opens RViz. It starts the full vehicle stack. The commit must be pushed and
the Jetson checkout clean; if an update cannot be performed safely, Jetson
processes are skipped with an error. `jetson_update:=false` skips this update
and uses the Jetson's existing checkout.

The Jetson runs ROS natively. LiDAR detection needs Open3D, NumPy and ROS
Python packages in the Python environment used by `python3`. Check on the Jetson:

```bash
python3 -c 'import numpy, open3d as o, rclpy, sensor_msgs_py; print(o.__version__, hasattr(o.geometry.PointCloud, "detect_planar_patches"))'
```

If they are missing, install versions compatible with the Jetson's ROS and
Python environment before starting detection; see the
[Open3D ARM guide](https://github.com/isl-org/Open3D/blob/main/docs/arm.rst).
The Karbon Docker image's
Open3D installation does not install Open3D on the Jetson. The startup script
builds `lidar_perception` on the Jetson when its installed detector or launch
file is missing or out of date, and builds `camera_panorama` when needed. The
camera overlay also needs system OpenCV and NumPy; the startup script checks
those with `PYTHONNOUSERSITE=1`, matching `panorama.launch.py`. If that check
fails, it reports the error and still starts the LiDAR detector. Build errors
are reported with a log path.

Both detectors are enabled by default; add `corners_on_jetson:=false` to
disable them, `panorama:=false` to skip the panorama, or `rviz:=false` for a
headless run. `./g` requests the host-side X11 permission for RViz; if no
`DISPLAY` is set it starts without RViz.

The `Lidar edges` display is enabled by default. The raw `Lidar` display is off to
avoid obscuring the colored points; enable it in the Displays panel when raw
lidar is needed, for example if the Jetson is unavailable. Disable
`Lidar edges` when comparing the raw cloud alone. RViz transforms the cloud
for display; the published message remains in its original frame. The
`Panorama` display shows `/panorama/edges`; enable `Panorama original` to show
the unmodified `/panorama/image`, including when camera edges are unavailable.

The detector first finds candidate edge voxels shared by nonparallel patch
boxes. For each point in those voxels, it checks normals of nearby points in
the whole cloud. Only points supported by both checks are blue; the remaining
cloud is black. Opposite-facing normals are treated as the same surface
orientation.

The initial point-normal check requires at least 3 neighbors within 0.10 m
whose normals differ by more than 30 degrees. It examines up to 30 neighbors,
including the point itself. These values and `EDGE_VOXEL_SIZE` are constants in
`corner_detector.py`. A larger edge voxel increases the spatial tolerance for
matching the checks; it does not change the normal search radius.

This intersection may remove false positives, but it can also miss edges when
the patch boxes do not overlap, the normals are noisy, or the cloud is sparse.
Jetson processing time depends on the number of points and patches; compare
the output and rate on a recorded scan when tuning the thresholds.

## Diagnosis

If no processed cloud appears, check that the Jetson started the detector, can
see `/ouster/points`, and can publish `/lidar_perception/edge_cloud`. Its full
log is on the Jetson at `~/.ros/jetson_cameras/corner_detector_latest.log`.
Check the `Lidar edges` status in RViz for topic or TF errors. The cloud's
source frame must have a transform to `base_link`.

If the annotated panorama is empty, confirm the panorama is enabled (the default
for `./g`), that the Jetson logs say `Panorama edge detector started`, and that
`/panorama/edges` is publishing. Both its logs and the LiDAR detector's logs are in
`~/.ros/jetson_cameras/corner_detector_latest.log` on the Jetson.

To measure the processed cloud rate on the Karbon, run
`ros2 topic hz /lidar_perception/edge_cloud --qos-reliability best_effort`.

The earlier Karbon reboot has not been explained by the available logs. They
show an Ouster driver segfault and i915 warnings, but later entries follow the
driver fault, and neither proves why the host rebooted. Offloading detection
does not establish or fix that cause.
