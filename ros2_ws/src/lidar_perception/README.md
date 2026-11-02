# Jetson LiDAR and ZED registration

Vehicle bringup starts `lidar_camera_registration.py` on the Jetson over the existing
SSH connection. The Jetson subscribes to `/ouster/points` and the four ZED
clouds, then publishes the registered result for the Karbon's `sensors.rviz`
view, which uses `base_link` as its fixed frame. The registration node keeps only the
latest pending scan when it is busy. Its update rate depends on the sensors
and the time needed to register each scan. Open3D's geometry operations use the
Jetson CPU in this program; the code does not use CUDA.

In the normal vehicle launch, the Karbon starts the ZED camera drivers, depth
processing, panorama (when enabled), and LiDAR-camera registration on the
Jetson over SSH. Registration decodes and downsamples the point clouds, computes
FPFH, RANSAC and ICP, and packs the registered output on the Jetson. There is no
Karbon registration fallback if the Jetson is unavailable.

The Ouster driver and KISS-ICP localization remain on the Karbon. RViz renders
on the machine where it is launched; when its raw ZED displays are enabled,
the four camera point-cloud streams also cross the cable for visualization.

The same node subscribes to the front, left, rear and right ZED X point clouds.
For each camera, it estimates an initial camera-to-LiDAR pose from FPFH shape
features and RANSAC, then refines that pose with point-to-point ICP. No camera
mounting measurement or camera-to-LiDAR TF is used for this registration.
It publishes the downsampled result on `/lidar_perception/registered_points` in
the LiDAR frame, with the LiDAR timestamp. The message contains source colors:
front blue, left green, rear orange and right magenta; LiDAR points are white.
Walls and ground remain in the input and output; ICP downsamples the point
clouds but does not detect or remove planar surfaces.

The registration node checks for the Jetson's configured cable address, `192.168.100.2`,
before loading its processing libraries and refuses to run without it. If the
Jetson or the link is unavailable, the rest of vehicle bringup continues; there
is no local registration fallback. If SSH startup fails or its session ends,
restart bringup to start the processes again.

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

The Jetson runs ROS natively. Registration needs Open3D, NumPy and ROS
Python packages in the Python environment used by `python3`. Check on the Jetson:

```bash
python3 -c 'import numpy, open3d as o, rclpy, sensor_msgs_py; print(o.__version__)'
```

If they are missing, install versions compatible with the Jetson's ROS and
Python environment before starting registration; see the
[Open3D ARM guide](https://github.com/isl-org/Open3D/blob/main/docs/arm.rst).
The Karbon Docker image's
Open3D installation does not install Open3D on the Jetson. The startup script
builds `lidar_perception` on the Jetson when its installed registration node or launch
file is missing or out of date. Build errors are reported with a log path.

Registration and depth on all started cameras are enabled by default through
`./g`. Use `registration_on_jetson:=false` to disable registration and automatic
depth, or `depth_cameras:=front,rear` to use only selected depth cameras.
The older `corners_on_jetson` switch remains a compatibility alias; if both are
provided, `registration_on_jetson` takes precedence.
`panorama:=false` skips the independent panorama, and `rviz:=false` starts
without RViz. `./g` requests host-side X11 permission for RViz; if no `DISPLAY`
is set it starts without RViz. The direct `zed_cameras.launch.py` defaults to
depth off, so pass `depth_cameras:=all` when launching cameras that way.

## Registration settings

Edit [`config/registration.yaml`](config/registration.yaml) to adjust voxel
sizes, feature search distances, RANSAC, ICP, synchronization and the packed
sensor colors. The comments show the units and effects of each group. The
normal, feature and RANSAC search distances are multipliers of the FPFH voxel
size, so changing that voxel size changes all three distances automatically.
The original values remain the defaults; they have not been validated on the
vehicle.

Settings are read once when the registration node starts. Restart it after changes;
ROS parameter updates while it is running are rejected rather than ignored.
For the normal `./g` workflow, commit and push the Karbon checkout changes so
the Jetson can sync them; uncommitted Jetson source changes block that sync.
`jetson_update:=false` uses the Jetson's existing checkout instead.

When launching the registration node directly on the Jetson, an alternate ROS parameter
file can be selected with:

```bash
ros2 launch lidar_perception lidar_camera_registration.launch.py config_file:=/absolute/path/registration.yaml
```

`ros2 run lidar_perception lidar_camera_registration.py` without a parameter file uses the
same built-in defaults instead of automatically reading the launch YAML.
If an older custom YAML file uses `corner_detector:` as its top-level node name,
rename that key to `lidar_camera_registration:` so the new node receives its settings.

RViz enables six cloud displays: raw LiDAR in red, raw ZED front in blue, left
in green, rear in orange, right in magenta, and the registered result in white.
The registered display uses smaller points to make overlapping inputs easier to
compare. Toggle displays individually if points obscure each other. RViz uses the
existing URDF/TF to place the original sensor clouds; these positions are not
updated by registration. The registered cloud remains in its LiDAR source frame.
Set the registered display's color transformer to `RGB8` to see
the source colors packed in its message instead of white. `Panorama` displays the
original `/panorama/image` when the independent panorama is enabled.
The legacy standalone `corner_viewer.py` still listens to
`/lidar_perception/filtered_points`, which this node no longer publishes.

By default, the registration node downsamples the clouds to 10 cm for ICP and 20 cm for FPFH. It
estimates normals, describes local shapes with FPFH, and searches for an initial
pose using RANSAC. It computes LiDAR features once per scan and reuses them for
all cameras needing an initial pose. ICP then refines the result with up to 20
iterations and a 0.5 m correspondence limit. Each camera's first accepted pose
is held in memory as the starting estimate for later scans; per-scan ICP results
do not overwrite it. Three consecutive ICP failures discard the cached estimate
and trigger a new global search. A changed camera or LiDAR frame ID also discards
the corresponding estimate. Restarting the node clears all estimates.
When a camera initializes, the registration node logs its 4x4 anchor matrix and the
camera and LiDAR frame IDs. The matrix maps points from the camera message's
frame into the LiDAR message's frame; it is not a `base_link` to camera URDF
mount transform.

With the default settings, a camera is skipped if its timestamp differs from the LiDAR by more than 0.30 s,
there are too few points or matches, RANSAC or ICP fitness is below 0.10, or
the ICP correction from its starting estimate exceeds 0.5 m or 15 degrees.
These are starting thresholds, not validated calibration. The output contains
LiDAR and only the cameras that pass; the node logs which cameras registered
or were skipped. With no accepted cameras the output contains LiDAR alone.
The initial global search can take substantially longer than later scans,
particularly when a scene has little overlap; monitor Jetson CPU and output rate.

The Karbon and Jetson clocks must agree, and LiDAR and camera message stamps
must use the same time base. The Ouster launch currently keeps the sensor's
configured timestamp mode; if it uses an unsynchronized internal clock, the
stamps can differ even when chrony is synchronized. The optional
`lidar_timestamp_mode:=TIME_FROM_ROS_TIME` uses Karbon packet-reception time for
LiDAR headers; it also changes timestamps seen by other LiDAR consumers, so
verify localization when using it. There is no motion compensation between
camera and LiDAR captures. FPFH/RANSAC and ICP cannot guarantee a correct pose:
plain or repeated surfaces, moving objects, low overlap, and the different
densities of the two sensors can produce a wrong match that passes the thresholds.
Start with the vehicle stationary and distinctive overlapping 3D objects visible
to both sensors. Check alignment in several different scenes before using the
result for perception.

This node publishes registered points only. It does not publish the estimated
camera transforms on TF, save them, or generate a URDF. Existing camera URDF/TF
mounts remain in use by other nodes and the raw RViz camera displays; the LiDAR
still needs its existing measured transform to `base_link`. A registered camera
cloud appearing aligned does not validate those existing camera TF mounts.

## Diagnosis

If no registered cloud appears, check `/ouster/points`,
`/lidar_perception/registered_points`, and the RViz display status. Read
`~/.ros/jetson_cameras/lidar_camera_registration_latest.log` on the Jetson for missing
camera messages, timestamp differences, or rejected RANSAC and ICP matches.
Use `bash scripts/check_link.sh` to check the Jetson clock and network link.
The output frame must have a TF to RViz's `base_link` fixed frame.

To check the registration and panorama processes on the Jetson from the Karbon:

```bash
ssh -i ~/.ssh/id_ed25519_jetson -o IdentitiesOnly=yes nova@192.168.100.2 \
  "pgrep -af '[l]idar_camera_registration|[p]anorama_node'"
```

To measure the registered cloud rate on the Karbon, run
`ros2 topic hz /lidar_perception/registered_points --qos-reliability best_effort`.

The earlier Karbon reboot has not been explained by the available logs. They
show an Ouster driver segfault and i915 warnings, but later entries follow the
driver fault, and neither proves why the host rebooted. Offloading registration
does not establish or fix that cause.
