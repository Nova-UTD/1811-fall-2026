# Jetson LiDAR and ZED registration

Vehicle bringup starts `lidar_camera_registration.py` on the Jetson over the existing
SSH connection. The Jetson subscribes to `/ouster/points` and the four ZED
clouds, then publishes the registered result for the Karbon's `sensors.rviz`
view, which uses `base_link` as its fixed frame. The node keeps the latest
sensor messages and starts calibration as soon as usable data and TF arrive.
The supplied configuration retries every 5 seconds; a slow attempt cannot overlap the next.
Open3D's geometry operations use the Jetson CPU; this code does not use CUDA.

In the normal vehicle launch, the Karbon starts the ZED camera drivers, depth
processing, panorama (when enabled), and LiDAR-camera registration on the
Jetson over SSH. Registration decodes and downsamples the point clouds, computes
FPFH, RANSAC and ICP, and packs the registered output on the Jetson. There is no
Karbon registration fallback if the Jetson is unavailable.

The Ouster driver and KISS-ICP localization remain on the Karbon. RViz renders
on the machine where it is launched; when its raw ZED displays are enabled,
the four camera point-cloud streams also cross the cable for visualization.

The same node subscribes to the front, left, rear and right ZED X point clouds.
For each camera, it removes large, nearly vertical planar patches from the
calibration copies of both clouds, estimates an initial camera-to-LiDAR pose from
FPFH shape features and RANSAC, then refines that pose with point-to-point ICP. No camera
mounting measurement or camera-to-LiDAR TF is used for this registration.
It publishes the unfiltered result on `/lidar_perception/registered_points` in
the LiDAR frame, with the LiDAR timestamp. The message contains source colors:
front blue, left green, rear orange and right magenta; LiDAR points are white.
The merged display is empty when no calibrated camera has a usable cloud, so a
copy of the LiDAR alone cannot be mistaken for a successful registration. The
original LiDAR remains visible on `/ouster/points`.
Walls and ground remain visible in the original streams and registered output.
The registered output contains every finite point by default; set
`visualization_voxel_size_m` to a positive value to reduce display traffic.
The wall filter uses the measured LiDAR TF to find up. For each uncalibrated
camera, it uses the camera body's +Z axis and the driver's internal camera TF,
assuming the camera is mounted roughly upright. A camera mounted sideways can
therefore classify walls incorrectly. The filter detects geometry, not semantic
walls: large vertical flat objects may also be removed, and incomplete walls
may remain. It normally keeps level floor patches, but can reduce overlap enough
to prevent calibration.
`Wall filter` logs report detected patches and point counts per sensor.

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
python3 -c 'import numpy, open3d as o, rclpy, sensor_msgs_py, tf2_ros; print(o.__version__)'
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

RViz shows raw LiDAR in red and the raw ZED clouds in blue, green, orange and
magenta. The registered display uses `RGB8` to show each sensor's color in the
merged message. It includes a downsampled LiDAR copy only while a calibrated
camera has a usable cloud. Toggle other displays if they obscure the colored
points. `Panorama` displays the original `/panorama/image` when enabled.

The source URDF retains the measured LiDAR mount and starts the four camera
mounts as floating joints. A floating mount does not publish a camera-to-base
TF. The previous manual camera measurements are backed up as comments in the
source xacro and are never supplied to registration. Until a camera is
calibrated its raw cloud has no TF path to `base_link`.

At startup the Jetson matches FPFH features with RANSAC and refines each result
with ICP. Three estimates from distinct LiDAR and camera scans must agree within
5 cm and 3 degrees before a new camera pose is published. This makes a single
random RANSAC result less likely to become the camera mount. The ROS log shows
the confirmation count, ICP fitness, RMSE and number of matched points.

At each configured interval the node uses the last accepted pose to run ICP
again. Updates cannot drift more than 5 cm or 3 degrees from the confirmed pose.
After three consecutive ICP failures or rejected updates, the next attempt
retries FPFH/RANSAC and requires independent confirmations again.
The last accepted mount remains active when a new attempt fails; a changed
cloud frame triggers a new global search. The node uses fresh, time-matched
clouds and computes LiDAR features once per attempt for cameras needing them.
The configured interval is a minimum between attempt starts: if registration takes
longer, the next attempt waits for it to finish.

ICP produces a transform from the camera *cloud frame* to the LiDAR *cloud
frame*. The node combines it with the LiDAR TF and the ZED driver's internal
body-to-lens TF to obtain the `base_link` to camera-body mount. It updates that
joint in `/robot_state_publisher`'s `robot_description` parameter. The vehicle
publisher then republishes the URDF and its fixed TFs; there is no separate
camera-mount TF broadcaster. The measured LiDAR joint is not changed.

After the vehicle publisher accepts an update, the Jetson writes the expanded
URDF to `~/.ros/lidar_camera_calibration/vehicle_1811_calibrated.urdf`. The
source checkout is not rewritten. That file is a record of the latest accepted
result, not an input on the next launch: registration starts from geometry
again. If a TF, URDF, service, or cloud is missing, the affected camera is not
newly calibrated and the node logs the reason. A rejected update keeps the last
accepted pose. The registered output uses the same accepted pose between
attempts, instead of running ICP for every displayed scan.

With the default settings, a camera is skipped if its timestamp differs from
the LiDAR by more than 0.30 s, either cloud is too old, there are too few
points or matches, RANSAC fitness is below 0.15, ICP fitness is below 0.10,
ICP match RMSE exceeds 0.08 m, the camera origin is estimated more than 3 m
from the LiDAR, or the ICP correction from its starting estimate exceeds 0.5 m
or 15 degrees. ICP only matches points within 0.15 m. The 3 m distance is a
broad limit for this vehicle, not an initial camera pose; adjust it if a camera
really is farther from the LiDAR. These thresholds are initial settings, not a
guarantee of physical accuracy; a
repetitive scene can still yield a repeatable incorrect match. Check alignment
against multiple real objects before using a calibration. Initial confirmation
requires multiple attempts and can take substantially longer than later ICP.

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

A camera calibration is relative to LiDAR. It cannot correct an incorrect
LiDAR-to-vehicle transform or independently level the LiDAR. Updating fixed TFs
changes the poses used by other ROS consumers as well as RViz. TF treats each
replacement as static, so it can also affect how older sensor data is interpreted.
Validate the estimated mounts before using them for vehicle perception.

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
