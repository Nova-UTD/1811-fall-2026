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

The same node subscribes to the front, left, rear and right ZED X point clouds
and `/ouster/imu`. It crops each cloud to a calibration box, removes LiDAR ground
using the IMU and a fitted plane, removes camera ground using the ZED SDK floor
planes, and optionally removes large, nearly vertical planar patches. It then
estimates an initial camera-to-LiDAR pose using FPFH and RANSAC and refines it
with point-to-point ICP.
No camera mounting measurement or
camera-to-LiDAR TF is used for this registration.
Each attempt publishes clouds for inspection on
`/lidar_perception/calibration_input/lidar` and
`/lidar_perception/calibration_input/{front,left,rear,right}`. They keep their
sensor frames and are published before RANSAC, even when no pose is accepted.
With `wall_filter_enabled: true`, these are the downsampled, filtered inputs
used for matching. When false, they contain the full-resolution cropped inputs
after the enabled ground filters, including walls; matching still uses downsampled copies.
The LiDAR input is visible in `base_link`. The camera input displays start
disabled: before calibration there is no camera-to-base TF, so enable one and
temporarily set RViz's fixed frame to its cloud frame (for example,
`zed_front_left_camera_frame`) to inspect it. Restore `base_link` afterward.

The optional [Vehicle UI Hub](../vehicle_ui_hub/README.md) starts this node in
interactive mode: calibration waits for the operator to start it, and each
camera's pose is applied only after explicit review and acceptance. The hub's
approval and preview backend is in `interactive_calibration.py`; the ordinary
launch retains automatic calibration.

`/lidar_perception/registered_points` is a colored overlay using the
last accepted poses. It is empty when no accepted camera has a usable cloud, so
LiDAR alone cannot be mistaken for a successful registration. The original
LiDAR and ZED messages are unchanged on their sensor topics. The overlay uses
the same filtered or full-resolution cropped points as the separate displays;
`visualization_voxel_size_m` can optionally downsample the merged overlay
without changing the input displays.
The wall filter uses the measured LiDAR TF to find up. For each uncalibrated
camera, it uses the camera body's +Z axis and the driver's internal camera TF,
assuming the camera is mounted roughly upright. A camera mounted sideways can
therefore classify walls incorrectly. The filter detects geometry, not semantic
walls: large vertical flat objects may also be removed, and incomplete walls
may remain. It can reduce overlap enough to prevent calibration.
At ROS DEBUG level, `Wall filter` logs report detected patches and point counts.

The LiDAR ground filter averages `/ouster/imu` acceleration samples close to the
LiDAR scan timestamp, rotates their estimated up direction into `base_link`, and
fits candidate planes to low LiDAR points. It checks each plane's tilt, height
at `base_link`, and number of inliers, then chooses the valid plane closest to
the nominal ground height at the origin. That plane is extended over the cropped
cloud: points within 8 cm above it and all points below it are removed before
downsampling. `LiDAR ground` logs report the IMU sample count, plane height and
tilt, inliers, and removed and remaining point counts.

Calibrate with the vehicle stationary. The filter requires at least three recent
samples within 0.30 s of the scan timestamp, each with acceleration magnitude
8.0–11.5 m/s² and angular speed at most 0.20 rad/s. These checks cannot detect
all vehicle motion. If the IMU samples or TF are missing, or no plane passes the
checks, that LiDAR input is empty and registration waits for a later attempt.
Set `lidar_ground_filter_enabled: false` to disable this filter independently of
the ZED ground filter.

The supplied 8 cm tolerance, 30 cm search height, 25 degree tilt limit, and
50-inlier minimum are starting values, not vehicle-calibrated thresholds.
Ground more than 8 cm above the fitted plane can remain on uneven terrain; low
objects and any geometry below the plane can be removed. The fitted plane may
also extend beyond the area where ground was observed. Inspect
`/lidar_perception/calibration_input/lidar` against `/ouster/points` in RViz and
check that enough shared, non-ground geometry remains for registration.

Each depth-enabled camera used for registration publishes an SDK floor estimate
on `/zed_<position>/zed_node/floor_plane`. Its position is the plane center and
its local +Z axis is the normal, in the same frame and capture timestamp as the
ZED cloud. The filter accepts only the plane from that exact camera frame. If
tracking or floor detection fails, or the matching message is missing, that
camera's cloud is skipped; the other cameras and LiDAR continue. `ZED ground`
logs show the cropped, removed and remaining point counts for each camera.

The 5 cm `ground_distance_tolerance_m` is a starting value, not a calibrated
vehicle threshold. The filter extends the detected plane across the cropped
cloud; low objects or other coplanar surfaces can be removed, and rough or
sloped ground may remain. Check `/lidar_perception/calibration_input/<camera>`
in RViz against the raw camera cloud before relying on a calibration.

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

For the SDK floor estimate, `scripts/jetson_cameras.sh` builds an isolated
patched copy of the installed `zed_components` 5.2.2 source under
`~/.cache/1811/zed_floor_wrapper`, then sources it for the camera launch. The
original `~/zed_ws` source and installation are not modified. The first build
can take several minutes; later starts reuse the overlay unless its source or
patch changes. A missing or incompatible wrapper source prevents registration
from starting, while the camera launch remains available. The depth-enabled
cameras use SDK positional tracking for floor detection; ZED odometry TFs stay
disabled so they do not compete with vehicle localization. Four-camera tracking
and floor detection add Jetson load and require live performance validation.
Setting `ground_filter_enabled: false` disables this filter, wrapper overlay and
extra tracking; ground points then remain in the camera calibration inputs.

Registration and depth on all started cameras are enabled by default through
`./g`. Use `registration_on_jetson:=false` to disable registration and automatic
depth, or `depth_cameras:=front,rear` to use only selected depth cameras.
The older `corners_on_jetson` switch remains a compatibility alias; if both are
provided, `registration_on_jetson` takes precedence.
`panorama:=true` enables the independent panorama, and `rviz:=false` starts
without RViz. `./g` requests host-side X11 permission for RViz; if no `DISPLAY`
is set it starts without RViz. To start the cameras and registration directly on
the Jetson, including the wrapper overlay, run:

```bash
bash ~/1811-fall-2026/scripts/jetson_cameras.sh --prepare --registration --depth-cameras=all
```

The direct `zed_cameras.launch.py` defaults to depth and floor detection off. It
does not build or source the overlay; use the script above for registration.

## Registration settings

Edit [`config/registration.yaml`](config/registration.yaml) to adjust voxel
sizes, calibration boxes, feature search distances, RANSAC, ICP, synchronization and the packed
sensor colors. The comments show the units and effects of each group. The
normal, feature and RANSAC search distances are multipliers of the FPFH voxel
size, so changing that voxel size changes all three distances automatically.

With `cloud_diagnostics_enabled: true`, the Jetson logs a summary of each fresh
LiDAR and ZED cloud processed at the calibration interval. It reports the ROS
frame and timestamp; raw, finite, cropped, downsampled and matching point counts;
detected wall patches and removed points; and the matching cloud's XYZ bounds and
centroid in its original sensor frame, in meters. A processing failure is logged
with the raw and finite counts when possible. These are sampled registration
inputs, not a line for every incoming ROS message.

During a global search, `FPFH` lines describe the actual 33-bin descriptors used
by RANSAC. They show the total, finite and all-zero descriptor counts, the mean
standard deviation across bins, and three groups of 11 mean histogram bins. The
spread summarizes variation within that cloud; it is not a match score. RANSAC
and ICP lines report fitness, match count and applicable limits; ICP also reports
RMSE. Accepted cameras normally use cached-pose ICP, so FPFH is computed and
logged again only when a new global search is needed. Set
`cloud_diagnostics_enabled: false` and restart the node to quiet these INFO logs.

The starting boxes use `[forward, left, up]` coordinates in meters:

| Cloud | Frame | Minimum | Maximum |
| --- | --- | --- | --- |
| LiDAR | `base_link` | `[-5.0, -5.0, -0.5]` | `[5.0, 5.0, 3.5]` |
| Each camera | `zed_<position>_camera_link` | `[0.5, -2.0, -1.5]` | `[3.5, 2.0, 1.5]` |

The camera box is 3 m long, 4 m wide and 3 m high, in front of its own body;
its outer edges can exclude parts of a wide-angle view. The LiDAR box is 10 m
long and wide, giving room for the camera positions and all four viewing
directions. This is a starting point, not a measured optimum: tune it to retain
distinctive surfaces that both sensors see. The LiDAR-to-`base_link` TF and
each camera's internal lens-to-body TF are required before its crop can be
prepared. The box test uses those transforms but leaves the published and matched
points in their original cloud frames. Missing TF skips that input until TF is
available; it does not assume a camera mount. Only registration inputs and its
RViz outputs are cropped; the raw sensor topics and other consumers are unchanged.

The 3.5 m forward limit favors nearby stereo data while retaining some geometry
beyond 3 m. [Stereolabs' ZED X NEURAL_LIGHT measurements](https://docs.stereolabs.com/docs/development/zed-sdk/modules/depth-sensing/depth-modes/)
report increasing depth error beyond 3 m for a ZED X with a 2.2 mm lens. Actual
accuracy and overlap depend on the camera, lighting, scene and LiDAR coverage;
these crop limits have not been validated on the vehicle.

The supplied matching settings use 5 cm voxels for ICP and 10 cm voxels for
FPFH. RANSAC and ICP use 15 cm and 12 cm correspondence limits, respectively;
the quality thresholds reject weak overlap and high match error. These are
initial settings, not measured accuracy guarantees. Finer ICP voxels use more
CPU, and stricter thresholds can reject usable scans when overlap is limited.
Check the alignment and rejection messages on the car before tuning one value
at a time.

The calibration regression checks are kept in one file. From the repository root:

```bash
python3 -m unittest discover -s ros2_ws/src/lidar_perception/test -p test_calibration.py
```

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

`ros2 run lidar_perception lidar_camera_registration.py` without a parameter file uses
built-in defaults, which can differ from the launch YAML.
If an older custom YAML file uses `corner_detector:` as its top-level node name,
rename that key to `lidar_camera_registration:` so the new node receives its settings.

RViz shows the LiDAR input in red. ZED input displays use blue,
green, orange and magenta; enable them when inspecting a camera or after its TF
is calibrated. The registered display uses `RGB8` for the colored overlay.
`Panorama` displays the original `/panorama/image` when enabled.

The source URDF retains the measured LiDAR mount and starts the four camera
mounts as floating joints. A floating mount does not publish a camera-to-base
TF. The previous manual camera measurements are backed up as comments in the
source xacro and are never supplied to registration. Until a camera is
calibrated its raw cloud has no TF path to `base_link`.

With `initial_pose_confirmation_enabled: true`, the Jetson reruns
FPFH/RANSAC and ICP until a camera has a confirmed pose. It counts estimates from distinct LiDAR and camera
scans independently for each camera. Failed attempts keep that camera's earlier
successful estimates; an estimate outside the agreement limits restarts only
that camera's count. A confirmed pose remains pending until the vehicle accepts
its URDF update. Reusing a provisional ICP result for the next ICP would not be
an independent global estimate. The cameras are processed sequentially, so a
slow RANSAC can delay the other cameras during an attempt. The terminal shows
accepted calibrations and important problems; repeated waiting and skipped-cloud
warnings are limited to one every 30 seconds. Filter timing, RANSAC and ICP metrics, and pose matrices
are available at ROS DEBUG level. Confirmation progress appears when a camera
is waiting for more agreeing estimates. The supplied YAML sets `initial_pose_confirmation_enabled: false`
to use one RANSAC/ICP estimate per camera. Quality and distance checks still
apply, and TF and the URDF update must succeed. A single estimate can be wrong;
the confirmation check helps detect inconsistent matches.

At each configured interval the node uses the last accepted pose to run ICP
again. Updates cannot drift beyond the configured limits from the accepted global pose.
After three consecutive ICP failures or rejected updates, the next attempt
retries FPFH/RANSAC and follows the configured confirmation setting again.
The last accepted mount remains active when a new attempt fails; a changed
cloud frame triggers a new global search. The node uses fresh, time-matched
clouds for registration and computes LiDAR features once per attempt. It also
publishes each fresh display input even when timestamps do not match.
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

According to the configured settings, a camera is skipped if its timestamp is
too far from the LiDAR, either cloud is too old, there are too few points or
matches, a fitness or RMSE limit is not met, the estimated camera origin is too
far from the LiDAR, or the ICP correction is too large. The 3 m distance is a
broad limit for this vehicle, not an initial camera pose; adjust it if a camera
really is farther from the LiDAR. These thresholds are initial settings, not a
guarantee of physical accuracy; a
repetitive scene can still yield a repeatable incorrect match. Check alignment
against multiple real objects before using a calibration. Initial confirmation
requires multiple attempts and can take substantially longer than later ICP.

The Karbon and Jetson clocks must agree, and LiDAR and camera message stamps
must use the same time base. `./g` defaults to `lidar_timestamp_mode:=TIME_FROM_ROS_TIME`,
which uses Karbon packet-reception time for LiDAR headers and affects other
LiDAR consumers, including localization. Pass `lidar_timestamp_mode:=` to keep
the sensor's configured mode; an unsynchronized internal clock can make its
stamps differ even when chrony is synchronized. Direct Ouster launch retains its
configured mode unless given an override. There is no motion compensation between
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
