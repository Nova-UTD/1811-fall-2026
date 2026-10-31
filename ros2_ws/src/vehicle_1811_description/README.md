# vehicle_1811_description

URDF/xacro for the 1811 Nova sub-scale vehicle: `base_link`, an Ouster
**OS1-070-128U-AX** lidar (`os_sensor`), **four ZED X** cameras in a 360° ring,
and four wheels. This is roadmap **Section 5, step 1** — the one *blocking* step.

## Frames

```
base_footprint
└─ base_link                      (rear axle, on the ground; x fwd, y left, z up)
   ├─ wheel_{front,rear}_{left,right}
   ├─ os_sensor                   (ouster-ros adds os_lidar, os_imu at runtime)
   └─ zed_{front,rear,left,right}_camera_link
        └─ zed_{...}_optical_frame  (z out of lens — used for projection math)
```

## The only thing that matters here: measurements

Every value tagged `# TODO-MEASURE` in `urdf/vehicle_1811.urdf.xacro` is a
placeholder. Take a tape measure to the real vehicle and edit them. Priority
order (worst-to-get-wrong first):

1. `mast_x, mast_y, mast_top_z` and the lidar mount `rpy` — lidar→base_link.
   Wrong here → KISS-ICP builds a self-consistent map rotated vs. the car, and
   *nothing downstream notices*.
2. `wheel_base`, `track_width` — also feed pure-pursuit / RTP tuning.
3. Camera ring `ring_r, cam_z, cam_pitch` and per-camera yaw — cameras are
   phase two, so rough is fine for now.

All units SI: **metres and radians**, `x` forward, `y` left, `z` up (REP-103).

## Use

```bash
# in ros2_ws/, after colcon build + source
ros2 launch vehicle_1811_description display.launch.py      # RViz + joint sliders
```

Done-when (roadmap): start the Ouster driver too, add a PointCloud2 on
`/ouster/points`, and confirm the cloud sits correctly against the model.
For the whole-stack TF tree, `description.launch.py` is the include-me version.

## "Automatic URDF creator" — where calibration actually fits

The tool you heard about is [ros2_calib](https://github.com/ika-rwth-aachen/ros2_calib)
(multi-sensor calibration from an mcap recording, with URDF export). It — and
checkerboard lidar↔camera calibration generally — solves a *different, later*
problem than this file:

- **This URDF (now):** rough tape-measure extrinsics, good enough for RViz to
  place the point cloud and for the lidar-only teach-and-repeat milestone. Your
  roadmap says it explicitly: *"tape measure, not a calibration rig."*
- **Checkerboard / ros2_calib (phase two):** precise camera intrinsics and
  lidar↔camera extrinsics, needed once you actually *fuse* camera and lidar
  (drivable surface, projecting lidar into the image, etc.). At that point you
  replace the `zed_*` mount numbers here with calibrated ones and commit them.

So: hand-write it now, calibrate-and-refine later. Camera **intrinsics** come
free from the ZED factory calibration via `camera_info` — don't retype those.
