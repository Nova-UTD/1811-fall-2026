# camera_panorama

`panorama_node` stitches the four ZED X cameras into one 360° image,
`/panorama/image`. It runs on the **Jetson**, next to the cameras, so only the
finished panorama crosses the cable to the Karbon.

> **Status:** tested in simulation (rendered camera images from the real URDF
> mounts, through the real node). Not yet run on the car.

## Run it

Needs the cameras running on the Jetson and `bringup` running on the Karbon
(the URDF places each camera on the car). Then, on the Jetson:

```bash
source ~/1811-fall-2026/ros2_ws/install/local_setup.bash
ros2 launch camera_panorama panorama.launch.py
```

It logs `Panorama ready: 2048x316 from front, left, rear, right, coverage 100%`
once every camera's image, `camera_info` and TF have arrived. In RViz on the
Karbon, `sensors.rviz` has a **Panorama** panel.

Build it on the Jetson once (with the camera workspace sourced):

```bash
cd ~/1811-fall-2026/ros2_ws
colcon build --symlink-install --packages-select camera_panorama
```

## What you see

- Left to right: **straight back → left → forward (centre) → right → back**,
  as if you turned around inside the car.
- Vertically: from 35° below the horizon to 15° above (`elev_min_deg`,
  `elev_max_deg`). All four cameras cover that band.
- **Seams line up for things about 8 m away** (`stitch_distance`). Closer
  objects, especially within a couple of metres, can look doubled or cut at the
  four corners. That's parallax — the cameras sit in different places — and it
  happens in every multi-camera surround view.
- Brightness differences between cameras are evened out at the seams
  (`exposure_compensation`), updated once a second.

## How it works

The cameras are fixed to the car, so the mapping from each panorama pixel to
the camera pixel(s) it comes from never changes. At startup the node takes each
camera's lens calibration (`camera_info`) and its position on the car (TF from
`base_link` to the image's frame), works out that mapping once, and after that
each frame is a lookup and a feathered blend. With OpenCV (installed on the
Jetson) that takes ~36 ms per frame on a laptop; numpy alone ~130 ms.

The maths is in [`panorama_core.py`](camera_panorama/panorama_core.py) with no
ROS imports, unit-tested in [`test/test_panorama_core.py`](test/test_panorama_core.py).

## Lining up the seams

The camera positions come from the URDF, which is tape-measured. If a seam
looks offset, nudge that camera's angles in
[`config/panorama.yaml`](config/panorama.yaml):

```yaml
    rear_correction_rpy_deg: [0.0, 0.5, -1.0]   # roll, pitch (+ = nose down), yaw (+ = pan left)
```

then restart the node. Use a scene with things about `stitch_distance` away.
Fix the URDF too once a correction is confirmed, so lidar–camera work gets the
same numbers.

## Settings

All in [`config/panorama.yaml`](config/panorama.yaml):

| Parameter | Default | Meaning |
|---|---|---|
| `width` | 2048 | Pixels for the full 360° (height follows from the elevation range) |
| `elev_min_deg` / `elev_max_deg` | -35 / 15 | Vertical range |
| `stitch_distance` | 8.0 m | Distance at which seams line up |
| `rate` | 10 Hz | Output rate |
| `max_age` | 0.5 s | Older camera images count as missing (that area goes black) |
| `exposure_compensation` | true | Even out brightness between cameras |
| `<camera>_correction_rpy_deg` | 0, 0, 0 | Mounting fixes, see above |
| `use_opencv` | true | Faster warping when OpenCV is available |
