# camera_panorama

`panorama_node` stitches the four ZED X cameras into one 360° image,
`/panorama/image`. It runs on the **Jetson**, next to the cameras, so only the
finished panorama crosses the cable to the Karbon.

> **Status: running on the car (October 2026)** -- all four cameras stitched on
> the Jetson with OpenCV, shown in `sensors.rviz` on the Karbon. Seams not yet
> checked or fine-tuned: see [Next steps](#next-steps).

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

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Panorama waiting: ... no image or camera_info yet` for every camera | The cameras aren't running | Start `jetson_bringup zed_cameras.launch.py` first and wait for four `started` lines |
| Only some cameras are used (e.g. `from front, coverage 29%`) and the rest never arrive | Processes started with an old copy of `config/fastdds_link.xml` (before it raised `maxInitialPeersRange`), so only the first 4 ROS processes per machine were discovered | `git pull` on **both** machines, `ros2 daemon stop`, then restart bringup, the cameras, the panorama and RViz |
| `no TF base_link -> zed_<pos>_left_camera_frame_optical` | Bringup isn't running on the Karbon (it publishes the URDF), or the same discovery problem as above | Start bringup; then as above |
| `A module that was compiled using NumPy 1.x cannot be run in NumPy 2` and `numpy warping` | The Jetson has a pip-installed NumPy 2 in `~/.local` that breaks OpenCV | Handled by the launch file (`PYTHONNOUSERSITE=1`). If you run `panorama_node` directly, prefix it with `PYTHONNOUSERSITE=1` |

## Next steps

1. **Check the seams** at the four corners (front-left, front-right, rear-left,
   rear-right) on things a few metres or more away: lined up, slightly
   offset (which corner, which direction), or badly off.
2. **Record 10 s of the four cameras** (on the Jetson, cameras running, car
   parked with things around it at different distances):
   ```bash
   cd ~
   timeout -s INT 12 ros2 bag record -o pano_test /zed_front/zed_node/rgb/color/rect/image /zed_left/zed_node/rgb/color/rect/image /zed_rear/zed_node/rgb/color/rect/image /zed_right/zed_node/rgb/color/rect/image /zed_front/zed_node/rgb/color/rect/camera_info /zed_left/zed_node/rgb/color/rect/camera_info /zed_rear/zed_node/rgb/color/rect/camera_info /zed_right/zed_node/rgb/color/rect/camera_info /tf_static
   du -sh ~/pano_test
   ```
   Used to refine each camera's mounting angles (`<camera>_correction_rpy_deg`)
   off the car.
3. **Then:** project the lidar points onto the camera images (lidar-camera
   fusion), reusing the refined camera angles.

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
