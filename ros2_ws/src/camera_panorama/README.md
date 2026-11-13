# camera_panorama

`panorama_node` stitches the four ZED X cameras into one 360° image,
`/panorama/image`. It runs on the **Jetson**, next to the cameras, so only the
finished panorama crosses the cable to the Karbon.

> **Status (November 2026):** running on the car since October, all four cameras
> stitched on the Jetson and shown in `sensors.rviz` on the Karbon. The frame
> rate was fixed off the car, so it is not yet confirmed on it: it was well under
> 10 Hz, for three reasons ([Frame rate](#frame-rate)). Camera positions now
> come from the LiDAR-camera calibration, so the seams need a fresh check
> ([Next steps](#next-steps)).

## Run it

Usually from the Karbon, together with the car and the cameras:

```bash
ros2 launch obc_bringup bringup.launch.py lidar_ip:=... port:=...
```

The panorama starts by default when cameras are enabled; pass `panorama:=false`
to skip it. The `./g` shortcut behaves the same way.

(See [`docs/camera_link_setup.md`](../../../docs/camera_link_setup.md#every-time-lidar-and-cameras-in-rviz).)
Or by hand on the Jetson, with the cameras running and `bringup` running on the
Karbon (the URDF places each camera on the car):

```bash
source ~/1811-fall-2026/ros2_ws/install/local_setup.bash
ros2 launch camera_panorama panorama.launch.py
```

It logs `Panorama ready: 2048x316 from front, left, rear, right, coverage 100%`
once every camera's image, `camera_info` and TF have arrived. In RViz on the
Karbon, the **Panorama** panel shows the original `/panorama/image`.

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
each frame is a lookup and a feathered blend, over only the part of the
panorama each camera covers. With OpenCV (installed on the Jetson) a frame takes
~20 ms on a laptop; numpy alone ~220 ms. If the calibration moves a camera by
more than 0.2° or 1 cm while it runs, the mapping is rebuilt (`Camera poses
changed ... rebuilding the panorama`).

The maths is in [`panorama_core.py`](camera_panorama/panorama_core.py) with no
ROS imports, unit-tested in [`test/test_panorama_core.py`](test/test_panorama_core.py).

## Frame rate

Every 30 s the node logs how it is doing (it also shows in bringup's terminal):

```
Panorama running: 10.0 Hz, 20 ms per frame; camera frames in: front 10.0, left 10.0, rear 10.0, right 10.0 Hz
```

- **ms per frame** is the panorama's own work. It must stay under 100 ms for 10 Hz.
- **camera frames in** is what actually reaches the node. If these are well
  under 10, frames are being lost on their way to the panorama (see the first
  troubleshooting row), and the panorama can't be faster than its cameras.
- It only publishes when a camera has sent a new frame, so it never repeats an
  old panorama to look faster.

What used to hold it back, measured on the `pano_test` recording (November 2026):

| Cause | Cost before | After |
|---|---|---|
| Filling the outgoing ROS message from `bytes` (ROS checks every pixel in Python) | ~150 ms per frame on a laptop | 0.1 ms (`array.array`) |
| Fast DDS's 512 KB shared-memory segment, too small for a 2.3 MB camera image | cameras reached the node at 2–5 Hz | 10 Hz (16 MB segment in `config/fastdds_link.xml`) |
| Warping each camera over the whole 360°, slow image conversion and brightness matching | ~125 ms per frame, plus 200 ms once a second | ~30 ms, plus 13 ms once a second |

## Lining up the seams

The camera positions come from the URDF, which the LiDAR-camera calibration
fills in on the Jetson (`lidar_perception`). Seams can still be slightly off.
Each camera's angles can be nudged **while the panorama runs**, on top of
whatever the calibration placed; it rebuilds on the next frame.

**Where the seams are** (left to right across the panorama):

| Position across the image | Seam between |
|---|---|
| ~1/8 | rear camera / left camera (rear-left corner) |
| ~3/8 | left camera / front camera (front-left corner) |
| ~5/8 | front camera / right camera (front-right corner) |
| ~7/8 | right camera / rear camera (rear-right corner) |

**Look at things a few metres or more away** (building edges, poles, door
frames, the horizon). Anything within ~2 m will always double at a seam
(parallax) and can't be tuned away.

**Keep the front camera as the reference**, fix left and right against it, then
fix the rear against left and right. On the Jetson, in another SSH session:

```bash
ros2 param set /panorama_node left_correction_rpy_deg "[0.0, 0.0, 1.0]"
```

The three numbers are **[roll, pitch, yaw]** in degrees, in the camera's own
frame. One degree moves that camera's part of the panorama about 6 pixels.

| What you see at a seam (compared with the neighbour) | Change on that camera |
|---|---|
| Its part is shifted **right** | **yaw +** (e.g. 1.0) |
| Its part is shifted **left** | **yaw −** |
| Its part is too **high** | **pitch +** |
| Its part is too **low** | **pitch −** |
| Vertical edges lean (the step changes from top to bottom) | **roll** ± |

Change one camera, one number, by about 1° at a time. Each `set` replaces all
three numbers for that camera, so keep the ones you've already tuned.

**Saving them:** once the seams look right, print the values:

```bash
for c in front left rear right; do echo -n "$c: "; ros2 param get /panorama_node ${c}_correction_rpy_deg; done
```

Put them in [`config/panorama.yaml`](config/panorama.yaml) (they're lost when
the node restarts otherwise) and, once confirmed, in the URDF camera mounts too,
so lidar–camera work uses the same numbers.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Panorama running: ...; camera frames in:` well under 10 Hz | A process started with a copy of `config/fastdds_link.xml` from before the 16 MB shared-memory segment | `git pull` on **both** machines, `ros2 daemon stop`, then restart bringup (which restarts the Jetson side) and RViz |
| `no TF base_link -> ...` for a camera, `(... has camera calibration placed it?)` | The camera mounts in the URDF are placeholders until the LiDAR-camera calibration accepts a pose for that camera | Run bringup with the registration (the default) and wait for `Calibrated ZED cameras`; the panorama adds each camera as soon as it is placed |
| `Panorama waiting: ... no image or camera_info yet` for every camera | The cameras aren't running | Start `jetson_bringup zed_cameras.launch.py` first and wait for four `started` lines |
| Only some cameras are used (e.g. `from front, coverage 29%`) and the rest never arrive | Processes started with an old copy of `config/fastdds_link.xml` (before it raised `maxInitialPeersRange`), so only the first 4 ROS processes per machine were discovered | `git pull` on **both** machines, `ros2 daemon stop`, then restart bringup, the cameras, the panorama and RViz |
| `no TF base_link -> zed_<pos>_left_camera_frame_optical` | Bringup isn't running on the Karbon (it publishes the URDF), or the same discovery problem as above | Start bringup; then as above |
| `A module that was compiled using NumPy 1.x cannot be run in NumPy 2` and `numpy warping` | The Jetson has a pip-installed NumPy 2 in `~/.local` that breaks OpenCV | Handled by the launch file (`PYTHONNOUSERSITE=1`). If you run `panorama_node` directly, prefix it with `PYTHONNOUSERSITE=1` |

## Next steps

1. **Confirm 10 Hz on the car:** the `Panorama running` line in bringup's
   terminal, and `ros2 topic hz /panorama/image` on the Karbon.
2. **Check the seams with the calibrated camera poses** at the four corners on
   things a few metres or more away. The October `pano_test` recording was made
   with the old tape-measured mounts; against those, the seams wanted mainly
   tilt fixes (rear about 2.3° and right about 1.8° less tilted than measured,
   left about 0.6° more). Those numbers don't apply on top of the calibration.
3. **Record 10 s of the four cameras** after `Calibrated ZED cameras` appears
   (on the Jetson, cameras running, car parked with things around it at
   different distances, ideally some far away: open the garage door or go
   outside, since nearby objects can't tell a tilted mount from a misplaced one):
   ```bash
   cd ~
   timeout -s INT 12 ros2 bag record -o pano_test /zed_front/zed_node/rgb/color/rect/image /zed_left/zed_node/rgb/color/rect/image /zed_rear/zed_node/rgb/color/rect/image /zed_right/zed_node/rgb/color/rect/image /zed_front/zed_node/rgb/color/rect/camera_info /zed_left/zed_node/rgb/color/rect/camera_info /zed_rear/zed_node/rgb/color/rect/camera_info /zed_right/zed_node/rgb/color/rect/camera_info /tf_static
   du -sh ~/pano_test
   ```
   Used to refine each camera's mounting angles (`<camera>_correction_rpy_deg`)
   off the car.

## Settings

All in [`config/panorama.yaml`](config/panorama.yaml):

| Parameter | Default | Meaning |
|---|---|---|
| `width` | 2048 | Pixels for the full 360° (height follows from the elevation range) |
| `elev_min_deg` / `elev_max_deg` | -35 / 15 | Vertical range |
| `stitch_distance` | 8.0 m | Distance at which seams line up |
| `rate` | 10 Hz | Highest output rate; it only publishes when a camera has a new frame |
| `stats_interval` | 30 s | How often it logs `Panorama running: ... Hz` (0 = never) |
| `max_age` | 0.5 s | Older camera images count as missing (that area goes black) |
| `exposure_compensation` | true | Even out brightness between cameras |
| `<camera>_correction_rpy_deg` | 0, 0, 0 | Mounting fixes, see above |
| `use_opencv` | true | Faster warping when OpenCV is available |
