# Cameras and the Karbon ↔ Jetson link

How to get the four ZED X cameras (on the Jetson) and the lidar (on the Karbon)
into one ROS 2 graph, and all of them into one RViz window on the Karbon.

> **Status: working on the car (October 2026).** Both setup scripts ran on the
> 1811 Karbon and Jetson, DDS works across the cable, the Jetson follows the
> Karbon's clock (it was 95 ms off before), and the lidar plus all four cameras
> show together in `sensors.rviz` on the Karbon, each camera at about 10 Hz.
>
> **Never plug or unplug a camera while the Jetson is on.** GMSL2 cameras are
> not hot-pluggable: it can damage a camera or the ZED Link board, and the
> camera software only detects cameras at boot. Shut the Jetson down
> (`sudo shutdown now`), change cables, then power it back on.

## The setup at a glance

```
 KARBON  (karby)                                   JETSON ORIN  (hostname: hailbopp-orin)
 enp4s0  192.168.100.1  ◄──── 2.5 GbE cable ────►  eth0  192.168.100.2
 ROS 2 Humble in Docker                            ROS 2 Humble native + ZED SDK 5.2.3
 lidar, Arduino, bringup, RViz                     4× ZED X → zed-ros2-wrapper v5.2.2 (~/zed_ws)
 time server (chrony)                              follows the Karbon's clock
```

The Jetson's hostname is `hailbopp-orin` (it was set up from the other car), but
it is the 1811 Jetson: the one on the other end of the Karbon's cable.

| Decision | Choice | Why |
|---|---|---|
| IP addresses | Karbon `192.168.100.1` (`enp4s0`, "Orin Connection"), Jetson `192.168.100.2` (`eth0`, "cable") | Already set up on both machines |
| ROS domain / DDS | Domain **0**, **Fast DDS**, profile [`config/fastdds_link.xml`](../config/fastdds_link.xml) on both | The Jetson changes to match the Karbon, so the tested driving stack stays as it is |
| What the profile does | Keeps ROS traffic on the cable (and same-machine traffic), with direct discovery between `.1` and `.2` | Both computers are on CometNet Wi-Fi, as is the other car; without it the graphs leak onto Wi-Fi and can mix |
| Time | **Karbon is the time server** (chrony, synced to the internet); the Jetson follows it | Lidar stamps come from the Karbon; camera stamps must agree with them |
| Vehicle TF (`robot_state_publisher`) | Stays on the **Karbon** (in `bringup`) | Already working; the Jetson only adds each camera's own frames under it |
| Camera frames | Cameras are named `zed_front/left/rear/right`, so each ZED's TF tree starts at the URDF's `zed_<pos>_camera_link` | No duplicate frames (see the note in `vehicle_1811.macros.xacro`) |
| Camera load | Depth and tracking off, images 960×600 at 10 Hz | Images only for now; ≈0.75 Gb/s for all four on the 2.5 Gb cable |

## One-time setup

Do these once, in order. Each step says what success looks like.

### 1. Get the branch onto both machines

**Karbon** (outside Docker):
```bash
cd ~/1811-fall-2026
git fetch
git switch feature/360camera/aaroh
```

**Jetson**: SSH in from the Karbon (`ssh nova@192.168.100.2`), then clone the repo
(needs your GitHub username and a personal access token):
```bash
git clone https://github.com/Nova-UTD/1811-fall-2026.git ~/1811-fall-2026
cd ~/1811-fall-2026
git switch feature/360camera/aaroh
```

### 2. Karbon setup

On the Karbon (outside Docker):
```bash
bash scripts/setup_link_karbon.sh
```
It lists its changes and asks before doing anything:
- a firewall rule letting in traffic from the Jetson on `enp4s0` (and nothing else),
- larger network buffers (`/etc/sysctl.d/99-zz-1811-dds.conf`),
- chrony, serving time to the cable.

Then recreate the Docker container so it picks up the DDS profile, and rebuild
(the new RViz layout has to be installed):
```bash
docker compose up -d --force-recreate dev
./scripts/dev.sh
colcon build --symlink-install --packages-ignore umrr_ros2_driver umrr_ros2_msgs smart_rviz_plugin
```

### 3. Jetson setup

On the Jetson:
```bash
cd ~/1811-fall-2026
bash scripts/setup_link_jetson.sh
```
It lists its changes and asks first:
- `~/.bashrc`: `ROS_DOMAIN_ID` 1 → 0, `RMW_IMPLEMENTATION` Cyclone → Fast DDS, and
  the profile path (old file kept as `~/.bashrc.bak-1811`),
- the same network buffers,
- chrony, following the Karbon,
- builds `jetson_bringup` (the camera launch) against `~/zed_ws`.

Then open a new terminal (or `source ~/.bashrc`) and run `ros2 daemon stop`.

> Changing the Jetson's domain and DDS affects anything else that runs on it.
> If someone uses this Jetson for other work, tell them. To undo:
> `cp ~/.bashrc.bak-1811 ~/.bashrc`.

### 4. Check both machines

On each machine (outside Docker):
```bash
bash scripts/check_link.sh
```
Every line should say `PASS`. Each `FAIL` says what to do. The two network-buffer
and time checks need the setup scripts to have run on that machine.

### 5. Let the Karbon start the cameras

So that one command on the Karbon starts the car **and** the Jetson's cameras.
On the Karbon (outside Docker):
```bash
cd ~/1811-fall-2026
git pull
bash scripts/setup_jetson_ssh.sh
```
It makes a key just for this (`~/.ssh/id_ed25519_jetson`; your other keys are
untouched) and installs it on the Jetson, which asks for the Jetson's password
once. It ends with `OK: passwordless login ... works`. The Jetson needs a Git
checkout at `~/1811-fall-2026`; `./g` updates an older checkout automatically.

Back on the Karbon, close other container terminals. The first `./g` below
starts the container with the key available and builds the workspace.

## Every time: lidar and cameras in RViz

One terminal on the Karbon starts the vehicle, the Jetson cameras, and RViz.
After the initial setup, you do not need to log in to the Jetson each time.
`./g` fetches the Karbon's exact committed revision on the Jetson if needed,
switches to its branch when safe, and incrementally builds the camera and
detector packages. The commit must be pushed and the Jetson needs Git access
to origin when it does not already have that commit. Local Jetson edits or
diverged commits block an update, as do running camera processes when the
checkout needs to change; the car stack continues and the error is printed.
Uncommitted Jetson source changes on the
Karbon also block Jetson startup so they are not silently left behind.

**The car, cameras, panorama and RViz** (Karbon host):
```bash
cd ~/1811-fall-2026
./g
```
Use the **real lidar IP**. With `lidar_ip:=0.0.0.0` the car still drives, but
there are no lidar points or lidar images in RViz. If the lidar has picked a
new address, `ping -c 2 os-122316000219.local` on the Karbon shows it. Pass
that address with `./g lidar_ip:=<sensor-ip>`.

The Jetson's lines start with `[jetson_cameras-1]`: each camera logs its serial
number and `=== zed_<position> started ===`, then `Panorama ready`. Full logs
stay on the Jetson in `~/.ros/jetson_cameras/` (`cameras_latest.log`,
`panorama_latest.log`).

| Option | Does |
|---|---|
| `panorama:=false` | Skips the 360° panorama |
| `rviz:=false` | Starts without RViz |
| `cameras:=front` or `cameras:=front,rear` | Starts only those cameras |
| `cameras:=false` | Skips the cameras; Jetson edge detection still runs by default |
| `corners_on_jetson:=false` | Skips Jetson LiDAR and panorama edge detection |
| `jetson_update:=false` | Uses the existing Jetson checkout without updating or preparing it |

Ctrl-C stops the car and the cameras together (`[jetson] stopped.`). If the
Jetson is off or can't be reached, the car still starts and the terminal says
`Jetson cameras did not start ... the car is fine`, and what to check.

**Starting the cameras by hand instead** (for example without the car): run
bringup with `cameras:=false`, or not at all, and on the Jetson:
```bash
ssh nova@192.168.100.2
bash ~/1811-fall-2026/scripts/jetson_cameras.sh --panorama
```
If the cameras are already running when bringup starts, bringup leaves them
alone (`cameras already running (started elsewhere)`).

`./g` requests X11 access and opens RViz on the Karbon's screen. You get the car
model and black lidar points with neon-blue patch-edge voxel centers,
and one panel per camera. To see raw lidar instead, untick
`Lidar edges` and tick `Lidar` in the Displays panel; raw lidar is still
available if detection is disabled. `Panorama` shows the stitched image with
neon-blue edges; enable `Panorama original` for the unmodified image, or when
edge detection is disabled.

### Is the data arriving?

In any Karbon container terminal:
```bash
ros2 topic list | grep zed                                    # camera topics from the Jetson
ros2 topic hz /zed_front/zed_node/rgb/color/rect/image        # ~10 Hz
ros2 topic bw /zed_front/zed_node/rgb/color/rect/image        # ~23 MB/s per camera
```

## Which camera is which

The cameras are picked by ZED Link port: `front=0, left=1, rear=2, right=3`.
**This order was checked on the car and matches the cable wiring** (port 0 is
the front camera, S/N 41230694). It only changes if the cables are moved
between ports. If that happens, check each RViz panel against where its camera
points and either:

- **Quick:** swap port ids on the command line, e.g. `front_id:=2 rear_id:=0`.
- **Permanent:** pin cameras by serial number (on each camera's sticker, and
  printed as `Serial Number -> ...` when the ZED node starts), e.g.
  `front_serial:=41230694 left_serial:=... rear_serial:=... right_serial:=...`,
  and add them as the defaults in
  [`zed_cameras.launch.py`](../ros2_ws/src/jetson_bringup/launch/zed_cameras.launch.py).

Image resolution, frame rate and depth are in
[`zed_1811.yaml`](../ros2_ws/src/jetson_bringup/config/zed_1811.yaml).

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ros2 topic list` on the Karbon shows no `/zed_...` topics | Domain/DDS mismatch, or the container predates the profile | `check_link.sh` on both; `docker compose up -d --force-recreate dev`; on the Jetson open a new terminal and `ros2 daemon stop` |
| Topics listed but RViz camera panels stay empty | QoS or bandwidth | Panels must be Best Effort (they are in `sensors.rviz`); check `ros2 topic hz`; lower `pub_frame_rate` |
| Some nodes find each other and others never do (e.g. the panorama only sees the front camera) | A copy of `config/fastdds_link.xml` from before `maxInitialPeersRange` was raised: Fast DDS then only discovers the first 4 ROS processes on each machine | Update the Karbon, run `ros2 daemon stop` on both machines, then restart `./g` |
| `ros2 topic list` shows a topic, but `ros2 topic hz` says it "does not appear to be published" | The topic list is **cached** by the ROS daemon and can keep topics from earlier runs | `ros2 daemon stop`, then `ros2 topic info <topic>`: `Publisher count: 0` means it really isn't running |
| No lidar points or lidar images in RViz | Bringup started with `lidar_ip:=0.0.0.0`, or the lidar is off | Restart `./g` with `lidar_ip:=169.254.148.80`; `ros2 topic info /ouster/points` should show `Publisher count: 1` |
| Cameras stopped showing after cables were moved | Cameras were re-plugged with the Jetson on | Shut the Jetson down, check the cables, power it on, relaunch the cameras |
| RViz prints `Stereo is NOT SUPPORTED` | Nothing wrong: it's about stereoscopic 3D-glasses displays, not the stereo cameras | Ignore it |
| Image rate well under 10 Hz | Network buffers not raised, or link busy | `check_link.sh` (buffers); `ros2 topic bw`; try fewer cameras with `cameras:=front` |
| `Authorization required` / `could not connect to display` from RViz | Screen permission | `xhost +SI:localuser:root` on the Karbon desktop (see above) |
| `Jetson cameras did not start (exit 255)` | Jetson off or not booted yet, or the key isn't set up / the container can't see it | `ping -c 2 192.168.100.2`; run [step 5](#5-let-the-karbon-start-the-cameras) again; restart `./g` |
| `Jetson checkout has local changes` or `branch has diverged or is ahead` | The update would interfere with existing Jetson work | Preserve and resolve that work on the Jetson, then restart `./g` |
| `Git fetch failed` or the Karbon commit is unavailable | The commit isn't pushed, or Jetson cannot reach or authenticate to origin | Push the Karbon commit and check the Jetson's network and Git credentials |
| `Jetson cameras not started: rebuild first` | Pulled on the Karbon but didn't rebuild | `colcon build --symlink-install --packages-ignore umrr_ros2_driver umrr_ros2_msgs smart_rviz_plugin` |
| A camera fails to open | Wrong port id/serial, or two instances asking for the same camera | Start one at a time (`cameras:=front`) and check its serial in the log |
| Camera images look delayed against lidar | Clocks apart | `check_link.sh` on the Jetson: "following the Karbon's clock" must PASS |
| The other car's topics show up | Profile not active on that machine | `printenv FASTRTPS_DEFAULT_PROFILES_FILE` should print the profile path |

## Files

| File | Runs on | Purpose |
|---|---|---|
| [`config/fastdds_link.xml`](../config/fastdds_link.xml) | both | DDS profile: cable only, direct discovery, bigger buffers |
| [`docker-compose.yml`](../docker-compose.yml) | Karbon | Sets `FASTRTPS_DEFAULT_PROFILES_FILE` for every container |
| [`scripts/setup_link_karbon.sh`](../scripts/setup_link_karbon.sh) | Karbon | Firewall rule, buffers, chrony server |
| [`scripts/setup_link_jetson.sh`](../scripts/setup_link_jetson.sh) | Jetson | `~/.bashrc` ROS settings, buffers, chrony client, build |
| [`scripts/check_link.sh`](../scripts/check_link.sh) | both | Read-only PASS/FAIL checks |
| [`scripts/setup_jetson_ssh.sh`](../scripts/setup_jetson_ssh.sh) | Karbon | One-time key so bringup can start the cameras |
| [`scripts/jetson_cameras.sh`](../scripts/jetson_cameras.sh) | Jetson | Starts the cameras (and panorama); bringup runs it over SSH |
| [`jetson_cameras_ssh.sh`](../ros2_ws/src/obc_bringup/scripts/jetson_cameras_ssh.sh) | Karbon | Bringup's SSH side: starts the script above, stops it cleanly on Ctrl-C |
| [`scripts/survey_link.sh`](../scripts/survey_link.sh) | both | Read-only report used to plan this setup |
| [`jetson_bringup`](../ros2_ws/src/jetson_bringup/) | Jetson | `zed_cameras.launch.py` + `config/zed_1811.yaml` |
| [`sensors.rviz`](../ros2_ws/src/vehicle_1811_description/rviz/sensors.rviz) | Karbon | Car model + lidar + 4 camera panels |
