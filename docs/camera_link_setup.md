# Cameras and the Karbon ↔ Jetson link

How to get the four ZED X cameras (on the Jetson) and the lidar (on the Karbon)
into one ROS 2 graph, and all of them into one RViz window on the Karbon.

> **Status: tested in simulation only.** Every piece below was tested on a
> laptop with containers standing in for the two computers (DDS discovery,
> Wi-Fi isolation, camera-sized images, time sync, the camera launch file with a
> stand-in ZED driver, both RViz layouts). None of it has run on the car yet.

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

## Every time: lidar and cameras in RViz

Three terminals. All on the Karbon; the cameras run on the Jetson through SSH.

**Terminal 1 — the car** (Karbon, as usual):
```bash
cd ~/1811-fall-2026
./scripts/dev.sh
ros2 launch obc_bringup bringup.launch.py lidar_ip:=169.254.148.80 port:=/dev/serial/by-id/$(ls /dev/serial/by-id/ | grep Arduino)
```

**Terminal 2 — the cameras** (on the Jetson):
```bash
ssh nova@192.168.100.2
source ~/1811-fall-2026/ros2_ws/install/local_setup.bash
ros2 launch jetson_bringup zed_cameras.launch.py
```
Each camera logs its serial number as it starts. If the SSH connection drops,
the cameras stop: use `tmux` on the Jetson for long sessions if it's installed.
Start only some cameras with `cameras:=front` or `cameras:=front,rear`.

**Terminal 3 — RViz** (on the Karbon's own screen, with a monitor attached):

Once per Karbon boot, in a normal terminal (not Docker):
```bash
xhost +SI:localuser:root
```
Then:
```bash
cd ~/1811-fall-2026
./scripts/dev.sh
rviz2 -d $(ros2 pkg prefix vehicle_1811_description)/share/vehicle_1811_description/rviz/sensors.rviz
```
You get the car model, the lidar points, and one panel per camera.

### Is the data arriving?

In any Karbon container terminal:
```bash
ros2 topic list | grep zed                                    # camera topics from the Jetson
ros2 topic hz /zed_front/zed_node/rgb/color/rect/image        # ~10 Hz
ros2 topic bw /zed_front/zed_node/rgb/color/rect/image        # ~23 MB/s per camera
```

## Which camera is which

Until serial numbers are set, the cameras are picked by ZED Link port:
`front=0, left=1, rear=2, right=3`. That order is a guess. Check each RViz
panel against where that camera actually points. To fix it:

- **Quick:** swap port ids, e.g. `front_id:=2 rear_id:=0`.
- **Permanent:** use serial numbers (on each camera's sticker, and printed when
  the ZED node starts), e.g.
  `front_serial:=40123456 left_serial:=... rear_serial:=... right_serial:=...`,
  and add them as the defaults in
  [`zed_cameras.launch.py`](../ros2_ws/src/jetson_bringup/launch/zed_cameras.launch.py).

Image resolution, frame rate and depth are in
[`zed_1811.yaml`](../ros2_ws/src/jetson_bringup/config/zed_1811.yaml).

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ros2 topic list` on the Karbon shows no `/zed_...` topics | Domain/DDS mismatch, or the container predates the profile | `check_link.sh` on both; `docker compose up -d --force-recreate dev`; on the Jetson open a new terminal and `ros2 daemon stop` |
| Topics listed but RViz camera panels stay empty | QoS or bandwidth | Panels must be Best Effort (they are in `sensors.rviz`); check `ros2 topic hz`; lower `pub_frame_rate` |
| Image rate well under 10 Hz | Network buffers not raised, or link busy | `check_link.sh` (buffers); `ros2 topic bw`; try fewer cameras with `cameras:=front` |
| `Authorization required` / `could not connect to display` from RViz | Screen permission | `xhost +SI:localuser:root` on the Karbon desktop (see above) |
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
| [`scripts/survey_link.sh`](../scripts/survey_link.sh) | both | Read-only report used to plan this setup |
| [`jetson_bringup`](../ros2_ws/src/jetson_bringup/) | Jetson | `zed_cameras.launch.py` + `config/zed_1811.yaml` |
| [`sensors.rviz`](../ros2_ws/src/vehicle_1811_description/rviz/sensors.rviz) | Karbon | Car model + lidar + 4 camera panels |
