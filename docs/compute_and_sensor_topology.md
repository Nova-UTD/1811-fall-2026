# Compute & Sensor Topology

**Two on-board computers, one Ethernet link between them.** This is the single
most important fact about how 1811's software is physically distributed, and it
is not obvious from the code. Read this before wiring up any node that consumes
both lidar and camera data.

> **Setting it up:** [`camera_link_setup.md`](camera_link_setup.md) has the
> step-by-step setup for the link (IPs, DDS, time sync) and the cameras.
> The Jetson's hostname is `hailbopp-orin` — it was set up from the other car —
> but it is the 1811 Jetson, at `192.168.100.2` on the Karbon's cable.

```
        ┌──────────────────────────────┐        Ethernet        ┌──────────────────────────────┐
        │  KARBON 800   (the "OBC")    │◄──────────────────────►│  JETSON ORIN  (hailbopp-orin)│
        │                              │   one ROS 2 graph       │                              │
        │  • Ouster OS1 lidar (UDP)    │   over DDS              │  • 4× ZED X cameras (GMSL2)  │
        │  • Arduino  (serial/USB)     │                         │    via ZED Link Quad         │
        │  • drives the vehicle        │                         │  • ZED SDK + CUDA            │
        │                              │                         │  • MOST fusion + ROS work    │
        │  docker service: `obc`       │                         │  docker service: `dev`       │
        └──────────────────────────────┘                        └──────────────────────────────┘
```

## Who owns what

| Thing | Lives on | Notes |
|---|---|---|
| **Ouster OS1 lidar** | **Karbon** | UDP lidar/IMU packets land on the Karbon. The `ouster-ros` driver runs here and publishes `/ouster/points`, `/ouster/imu`, and the `os_sensor→os_lidar`/`os_imu` TF. |
| **4× ZED X cameras** | **Jetson** | GMSL2 → ZED Link Quad → ZED SDK. The `zed-ros2-wrapper` node runs **natively** on the Jetson (`~/zed_ws`, not in Docker — needs CUDA/SDK). Publishes `/zed/...`. |
| **Arduino / vehicle actuation** | **Karbon** | Serial JSON at 57600 baud (see README). The serial bridge node runs on the Karbon because that's where `/dev/ttyACM*` is. |
| **Fusion, KISS-ICP, planning, control logic** | **Jetson** | The Jetson is intended to carry most of the ROS/fusion compute. |
| **URDF / `robot_state_publisher`** | **decide — recommend Jetson** | One machine must own the vehicle TF tree (`base_link` and the fixed sensor mounts). Recommend the Jetson as the "main" ROS host. See open questions. |

## Why this matters — consequences of the split

**1. It is one ROS 2 graph spread over two machines.** The Karbon and Jetson
nodes discover each other over the Ethernet link via DDS. For that to work:
- **Same `ROS_DOMAIN_ID` on both** (the compose files set it — keep them identical).
- **Same RMW on both** (`rmw_fastrtps_cpp`, per compose).
- **DDS discovery has to cross the link.** A point-to-point Ethernet link often
  does **not** pass multicast reliably. If nodes on one machine can't see topics
  on the other, don't assume a code bug — configure Fast DDS with **unicast
  peers** (each machine's IP) or run a **Fast DDS Discovery Server**, instead of
  relying on default multicast discovery.
- Note: `ipc: host` (added to fix *cross-container, same-machine* latched-topic
  delivery) does **nothing** across machines — that's a DDS-over-the-wire
  concern, a different layer.

**2. The lidar cloud travels over the wire every scan.** The Jetson's fusion
subscribes to `/ouster/points`, which is published on the Karbon. A full
OS1-128 cloud at 10–20 Hz is real bandwidth — gigabit handles it, but watch it,
and consider the point-cloud profile/rate if the link gets loaded.

**3. Control commands travel the other way.** Planning/control run on the
Jetson, but the Arduino is on the Karbon — so `/vehicle_command` (or
`/vehicle/control`) crosses Jetson → Karbon to the serial bridge. If the link
drops, the Karbon stops getting commands: the Arduino watchdog (README "known
issues") must brake on stale input.

**4. TF spans both machines.** To place a lidar point in a camera frame, fusion
on the Jetson needs the whole chain: `base_link → os_sensor` (URDF /
`robot_state_publisher`, wherever it runs) **and** `os_sensor → os_lidar`
(Ouster driver, on the Karbon). Both must reach the Jetson. `/tf_static` is
latched (`transient_local`), so cross-machine latched delivery matters — same
*class* of gotcha as the cross-container one, but here the fix is DDS
discovery/QoS over the link, not `ipc: host`.

**5. Time sync is now cross-machine and mandatory, not optional.** Lidar points
are stamped with the Karbon's clock; camera frames and any Jetson-side pose are
stamped with the Jetson's clock. If the two clocks disagree, fused output is
wrong in a way that **looks exactly like a calibration error** and will waste
days. Run **PTP or chrony over the Ethernet link** so both machines share a
clock. (The roadmap already flags time sync; the two-machine split is what makes
it non-negotiable.)

## Settled questions

Decided while setting up the cameras (details in
[`camera_link_setup.md`](camera_link_setup.md)):

1. **Which machine runs `robot_state_publisher`?** The **Karbon**, for now — it
   already runs in `bringup`. The Jetson adds only each camera's own frames,
   under the URDF's `zed_<pos>_camera_link`. Moving it to the Jetson can come
   later, when most consumers live there.
2. **DDS discovery over the link:** **static unicast peers**. Both machines use
   Fast DDS, domain 0, and [`config/fastdds_link.xml`](../config/fastdds_link.xml),
   which lists both cable addresses as initial peers and keeps DDS off Wi-Fi.
3. **Static IPs:** Karbon `192.168.100.1` (`enp4s0`), Jetson `192.168.100.2`
   (`eth0`), both set in NetworkManager.
4. **Time-sync master:** the **Karbon** (chrony, synced to the internet); the
   Jetson follows it over the cable.

> Corrections welcome — edit this file as the wiring firms up. If a fact here
> ever conflicts with the code, the code wins and this doc is stale; fix it.
