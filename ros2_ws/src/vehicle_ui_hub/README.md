# 1811 Vehicle UI Hub

The hub uses PyQt5 and `rclpy`, with a small native bridge for the embedded
RViz 3D view. It is independent of the LiDAR-camera registration implementation.
The vehicle container supplies ROS Humble and Qt 5, so PyQt and RViz use the
same Qt version. A build whose RViz uses a different Qt major version is rejected
before the bridge loads.

On the Karbon, start the full stack and the hub with:

```bash
cd ~/1811-fall-2026
./g
```

With a desktop display, this opens the hub full screen and starts registration
in interactive mode on the Jetson. Use the full-screen icon, F11, or Escape to
leave full screen. Use the icon or F11 to return. To start in a window when
launching the hub executable directly, pass `--windowed`.

Choose **Black** or **White** under **DevTools → Theme**. The choice is saved
across hub and container restarts. Black uses charcoal panels and bright primary
actions; White uses dark primary actions for contrast. The 3D view stays black
in both themes so the existing point-cloud colors remain easy to distinguish.

Tool panels slide between views. Repeated taps redirect the current transition.
The native RViz window stays in place during navigation to reduce flicker.
Camera subscriptions still stop when leaving Surround; the last frames remain
visible only until the outgoing panel finishes sliding away.
Swipe inside a panel to scroll; sliders, text fields, and camera drag handles
keep their usual touch gestures.
Calibration and tool messages fade upward as they change. Rapid updates keep
the latest message, while live rates and numeric readings update directly.

Choose **DevTools → Idle** to leave the hub on a mostly black screen. The hub
releases its camera, point-cloud, TF, calibration, and vehicle subscriptions,
pauses RViz rendering and network checks, and stops its executor and periodic
UI refreshes. The idle screen fades in and out without moving the 3D viewport.
Tap **Wake up**, then **Confirm wake**, to restore the same page and subscriptions.
Stop an active bag recording and wait for calibration requests to finish before
entering idle; these are not interrupted automatically.
The LiDAR, cameras, calibration, and other processes in the vehicle stack
continue running. Idle does not put the computer to sleep or turn off the display.

When host PyQt5 or an existing vehicle image is available, the startup screen
opens as `./g` begins preparing Docker and building the workspace. On a first
installation without either, it opens after the image is built. It closes once
RViz renders and the hub receives a recent `/ouster/points` cloud plus an image
from each enabled ZED camera. Disabled cameras are not required. Receiving data
does not verify sensor health or calibration. If a sensor is unavailable, the
screen shows what is missing; after 15 seconds **Open hub** lets you inspect
the interface without waiting. If the hub starts but inputs or the 3D view are
still unavailable after 30 seconds, it opens automatically and identifies what
was missing. This does not mark the sensors as ready. If the hub cannot start
at all, the early screen closes after a further timeout and reveals the launch
output. This timeout starts after the workspace build, not during it. Closing
the launcher also closes the early screen.

`./g hub:=false` uses the separate RViz window and automatic registration;
`./g hub:=false rviz:=false` starts the full stack without a graphical window.
`./g rviz:=false` also starts without a graphical window. With no `DISPLAY`, `./g`
starts headless and uses automatic registration. An explicit `hub:=true` still
requests the hub even without a display. The shortcut syncs the committed,
pushed Karbon revision to the Jetson; local uncommitted Jetson source changes
are not copied. The Jetson checkout must also be clean for that update.

The overview shows the full `/ouster/points` cloud and ZED cloud previews,
with a separate color and layer toggle for each sensor. The initial view looks
down from behind the vehicle. Drag with the
left mouse button or one finger to orbit the scene. Use the scroll wheel or a
two-finger pinch to zoom. The camera stays centered and zoom-out is limited to
18 meters from its focal point.
Before a camera has an alignment estimate its cloud cannot be placed in the
LiDAR frame and is not drawn; it appears as soon as a valid estimate is available.
An absent camera or disabled depth stream remains unavailable.
For normal display the Jetson sends approximately one quarter of each ZED
cloud's points. When a camera is selected for calibration review, its preview
uses the full cloud so the operator can inspect alignment. This sampling does
not change the original camera topics or the registration inputs. The LiDAR
display still uses the full input cloud.
RViz targets 30 frames per second; the
achieved rate depends on the graphics hardware and the point-cloud data displayed.
This limits the display update rate, not the sensor or localization rates.
The Jetson sends each camera-cloud preview at most twice per second while
its pose is unchanged. A changed pose or newly selected review is sent promptly;
the original camera topics and calibration inputs are not throttled.

The rate below each cloud layer is measured by registration on the Jetson from
its existing sensor subscriptions. Camera tiles show the message rate received
by the hub before its display throttle. These are observed receive rates, not a
guarantee of publisher frequency; dropped messages or load can lower them.
Rates disappear when data becomes stale.

The **Surround view** camera panel also subscribes to the four live ZED image
topics `/zed_<front|left|rear|right>/zed_node/rgb/color/rect/image` and to
`/panorama/image`. `./g` starts the independent panorama producer on the Jetson
by default; pass `panorama:=false` to disable it. A tile says
"Waiting" until a frame arrives and clears stale frames after three seconds.
The panel displays at most 10 frames per second per stream, deserializing only
frames selected for display. The ROS image topics can still carry their full
publishing rate and resolution over the network.
Tap a camera or panorama tile to expand it. To swap a tile with the main view,
drag its six-dot handle onto the main view. Drag the point-cloud handle back to
restore the 3D view. Navigating to another page unsubscribes all five image
topics. Other subscribers may still receive them.

The **Vehicle** page keeps the 3D view open and shows live ROS signals:

- `/guardian/mode` and `/vehicle_command`: drive mode and the throttle,
  steering, and brake commands. These are commands, not actuator feedback.
- `/odometry`: X and Y in the odom frame, heading, and a speed estimate in mph
  calculated from consecutive LiDAR odometry poses and timestamps. KISS-ICP
  does not publish a measured velocity; this estimate can be noisy or jump.
- `/joy` and the hub's sensor status: recent gamepad, LiDAR, and camera topic
  activity. Receiving a topic does not prove that the device is healthy.
- `/vehicle_state`: a positive reported battery voltage, when available. The
  current Arduino firmware does not send telemetry, and the serial bridge uses
  zero for missing fields, so a missing voltage is shown as unavailable.
- `/diagnostics`: up to four recent component messages, prioritizing warnings
  and errors. Missing diagnostics do not mean the vehicle is healthy.

Stale readings clear rather than remaining on screen as live values. Mode is
shown only while its publisher is present and the command stream is recent.
The page subscribes only while open, so rates and odometry-based speed need a
moment to populate after navigating to it. The vehicle nodes continue running.

Open **DevTools → Calibration**, then select **Start**. Each camera continues
to be estimated until you select it for review and accept it. A successful
RANSAC/ICP result can be reviewed immediately; the independent confirmation
count is displayed as additional evidence and continues to update. The pose in
the review is held fixed while live clouds and newer estimates keep updating,
so **Accept** applies the exact pose being reviewed. **Decline and retry**
discards that proposal and starts a fresh search for that camera.
The blue sweep and arcs in the 3D view are an activity animation while
calibration runs and the Calibration page is open; they do not represent
measured scan coverage or progress. The TF tree subscribes while this page is
open and receives static transforms again on return.

After acceptance, computation stops for that camera. Select its button again
to recalibrate it; the previously accepted vehicle transform remains active
until a replacement is explicitly accepted. Other cameras continue computing.
Select **Stop** to stop searching for the remaining cameras. It
discards unaccepted proposals and keeps accepted poses; an acceptance already
queued for the vehicle publisher continues to completion. An Open3D operation
already in progress must return before its result can be discarded. Use
**Start** to resume searching for the remaining cameras.
The accepted camera mount is sent to `robot_state_publisher` and saved by the
registration node; preview transforms use separate frames and do not replace the
vehicle's TF before approval. The saved URDF is a record of the result and is
not automatically restored on the next launch, so calibration starts over when
the stack restarts.

**Calibration details** lets you select a camera without recalibrating it. It
shows the camera-cloud-to-LiDAR and camera-body-to-`base_link` homogeneous
matrices, with translation in meters, plus available RANSAC/ICP quality
metrics. A proposal or fixed review snapshot is explicitly marked as not yet
applied. The TF tree separately shows frames actually received from `/tf` and
`/tf_static` in both a graphical tree and a selectable list; dynamic frames
with no update for five seconds are marked stale. Tap a graph node or list row
to inspect it. Swipe the graph horizontally or vertically to explore it.
The highlighted frames identify the base and the selected camera where present.
Select a TF frame to inspect the published child-to-parent matrix, including
the LiDAR mount if its frame is in the tree. Static transforms remain visible;
dynamic transforms are labeled live or stale.

For a layout-only preview with simulated 3D points and no ROS connection:

```bash
ros2 run vehicle_ui_hub vehicle_ui_hub --preview --calibration --windowed
```

Use `--preview --vehicle --windowed` to inspect the vehicle status layout with
simulated values. Add `--preview-size 800x560` to check a smaller window. The
preview does not connect to the vehicle. Use `--preview --recording --windowed`
to inspect the recording panel, or `--preview --tools --windowed` for the
developer tools panel.
`--preview --network --windowed` shows local interface information without
making external requests or starting packet capture.
Use `--preview --idle --windowed` to inspect the idle screen without ROS.

Tap **Internet** in the top bar, or choose **DevTools → Network**, to see the
Karbon's physical network ports. The list refreshes every two seconds and shows
carrier state and IPv4 addresses. Select a port to see its speed, MAC address, and cumulative
interface error/drop counters. A connected link does not prove that the device
at the other end is reachable; counters can also reset when a driver restarts.
The internet indicator tests a TCP connection to `1.1.1.1:443` every 30 seconds.
An unverified result can mean the connection is down or that a firewall blocks
that particular check. The container reads the Karbon's `/etc/localtime` so the
clock uses the host's local timezone.

The top bar also shows weather for an approximate location derived from the
connection's public IP using [IPWHOIS](https://ipwhois.io/documentation), then
retrieves current conditions from [Open-Meteo](https://open-meteo.com/en/docs).
It refreshes every 15 minutes. This is not GPS: the location may be wrong on a
campus network, VPN, or mobile connection. The network page displays the
estimated city, and weather is unavailable when either service cannot be reached.
These services see the public IP; Open-Meteo also receives the estimated location.

For packet diagnostics, select a physical port, including any `enp…` port, and
press **Start**. The hub runs `tcpdump` without a shell and does not request
promiscuous mode. It shows packet summaries without saving a capture file. Each run stops after
5,000 packets; the display retains the last 400 lines and may skip lines during
bursts. Press **Stop** or leave the page to stop sooner. Packet summaries can
contain addresses and other network details. Capture requires permission to use
the interface and the `tcpdump` executable. The supplied Docker image includes
it; rebuild the image and recreate the container after updating the Dockerfile:

```bash
docker compose build dev
docker compose up -d dev
```

Use **DevTools → Record** to save a ROS bag. Select a preset and tap **Use
preset** to replace the current selection with:

- **Perception / world model:** the four rectified color images and matching
  `camera_info`, `/ouster/points`, `/ouster/imu`, `/tf`, `/tf_static`,
  `/robot_description`, `/odometry`, `/vehicle_command`, `/vehicle_state`, and
  `/guardian/mode`.
- **Perception + ZED depth:** the same topics plus each camera's registered
  depth image and depth `camera_info`. Depth must be enabled and these extra
  streams use substantially more bandwidth and storage.
- **LiDAR / localization:** LiDAR, IMU, transforms, robot description and odometry.
- **Gamepad / controls:** gamepad input, manual and selected commands, vehicle
  feedback, drive mode, odometry, and transforms.

These are starting points, not a complete dataset specification for every
world model. A bag alone does not reconstruct a CARLA world. Verify sensor
calibration, clock synchronization, topic rates and model-specific requirements.
The base perception preset avoids duplicate panorama and camera point clouds;
add a topic manually if your pipeline needs it.
The fields in `/vehicle_state` depend on what the vehicle firmware sends; a
zero speed field does not establish that the vehicle is stopped.

After applying a preset, **Selected only** shows its topics. Turn it off to
add other topics, or filter by LiDAR, cameras, IMU, or transforms. **Save
selection for later** stores one custom selection in the Hub settings; choose
**My saved selection** and **Use preset** to restore it after a restart.
Known topics appear even while offline; other topics appear as ROS discovers
them. **Waiting** means no publisher is currently visible. Selecting a topic
does not enable its sensor or ZED depth stream.

Highlight a topic and choose **Measure rate** to inspect its observed receive
rate. This temporarily subscribes to that one topic, which can add network and
CPU load for large images or clouds. Monitoring stops when you leave the page
or start recording. A dash can mean no recent messages; low-rate and latched
topics may show a dash between messages. The Hub also pauses its topic-list
refresh while recording; rosbag continues discovering the selected topics.

Enter an optional **Bag name** before recording. Use **Keyboard** for touch
input, or leave the name empty for an automatic timestamp. The name is used for
the bag directory and its storage files. If that name already exists, a number
is added; existing bags are never overwritten. Slashes and control characters
are not allowed in a name.

Choose **Start**, then **Stop** to finish the bag. Recording
continues while you use other hub pages; the 3D view has a thin red outline and
the top bar shows **REC** with elapsed time. The outline changes color without
resizing the view. It clears when recording stops, while the header shows **Saving**
until the bag finishes closing.
Closing the hub requests a stop and waits for the recorder to finish. By
default bags are saved under `recordings/` in the repository on the Karbon;
the folder is bind-mounted into the container and ignored by Git. **Change**
chooses another folder; use a mounted folder to retain bags if the container is
recreated. Include `/tf` and `/tf_static` when playback needs frame transforms.
Images and point clouds can consume substantial disk space and network bandwidth.
The recorder does not enable compression, avoiding extra compression work on
the Karbon. After saving, the page reports selected topics with zero recorded
messages when metadata is available. A nonzero count does not prove that every
frame arrived or that the sensors were synchronized.
Verify the saved topics and message counts with
`ros2 bag info <bag-directory>`. Stopping the full stack also requests a clean
bag stop; allow it time to finish before shutting down the computer.

The hub uses `/lidar_perception/hub/status` for state, the
`/lidar_perception/hub/{start,stop}` services for the calibration session, the
`/lidar_perception/hub/<camera>/{review,accept,recompute,release}` services for
decisions, and `/lidar_perception/hub/clouds/<camera>` for the camera-cloud
previews. The initial view configuration is in `config/hub.rviz`.

## Source layout

- `src/hub.py` assembles the window, pages, calibration services, and ROS executor.
- The feature-named Python files implement cameras, vehicle status, calibration
  details, recording, network, and startup/idle screens.
- `src/rviz_bridge.cpp` and `src/rviz_viewport.py` embed and control the native
  RViz widget. RViz handles its own point-cloud, TF, and robot-model rendering.
- `test/` exercises the UI and ROS behavior with simulated inputs. To run the
  native RViz integration tests, provide an X display (or Xvfb), set
  `NOVA_RVIZ_INTEGRATION=1`, and run the package's CTest suite after building.
