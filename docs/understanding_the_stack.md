# Understanding the 1811 Stack — a ground-up ROS 2 guide (using our own code)

Written for someone new to ROS 2 who wants to *understand* the machine, not
just run commands. Every concept is grounded in a real file in this repo. Read
it slowly once; keep it as a reference.

**Contents**
1. The one big idea: ROS 2 is many small programs talking over a network
2. Nodes
3. Topics and messages (pub/sub)
4. Services (request/response)
5. Parameters
6. TF: the coordinate-frame tree
7. How nodes find each other (the graph, DDS, domains)
8. Packages — the unit of code
9. The workspace and `colcon` (building)
10. Sourcing and overlays (why every terminal needs it)
11. Launch files
12. YAML parameter files
13. Remapping
14. The `ros2` command line, decoded
15. The Docker layer
16. The whole 1811 graph, node by node
17. How to write your own node (anatomy + checklist)

---

## 1. The one big idea

ROS 2 is not a program. It's a **way for many small independent programs to talk
to each other.** Each program does one job (read the lidar, estimate pose, follow
a path) and they exchange data over named channels. That's the whole philosophy:
small pieces, loosely joined, so you can develop, restart, and reason about each
one alone.

In our stack, "read the gamepad," "turn stick positions into a drive command,"
"send that command to the Arduino," and "estimate where we are from lidar" are
four *separate programs*. None imports the others. They only agree on **what
messages they send and receive.** That agreement — the topic names and message
types — is the real interface, not function calls.

Why this matters for you: when something breaks, you don't debug "the system,"
you ask "which program, and is the data crossing between them?" That's why we
spent so long on `ros2 topic hz` and `ros2 topic list` — those inspect the
channels between programs.

---

## 2. Nodes

A **node** is one running program in the ROS graph. It has a name, and it can
publish/subscribe topics, offer/call services, hold parameters, and run timers.

Our nodes (each is a node even though several live in one *package*):

| Node | Package | Job |
|---|---|---|
| `joy_node` | `joy` (external) | read the physical gamepad → publish `/joy` |
| `gamepad_node` | `teleop_bridge` | `/joy` → `/vehicle_command` |
| `serial_bridge_node` | `teleop_bridge` | `/vehicle_command` → Arduino serial |
| `kiss_icp_node` | `kiss_icp` (vendored) | `/ouster/points` → `/odometry` + TF |
| `robot_state_publisher` | (URDF) | URDF → `/tf_static`, `/robot_description` |
| `route_recorder_node` | `routing` | `/odometry` → CSV waypoint file |
| `pure_pursuit_node` | `control` | `/odometry` + path → `/cmd/auto` |

In Python, a node is a class that inherits from `rclpy.node.Node`. Open
[`teleop_bridge/gamepad_node.py`](../ros2_ws/src/teleop_bridge/teleop_bridge/gamepad_node.py)
and you'll see:

```python
class GamepadNode(Node):
    def __init__(self):
        super().__init__('gamepad_node')   # <- the node's name in the graph
```

The string `'gamepad_node'` is the name you see in `ros2 node list`. Everything
the node does (subscriptions, publishers, timers) is set up in `__init__`.

---

## 3. Topics and messages (pub/sub)

A **topic** is a named channel for a continuous stream of data — like `/odometry`
or `/joy`. Communication is **publish/subscribe**:
- a node **publishes** messages onto a topic,
- any number of nodes **subscribe** to receive them,
- it's **asynchronous** and **anonymous**: the publisher doesn't know or wait for
  subscribers. Fire and forget.

Every topic carries exactly one **message type** — a typed data structure.
Publisher and subscriber *must agree on the type*, or they don't connect.

Look at `gamepad_node` setting up both ends:

```python
self.sub = self.create_subscription(Joy, '/joy', self.on_joy, 10)
self.pub = self.create_publisher(VehicleCommand, '/vehicle_command', 10)
```

- `create_subscription(Joy, '/joy', self.on_joy, 10)` — "subscribe to `/joy`,
  which carries `sensor_msgs/Joy`; every time a message arrives, call
  `self.on_joy`; keep a queue of up to 10." That callback-on-arrival is the core
  pattern — your code runs in response to incoming messages.
- `create_publisher(VehicleCommand, '/vehicle_command', 10)` — "I will publish
  `vehicle_msgs/VehicleCommand` on `/vehicle_command`."

The `10` is the **queue depth** (part of "QoS" — quality of service). If messages
arrive faster than you process them, up to 10 wait; older ones drop. (QoS also
covers reliability and "durability" — that's the `TRANSIENT_LOCAL`/latched thing
you saw with `/tf_static`: a latched publisher re-delivers its last message to
subscribers that join late. `pure_pursuit_node` matches that QoS when it
subscribes `/planning/path`, or it'd never see a path published before it
started.)

### Messages

A message type is defined in a `.msg` file — plain field declarations. Ours live
in `vehicle_msgs`. [`msg/VehicleCommand.msg`](../ros2_ws/src/vehicle_msgs/msg/VehicleCommand.msg):

```
std_msgs/Header header
float32 throttle
float32 steer
float32 brake
```

That's it — a header (timestamp + frame id) plus three floats. When you build
`vehicle_msgs`, ROS **generates code** from this (Python classes, C++ headers) so
any node in any language can construct and read it. Standard message packages you
already use: `std_msgs` (primitives), `sensor_msgs` (`Joy`, `PointCloud2`,
`Imu`), `nav_msgs` (`Odometry`, `Path`), `geometry_msgs` (poses, transforms).

**Why a custom `vehicle_msgs`:** no standard message means "throttle/steer/brake
for our vehicle," so we define our own. That's normal — you invent message types
for your domain.

---

## 4. Services (request/response)

Topics are streams. **Services** are one-shot **request → response** calls — like
calling a function on another program and waiting for the answer.

`route_recorder_node` uses this for "save now":

```python
self.create_service(Trigger, '~/save', self._on_save)
```

- `Trigger` is a built-in service type: empty request, `{success, message}`
  response.
- `~/save` — the `~` means "my node's namespace," so the full name becomes
  `/route_recorder_node/save`.

You call it from the CLI:
```bash
ros2 service call /route_recorder_node/save std_srvs/srv/Trigger {}
```
The node runs its `_on_save` handler (writes the CSV, builds a response) and the
response prints back to you.

**Topic vs service, the rule of thumb:** continuous data that flows regardless of
who's listening → topic (odometry, joy). A one-time imperative action with an
answer → service (save, reset, arm). Recording pose is a topic subscription;
"save the buffer to disk right now" is a service.

---

## 5. Parameters

**Parameters** are per-node configuration values, set at launch, read in code.
They're how you avoid hard-coding numbers.

In `pure_pursuit_node`:
```python
self.declare_parameter('lookahead_distance', 1.0)   # name, default
...
self._lookahead_base = self.get_parameter('lookahead_distance').value
```

`declare_parameter` announces the parameter and its default; `get_parameter(...).value`
reads it. The **default** is used unless someone overrides it — and you can
override three ways:

1. **CLI:** `ros2 run control pure_pursuit_node --ros-args -p lookahead_distance:=1.5`
2. **Launch file:** `parameters=[{'lookahead_distance': 1.5}]`
3. **YAML file:** a `.yaml` passed to the node (section 12).

Parameters are typed. That's why the launch file wraps baud with
`ParameterValue(baud, value_type=int)` — a launch argument is a string, but
`baud` was declared as an int, so it must be coerced.

---

## 6. TF: the coordinate-frame tree

Every sensor reports data in *its own* coordinate frame. The lidar's points are
in `os_lidar`; the vehicle's pose is in `base_link`. To combine them you need to
know how frames relate. **TF** (transform framework, library `tf2`) is ROS's
system for that.

- Frames form a **tree**: `odom → base_link → os_sensor → os_lidar`, etc.
- Each edge is a transform (translation + rotation) from parent to child.
- Transforms are published on two topics: **`/tf`** (things that move, e.g.
  `odom → base_link` from KISS-ICP, ~10 Hz) and **`/tf_static`** (fixed mounts,
  published once, latched — e.g. `base_link → os_sensor` from the URDF).
- Any node can then ask tf2 "give me the transform from A to B at time t" and tf2
  walks the tree to compute it.

This is why nothing worked until `robot_state_publisher` was running: it's the
node that turns your URDF into the `/tf_static` edges (`base_link → os_sensor`,
wheels, cameras). Without it, KISS-ICP couldn't connect `base_link` to
`os_lidar`, and the tree was in two disconnected halves.

`robot_state_publisher` reads the URDF (an XML description of the vehicle's
geometry, in `vehicle_1811_description`) and publishes the static transforms.
The URDF *is* your measurements turned into frames.

---

## 7. How nodes find each other (the graph, DDS, domains)

Nodes discover each other **automatically over the network** — you never tell a
node where another one is. Underneath, ROS 2 uses **DDS** (a pub/sub middleware);
our implementation is **Fast DDS** (`rmw_fastrtps_cpp`). When a node starts, it
announces itself; others hear the announcement and connect if topic names + types
+ QoS match.

Two knobs control who can hear whom:
- **`ROS_DOMAIN_ID`** — an integer. Nodes only see each other if they share the
  same domain. (Two machines on different domains → invisible to each other. That
  was the Karbon-vs-Jetson bug: one was on 0, one on 1.)
- **`RMW_IMPLEMENTATION`** — which DDS. Must match across nodes.

On one machine, DDS uses **shared memory** to move data between processes
(fast). Across machines it uses UDP over the network. All the
`fastdds_cable.xml` / `ipc: host` work was tuning *which network path* DDS uses
so cross-machine and cross-container discovery actually worked. You don't need
that day to day — just know: same domain + reachable network = they find each
other.

Inspect the live graph:
```bash
ros2 node list           # who's running
ros2 topic list          # what channels exist
ros2 topic info /odometry -v   # who publishes/subscribes it, and their QoS
```

---

## 8. Packages — the unit of code

A **package** is the smallest thing you can build and install. Our
`ros2_ws/src/` holds many: `teleop_bridge`, `control`, `routing`,
`localization`, `vehicle_msgs`, `vehicle_1811_description`, plus vendored ones
(`ouster-ros`, `kiss-icp`) and empty skeletons (`camera_perception`, etc.).

Every package has a **`package.xml`** — its manifest: name, version, and
dependencies. From [`teleop_bridge/package.xml`](../ros2_ws/src/teleop_bridge/package.xml):

```xml
<name>teleop_bridge</name>
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>vehicle_msgs</depend>
<export>
  <build_type>ament_python</build_type>
</export>
```

The `<depend>` lines let the build tool order things (build `vehicle_msgs` before
`teleop_bridge`, since it uses those messages). The `<build_type>` decides which
of the **two package flavors** this is:

### Flavor A: `ament_python` (pure Python nodes)
Used by `teleop_bridge`, `control`, `routing`. Built via a **`setup.py`** (Python
packaging). The key part, from [`teleop_bridge/setup.py`](../ros2_ws/src/teleop_bridge/setup.py):

```python
entry_points={
    'console_scripts': [
        'gamepad_node = teleop_bridge.gamepad_node:main',
        'serial_bridge_node = teleop_bridge.serial_bridge_node:main',
    ],
},
```

**This is the line that makes `ros2 run teleop_bridge gamepad_node` work.** It
says "create an executable named `gamepad_node` that runs the `main` function in
`teleop_bridge/gamepad_node.py`." If you add a new node file, you add a line here
or `ros2 run` won't find it.

`data_files` in the same `setup.py` is what installs non-code files (launch files,
YAML configs) into the install tree so they can be found at runtime:
```python
('share/' + package_name + '/launch', ['launch/teleop_bridge.launch.py']),
```

### Flavor B: `ament_cmake` (C++, or message generation)
Used by `vehicle_msgs`, `vehicle_1811_description`, `localization`. Built via a
**`CMakeLists.txt`**. `vehicle_msgs` uses it because generating message code is a
CMake job — [`vehicle_msgs/CMakeLists.txt`](../ros2_ws/src/vehicle_msgs/CMakeLists.txt):

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleCommand.msg"
  "msg/VehicleState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
```

That turns your `.msg` files into usable Python/C++ types at build time.
`vehicle_1811_description` and `localization` use `ament_cmake` just to *install*
files (URDF, launch, config) — they have no compiled code, they're config
packages.

**"ament"** is ROS 2's build system layer that sits on top of Python's setuptools
and CMake and makes packages discoverable by the rest of ROS.

---

## 9. The workspace and `colcon` (building)

A **workspace** is a directory with a `src/` full of packages. Ours is
`ros2_ws/`. You build it with **`colcon`**:

```bash
cd /vehicle_1811/ros2_ws
colcon build
```

`colcon` finds every package under `src/`, builds each in dependency order, and
produces three sibling directories:
- **`build/`** — intermediate build artifacts.
- **`install/`** — the finished, usable result (this is what you *source*).
- **`log/`** — build logs.

Flags you've used and what they mean:
- **`--packages-select control routing`** — build only these packages, not the
  whole workspace. Fast when you changed one thing.
- **`--packages-up-to ouster_ros`** — build this package *and everything it
  depends on*. (Used for the Ouster driver so its message deps got built too.)
- **`--symlink-install`** — instead of *copying* Python files and configs into
  `install/`, symlink them. Consequence: **editing a Python node takes effect
  with just a node restart, no rebuild.** (C++ still needs a rebuild — it's
  compiled.) This is why we kept saying "pure-Python change, no rebuild needed."

When do you *have* to rebuild?
- Edited C++ → yes.
- Added/removed a package, or a new node's `entry_points`, or `package.xml`/
  `CMakeLists`/`setup.py` → yes.
- Edited a `.msg` → yes (regenerate types).
- Edited an existing Python node's logic (with `--symlink-install`) → no, just
  restart the node.

---

## 10. Sourcing and overlays (why every terminal needs it)

This trips up everyone. **"Sourcing"** a setup script edits your shell's
environment variables so ROS can find packages, executables, and message types.

There are two layers:
1. **The underlay:** `source /opt/ros/humble/setup.bash` — makes core ROS 2 and
   all system-installed packages available. (Our container's entrypoint does this
   for you automatically.)
2. **The overlay:** `source /vehicle_1811/ros2_ws/install/setup.bash` — layers
   *your workspace's* built packages on top. **This is the one you keep having to
   run.** Without it, the shell doesn't know `teleop_bridge`, `control`, or the
   custom `vehicle_msgs` type exist.

That's exactly why `ros2 topic echo /vehicle_command` failed with "message type
invalid" — the shell hadn't sourced the overlay, so it had never heard of
`vehicle_msgs/VehicleCommand`. And why it's per-terminal: sourcing only affects
*that shell's* environment. A new `docker compose run` shell starts fresh and
must source again.

What sourcing actually sets (peek with `echo $AMENT_PREFIX_PATH`): `PATH` (so
`ros2 run` finds your executables), `AMENT_PREFIX_PATH` and
`PYTHONPATH`/library paths (so packages and message types resolve). It's just
environment variables — nothing magic, nothing permanent.

> Tip: you can add `source /vehicle_1811/ros2_ws/install/setup.bash` to the
> container's shell startup so you never type it again — a good future Dockerfile
> tweak.

---

## 11. Launch files

Running nodes one-by-one with `ros2 run` gets old fast — a real system is 5+
nodes with params and remaps. A **launch file** starts a whole set at once, with
their configuration, in one command. Ours are Python (ROS 2 also supports XML).

Walk through [`control/launch/pure_pursuit.launch.py`](../ros2_ws/src/control/launch/pure_pursuit.launch.py):

```python
def generate_launch_description():
    control_share = FindPackageShare('control')
    config = PathJoinSubstitution([control_share, 'config', 'pure_pursuit.yaml'])
    path_file = LaunchConfiguration('path_file')

    return LaunchDescription([
        DeclareLaunchArgument('path_file', default_value=default_path_file, ...),
        DeclareLaunchArgument('cmd_topic', default_value='/cmd/auto', ...),
        DeclareLaunchArgument('use_sim', default_value='false', ...),
        Node(
            package='control', executable='pure_pursuit_node', name='pure_pursuit_node',
            parameters=[config, {'path_file': path_file, 'cmd_topic': cmd_topic}],
            output='screen',
        ),
        Node(
            package='control', executable='bicycle_sim_node', name='bicycle_sim_node',
            condition=IfCondition(use_sim),
        ),
    ])
```

Piece by piece:
- **`generate_launch_description()`** — every launch file defines this; it returns
  a `LaunchDescription` (a list of things to do).
- **`DeclareLaunchArgument('path_file', default_value=...)`** — declares a
  *launch argument* you can set on the command line:
  `ros2 launch control pure_pursuit.launch.py path_file:=/my/route.csv`. If you
  don't, it uses the default.
- **`LaunchConfiguration('path_file')`** — a reference to that argument's value,
  resolved when the launch runs.
- **`FindPackageShare('control')` + `PathJoinSubstitution`** — build a path into
  the *installed* `control` package (its `share/` dir), so the launch finds
  `config/pure_pursuit.yaml` no matter where the workspace lives. Don't hard-code
  absolute paths; find them via the package.
- **`Node(package=..., executable=..., name=..., parameters=[...])`** — start one
  node. `executable` matches the `entry_points` name from `setup.py`.
  `parameters` is a list: a YAML file *and/or* inline dicts, applied in order
  (later overrides earlier — note it loads `config` first, then overrides
  `path_file`/`cmd_topic` from the launch args).
- **`condition=IfCondition(use_sim)`** — only start this node if `use_sim:=true`.
  That's how one launch file serves both "with simulator" and "without."
- **`output='screen'`** — print the node's logs to the terminal.

Other launch pieces you've seen elsewhere:
- **`IncludeLaunchDescription(...)`** — run another launch file from inside this
  one (your `localization.launch.py` includes KISS-ICP's launch).
- **`SetRemap(src=..., dst=...)`** — rename a topic for nodes launched after it
  (used to send KISS-ICP's `/kiss/odometry` out as `/odometry`).

---

## 12. YAML parameter files

For nodes with many parameters, listing them in the launch file is messy — put
them in a **YAML file** instead. The structure is fixed and strict. From
[`localization/config/kiss_icp.yaml`](../ros2_ws/src/localization/config/kiss_icp.yaml):

```yaml
kiss_icp_node:            # <- the NODE NAME this applies to
  ros__parameters:        # <- literally this key, always
    data:
      max_range: 100.0
      min_range: 1.0
    registration:
      max_num_iterations: 500
```

Rules that bite people:
- The **top key must be the node's name** (or `/**` to match any node). If it
  doesn't match the running node's name, the params are silently ignored.
- The second key is **always** `ros__parameters:` (two underscores).
- Nesting (`data:`, `registration:`) is how that particular node groups its
  params — it's node-specific, not a ROS rule.

You feed a YAML to a node either on the CLI
(`--ros-args --params-file kiss_icp.yaml`) or in a launch file
(`parameters=[config]`, as `pure_pursuit.launch.py` does). Values in the YAML
override the `declare_parameter` defaults in code.

---

## 13. Remapping

Nodes hard-code topic names in code (`/cmd/auto`, `/joy`), but you can **rename
them at launch without editing the node** — that's remapping. Two forms:
- CLI: `ros2 run control pure_pursuit_node --ros-args -r /cmd/auto:=/vehicle_command`
- Launch: a node's `remappings=[('/cmd/auto', '/vehicle_command')]`, or the
  standalone `SetRemap`.

This is why `pure_pursuit_node` can be *bench-safe by default* (publishes to
`/cmd/auto`, which drives nothing) yet be pointed at the real Arduino for a test
(`cmd_topic:=/vehicle_command`) with no code change. Remapping is how you reuse a
node in different wirings.

---

## 14. The `ros2` command line, decoded

`ros2` is the one CLI; everything is `ros2 <thing> <verb>`:

| Command | What it does |
|---|---|
| `ros2 run <pkg> <exe>` | run one node (the `exe` from `entry_points`) |
| `ros2 launch <pkg> <file>` | run a launch file (many nodes + config) |
| `ros2 node list` / `node info <n>` | list running nodes / inspect one's pubs/subs |
| `ros2 topic list` | all active topics |
| `ros2 topic echo <t>` | print messages on a topic |
| `ros2 topic hz <t>` | measure a topic's publish rate |
| `ros2 topic info <t> -v` | type, publisher/subscriber count, QoS |
| `ros2 topic pub <t> <type> <data>` | publish a message by hand (testing) |
| `ros2 service call <s> <type> <req>` | call a service once |
| `ros2 param get/set <node> <name>` | read/change a running node's parameter |
| `ros2 bag record/play` | record topics to disk / replay them |
| `ros2 pkg list` / `pkg prefix <p>` | list packages / find a package's install path |
| `ros2 run tf2_tools view_frames` | dump the live TF tree to a PDF |

When something's wrong, the debugging ladder is almost always:
`ros2 node list` (is it running?) → `ros2 topic list` (does the channel exist?)
→ `ros2 topic hz`/`echo` (is data flowing?) → `ros2 topic info -v` (do the two
ends match on type + QoS?).

---

## 15. The Docker layer

None of the above changes because of Docker — Docker just provides the *Ubuntu +
ROS 2 environment* so you don't install ROS on the host. Key ideas:
- **Image** = the built filesystem template (`vehicle_1811`, from the
  `Dockerfile`). **Container** = a running instance of it.
- `docker compose run --rm dev bash` = start a throwaway container from the `dev`
  service and drop into a shell. `--rm` deletes it on exit.
- **Bind mounts** (in `docker-compose.yml`) punch holes between host and
  container: the repo (`$PWD` → `/vehicle_1811`), `/dev` (so USB/serial devices
  appear), the X socket (so GUIs show). Because the repo is bind-mounted, editing
  files on the host changes them in the container instantly, and things written
  in the container (bags, routes, `install/`) appear on the host.
- `network_mode: host` + `ipc: host` = share the host's network and shared-memory
  so DDS works normally across containers/machines.
- The container runs as **root**, which is why `git`/`colcon` inside it leave
  root-owned files — do git on the host.

So the layering is: **Docker gives you the OS+ROS → you source the workspace →
you run nodes.**

---

## 16. The whole 1811 graph, node by node

Two data paths share the same odometry.

**Teleop (manual driving):**
```
gamepad → joy_node → /joy → gamepad_node → /vehicle_command → serial_bridge_node → Arduino
        (sensor_msgs/Joy)   (maps axes)    (vehicle_msgs/VehicleCommand)   (JSON over serial)
```
- `joy_node` reads the pad, publishes raw axes/buttons as `/joy`.
- `gamepad_node` maps `axes[1]`→throttle, `axes[3]`→steer, `axes[2]`→brake (the
  constants you tuned), enforces "brake overrides throttle," publishes
  `VehicleCommand`.
- `serial_bridge_node` converts to `{"speed", "steering", "braking"}` JSON,
  scales throttle by `MAX_SPEED_MPH`, writes it to the Arduino.

**Localization (always running underneath):**
```
Ouster → ouster driver → /ouster/points → kiss_icp_node → /odometry + TF(odom→base_link)
robot_state_publisher → /tf_static (base_link→os_sensor, wheels, cameras) + /robot_description
```
- KISS-ICP scan-matches each cloud to estimate motion → `/odometry`.
- `robot_state_publisher` supplies the fixed geometry so the TF tree connects.

**Teach-and-repeat (built on odometry):**
```
TEACH:  /odometry → route_recorder_node → routes/route_<ts>.csv (x,y,yaw waypoints)
REPEAT: CSV + /odometry → pure_pursuit_node → /cmd/auto → mode_manager → serial_bridge → Arduino
```
- `route_recorder_node` downsamples `/odometry` to waypoints, saves a CSV.
- `pure_pursuit_node` loads that CSV, follows it against live `/odometry`,
  publishes steering+throttle on `/cmd/auto`. It fails safe (zeros on stale
  odometry). `mode_manager` gates `/cmd/auto` → `/vehicle_command` behind a
  deadman; bringup runs it by default. Outside bringup, you bench-test by
  remapping.

Notice the whole thing is just nodes + topics from sections 2–3. Nothing more
exotic.

---

## 17. How to write your own node (anatomy + checklist)

Here's the skeleton every Python node follows — compare it to `gamepad_node` and
`route_recorder_node`:

```python
import rclpy
from rclpy.node import Node
from some_msgs.msg import SomeType          # the message types you use

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')                       # 1. name
        self.declare_parameter('rate_hz', 10.0)           # 2. params
        self._rate = self.get_parameter('rate_hz').value

        self.sub = self.create_subscription(              # 3. inputs
            SomeType, '/in_topic', self._on_msg, 10)
        self.pub = self.create_publisher(                 # 4. outputs
            SomeType, '/out_topic', 10)

        self.timer = self.create_timer(                   # 5. periodic work
            1.0 / self._rate, self._tick)

    def _on_msg(self, msg):                               # runs on each input
        self._last = msg

    def _tick(self):                                      # runs on the timer
        out = SomeType()
        self.pub.publish(out)

def main():
    rclpy.init()                                          # start ROS
    node = MyNode()
    rclpy.spin(node)                                      # process callbacks forever
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

The mental model: **`__init__` wires everything up; then `rclpy.spin()` sits in a
loop calling your callbacks** (`_on_msg` when a message arrives, `_tick` on the
timer). You never write the loop — spin does. Two patterns you'll see in our
code:
- **Callback-driven** (`gamepad_node`): do the work inside the subscription
  callback — a new `/joy` in, a `/vehicle_command` out.
- **Timer-driven** (`pure_pursuit_node`): the subscription just *stores* the
  latest odometry; a fixed-rate timer does the control math and publishes. This
  decouples output rate from input rate and makes stale-data failsafes easy
  (check "how old is the last message?" every tick).

**Checklist to add a new node to a package:**
1. Write `my_pkg/my_pkg/my_node.py` with a `main()` (as above).
2. Add it to `setup.py` `entry_points`:
   `'my_node = my_pkg.my_node:main'`.
3. Add any new message deps to `package.xml` (`<depend>...`).
4. `colcon build --packages-select my_pkg && source install/setup.bash`.
5. `ros2 run my_pkg my_node`.

For heavier logic, follow the repo's own pattern: put the pure math in a
`_core.py` with no ROS imports (`pure_pursuit_core.py`, `route_recorder_core.py`)
and keep the `_node.py` thin. That's why those packages can unit-test the logic
with plain `pytest` — no ROS, no hardware. Do the same and your code stays
testable.

---

## Where to go next

- Re-read one node end-to-end with this guide open — `route_recorder_node.py` is
  a great one: params, a subscription, three services, a timer, file I/O.
- Then read its `_core.py` to see the ROS-free logic split.
- Then read `pure_pursuit_node.py` for the timer-driven control pattern.
- The two milestone docs — [`teach_and_repeat_plan.md`](teach_and_repeat_plan.md)
  and [`teach_and_repeat_guide.md`](teach_and_repeat_guide.md) — connect this
  machinery to *why* the system is shaped the way it is.
```
