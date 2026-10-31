# Lidar Teach-and-Repeat: Plan & Primer

A guided, staged plan to put a real autonomous behavior on the vehicle — with a
rigorous localization deliverable as the guaranteed floor if the full goal runs
out of runway. Doubles as a primer on *how* lidar odometry works, so the plan is
legible to someone new to the stack.

---

## 1. The goal

**Teach-and-repeat:** drive a route once by hand while the vehicle records where
it went, then have it drive that same route back autonomously. No cameras, no
obstacle avoidance, no pre-drawn map — just sensing → state estimation → path
following → control → actuation, the complete autonomy loop in its simplest
honest form. The payoff artifact is a video of the car driving a route by
itself, backed by real numbers.

---

## 2. How the pieces actually work

The plan only makes sense once these click, so start here.

### The lidar and what it produces

The Ouster OS1-128 is a spinning laser range-finder. It fires laser pulses across
**128 vertical channels**, sweeping ~1024–2048 steps per full rotation, ~10–20
times a second. For each pulse it measures time-of-flight to get a distance. One
full rotation produces a **point cloud**: tens of thousands of 3D points
`(x, y, z)`, each a spot where a beam hit something, in the sensor's own frame.
That cloud — published on `/ouster/points` — is the raw geometry of the world
around the car, refreshed every rotation. That is *all* the lidar does: it
measures shape. It has no idea where it is.

### KISS-ICP: turning geometry into motion

**KISS-ICP** ("Keep It Small and Simple — Iterative Closest Point") is a lidar
**odometry** pipeline from PRBonn. Odometry = estimating your own motion over
time.

The core idea: **the lidar can't feel motion, but it can see the world shift
between scans, and motion is what explains the shift.** Just as you sense you're
walking forward because the room slides past you, KISS-ICP infers "I moved
forward 0.2 m" from "the walls moved backward 0.2 m in the point cloud."

Each scan, it runs roughly this loop:

1. **De-skew** — the car moved *during* the rotation, so points captured early
   and late sit in slightly different places; correct for that.
2. **Downsample** to a voxel grid so the math is fast.
3. **Predict** where the car probably moved, from a constant-velocity guess.
4. **ICP alignment** — the heart of it. Take the new scan and a "local map" (the
   last several scans stacked), and find the rigid transform (rotation +
   translation) that best slides the new scan onto the map: match each point to
   its nearest neighbor in the map, compute the movement that shrinks those
   distances, repeat until it stops improving.
5. That transform **is** how far the car moved since the last scan. Chain them
   and you have a running **pose** — position + heading over time.
6. Fold the aligned scan into the local map and continue.

The output is a pose stream (`/odometry`) and a transform `odom → base_link` —
the car's pose in a fixed frame anchored wherever you switched it on. KISS-ICP
uses **no GPS and no IMU** — pure geometry. That makes it simple and robust in
structured places, but gives it a specific weakness (drift, below).

### The KISS family, and which variant you want

There are three related PRBonn repos, and it's worth knowing which does what so
the choice of "plain KISS-ICP upstream" is deliberate, not accidental:

| Repo | What it is | Role |
|---|---|---|
| **KISS-ICP** | Lidar odometry. Aligns each scan to a rolling local map, outputs a pose stream. | Live, on-vehicle. Drift accumulates forever (no loop closure). |
| **MapClosures** | Loop-closure *detection* — recognizes revisited places and reports the transform. | A library, not a pipeline. Detects only; fixes nothing itself. |
| **kiss-slam** | Full SLAM: runs KISS-ICP for odometry, builds a pose graph, calls MapClosures on revisits, optimizes globally. | **Offline.** Builds a corrected map from a bag. |

For this milestone you want **plain KISS-ICP, from PRBonn upstream**, for live
odometry. kiss-slam is skipped entirely here — global optimization retroactively
rewrites *past* poses, which a live controller can't consume, and you don't need
a corrected cross-session map when you record and drive in one sitting.

**Upstream vs. Nova's fork — the distinction that matters.** All three repos are
PRBonn's (MIT, well-maintained); Nova forked them with adaptations. Nova's fork
of KISS-ICP is specifically adapted for **adding incoming scans to an
already-existing map** — i.e. loading a map built earlier and localizing against
it. That is a **relocalization** capability ("given a map from before, where am I
in it *now*"), which is a fundamentally different problem from **odometry**
("starting from here, how have I moved since"). Teach-and-repeat in a single
continuous session is pure odometry: you never power-cycle the frame, so there's
no prior map to relocalize into. Upstream is the simpler, correct tool for that;
Nova's fork solves a problem you don't have yet. The day you want to drive a
route recorded on a *different day* — a fresh boot, a moved origin — is the day
the fork (or kiss-slam + relocalization) earns its keep. See §8, "how it fits."

### Drift — the thing you must measure first

Because KISS-ICP chains one scan-to-scan estimate onto the next, each tiny
alignment error **accumulates**. There's no GPS pulling it back to truth and no
loop-closure correction in live mode, so its idea of "where I am" slowly wanders
from reality. That wander is **drift**, and it's worst in **geometrically
featureless space** — a bare, flat lot with nothing vertical lets a scan slide
freely in one direction with nothing to lock onto.

Drift matters here specifically because the recorded route and the car's live
pose are *both* in KISS-ICP's frame, and the controller steers by comparing them.
Small drift → the comparison holds and the car stays on route. Large drift → the
car confidently drives off course and nothing detects it.

Measuring it without ground truth (you have no RTK/GPS) uses a **closed loop**:
drive out and physically return the car to the exact spot it started. In reality,
start = end, so whatever gap KISS-ICP reports between its start and end pose *is*
the accumulated drift — "it thinks I ended 0.4 m and 3° off, but I'm back on the
same tape mark → 0.4 m of drift over a 30 m loop." A visual version too: overlay
the point-cloud map of the start area against the end area; misalignment is the
drift made visible.

### Bags — how you develop without the vehicle

A **rosbag** (`ros2 bag`) is a timestamped recording of ROS messages on chosen
topics. Record the lidar, TF, and odometry once:

```bash
ros2 bag record -o loop1 /ouster/points /tf /tf_static /odometry
```

…then replay it any time, and downstream nodes **can't tell it isn't live**:

```bash
ros2 bag play loop1
```

This is the multiplier for a one-week timeline: record one drive, then tune
KISS-ICP and the controller at a desk against identical, repeatable input — no
vehicle needed — and let several people work off the same recording in parallel.

### The map is a coordinate system, not a destination

The local map KISS-ICP builds isn't something the car "reads for directions."
It's a **frame of reference** — like a street grid. Your recorded route and your
live pose are two addresses in that grid, which is the only reason subtracting
them ("route is 5 m ahead, 2 m left") means anything. The single constraint this
imposes: **teach and repeat must happen in one continuous session.** Restart
KISS-ICP between recording and driving and the grid's origin moves — the route
becomes gibberish in the new frame.

---

## 3. Where this fits: the foundation, not a detour

This milestone is not a throwaway demo — it is the **load-bearing spine** of the
full autonomy stack, and almost nothing built here gets discarded when the
system grows. Localization and control are bedrock: costmaps, obstacle
avoidance, planning, and relocalization all *assume* a working pose estimate and
a working controller underneath them. You cannot build the upper floors until
this one exists.

Concretely, here is how each piece persists into the full system (roadmap §2's
live loop) rather than being replaced:

| Built now | Becomes, later |
|---|---|
| **KISS-ICP odometry** (`/odometry`) | The state-estimation layer for the whole stack — later fused with IMU/encoders via `robot_localization`, or upgraded to FAST-LIO2, but the *slot* is permanent. Pose feeds the route layer, every costmap transform, and the controller. |
| **The recorded route** | The **route cost layer** (`/grid/route_distance`) — a static prior the planner prefers. When obstacles arrive, the planner deviates *around* them but still favors the recorded route. Nothing thrown away. |
| **Pure-pursuit controller** | Stays as the control layer. Later a Recursive Tree Planner (RTP) produces the path and pure pursuit follows it — the controller doesn't change, only what feeds it. |
| **Eval harness / drift number** | Remains how localization is validated forever — the same closed-loop metric guards every future change. |
| **Mode manager, serial bridge, TF/URDF** | Permanent infrastructure the whole vehicle runs on. |

Phase two — obstacle grid from lidar, grid summation, RTP planning, cameras — are
**layers added on top of a working system**, not prerequisites for it. That is
the entire reason to build this spine first: it turns everything afterward into
an *upgrade to something that already drives*, rather than one more half-finished
subsystem. This is also the natural point at which relocalization (Nova's KISS-ICP
fork, or kiss-slam) becomes worth adding — once single-session driving works and
you want it to survive across sessions.

---

## 4. Architecture — the node graph

```
                 ┌─────────────┐     /ouster/points      ┌───────────┐
   Ouster ──────▶│ ouster-ros  │────────────────────────▶│ KISS-ICP  │
                 └─────────────┘                          └─────┬─────┘
                                          /odometry + TF (odom→base_link)
                                                                │
        ┌───────────────────────────────────────┬─────────────┘
        │ TEACH                                  │ REPEAT
        ▼                                        ▼
  [route_recorder]                        [route_publisher] ──▶ /planning/path
  pose stream → file                                                │
        │                                 [pure_pursuit] ◀── /odometry + /planning/path
        │                                        │ → /vehicle_command
   (drive via gamepad)                           ▼
                                        [mode_manager] gates cmd ──▶ [serial_bridge] ──▶ Arduino
```

Already in hand: the Ouster driver, serial bridge, gamepad. New work: **KISS-ICP
(configured, not written), route recorder, route publisher, pure pursuit, mode
manager, eval harness.**

### Where it runs

Run the **entire loop on the Karbon.** The lidar and the Arduino are both there,
so keeping sensing → estimation → control → actuation local means no network
inside the safety-critical control loop and no dependence on the cross-machine
link for this milestone. That link stays useful as the **monitoring and fallback
path**: run RViz on the Jetson to watch map/route/pose live while the Karbon
drives, and if the Karbon's CPU can't keep up with KISS-ICP, stream
`/ouster/points` to the Jetson over the cable and run it there instead.

---

## 5. The workflow you're building toward

1. **Place** the car on a marked start spot.
2. **TEACH (manual):** drive a **loop** with the gamepad, back to the start. The
   recorder saves the pose path throughout.
3. Back at start — and **without restarting anything** (same session = same
   frame),
4. **REPEAT (autonomous):** flip the mode switch; the car drives the loop itself.

Driving a loop back to start does double duty: it positions the car at the
route's start so autonomous mode has somewhere to begin, **and** it is your drift
measurement for free. (A→B routes work too, but a loop is the best demo and the
built-in eval in one maneuver.)

---

## 6. The staged plan

Structured so each stage is independently shippable — you cannot end the week
with nothing.

### B1 — KISS-ICP live → `/odometry`  *(day 1)*
- **Vendor it:** clone PRBonn kiss-icp into `external/` (or host the wrapper in
  the empty `lidar_perception` skeleton), build against ROS 2.
- **Configure:** input `/ouster/points`; `base_frame = base_link`; publish the
  `odom → base_link` TF and an odometry topic remapped to `/odometry`.
- **Where the URDF pays off:** KISS-ICP needs the lidar mounting transform
  (`base_link → os_sensor → os_lidar`) to report the *vehicle's* pose, not the
  sensor's — that comes straight from step 1.
- **Note:** KISS-ICP likes good per-point timestamps for de-skew; the
  `LEGACY`-firmware workaround weakens those, but at 2–5 mph de-skew barely
  matters.
- **Done when:** in RViz (fixed frame `odom`), driving under teleop keeps the
  accumulated map *crisp* — walls stay thin, not smeared — and `base_link` tracks
  correctly through it.

### B2 — Bags + eval harness  *(day 1–2 · the floor)*
- **Record** 3–5 bags, at least one a closed loop.
- **Write ~250 lines of Python** that, from a bag, computes closed-loop drift
  (start-vs-end pose gap), renders the map self-overlay, and plots velocity
  continuity (jumps = tracking loss).
- **Done when:** you can state a drift number. **If the week collapses here, this
  alone is a legitimate, rigorous deliverable.**

### 🚦 Gate  *(end of day 2)*
Is drift acceptable in the test area? **Yes →** continue to A. **No** (bare
asphalt starving the geometry) **→** ship B, and scope any drive to short
distances where drift stays small. Mitigation: test near vertical structure —
buildings, curbs, poles, parked cars.

### A1 — Record & replay the route  *(day 3)*
- **`route_recorder`:** in TEACH mode, subscribe `/odometry` and append
  `(t, x, y, yaw)` to a file, downsampled to ~every 0.1–0.2 m.
- **`route_publisher`:** load the file, publish it as a latched `nav_msgs/Path`
  in the `odom` frame.
- **Done when:** drive a route, then see the exact path drawn in RViz.

### A2 — Pure pursuit  *(day 4)*
- **The algorithm:** find the point on the path a fixed "lookahead distance"
  ahead of the car, compute the steering curvature that arcs you to it
  (`δ = atan(wheel_base · κ)`), and hold a low target speed.
- **The calibration that bites:** `δ` is in **radians**, but the serial protocol
  wants **normalized −1..1 steering**, so you map `δ / MAX_TURN_ANGLE` — which
  means measuring `MAX_TURN_ANGLE` off the chassis. Sign/scale + lookahead tuning
  (too short → snakes; too long → cuts corners) is the fiddly half-day.
- **Done when:** on blocks, pushing the car along the path makes the wheels steer
  *toward* the path.

### A3 — Mode manager, integration, drive  *(day 5–6)*
- **`mode_manager`:** a gamepad **button** switches DISABLED / MANUAL /
  AUTONOMOUS and gates `/vehicle_command` — MANUAL passes gamepad, AUTONOMOUS
  passes pure pursuit, DISABLED zeros/brakes. Add a **deadman** (hold-to-drive)
  for autonomous.
- **Safety choreography:** wheels off the ground first, then slow on the ground,
  spotter present, hand on the kill switch, watchdog active.
- **Done when:** teach a loop, flip to autonomous, the car drives it.

### Demo  *(day 7)*
Video + a tight README + the eval plots.

---

## 7. Feasibility and risks

**Feasible for a focused team in a week — precisely because B is the floor.**
Full A lands if two risks cooperate; if they don't, you compress to B with a real
result in hand.

| Component | Effort | Risk | Why |
|---|---|---|---|
| KISS-ICP integration | ~0.5–1 day | Medium | Off-the-shelf, but point-cloud/timestamp wiring |
| Eval harness | ~0.5–1 day | Low | Pure Python on recorded bags |
| **Drift in the lot** | — | **High** | The roadmap's #1 risk; measured day 2 by design |
| Route record/replay | ~0.5 day | Low | Simple |
| **Pure-pursuit calibration** | ~1 day | **Medium** | Radians→normalized mapping + low-speed tuning |
| Integration + safety | ~1–2 days | Medium | Always the hidden tax |

The two deciding factors: **(1)** KISS-ICP drift in the environment — mitigated by
testing near structure and by the slow speed, and gated on day 2; **(2)**
pure-pursuit steering calibration — mitigated by budgeting the half-day and
starting on blocks.

---

## 8. Parallelizing across 3–4 people (after day 2)

- **Localization + eval** (B1/B2) — front-loaded, unblocks the gate.
- **Routing + control** (A1/A2) — develops against recorded **bags**,
  off-vehicle, the moment B2 produces them.
- **Integration + safety** — owns the gamepad mode switch, deadman, watchdog, and
  test-drive choreography.

Bags are the shared substrate: one recorded loop lets control development proceed
in parallel with everything else.

---

## 9. Why this is worth the week

It's a **vertical slice through the entire autonomy stack on real hardware,
validated with numbers** — rare, and exactly the profile AV teams look for. The
differentiator isn't "the car drove." It's being able to say: *"KISS-ICP drifted
0.4 m over a 30 m closed loop on textured pavement, degraded to 1.2 m on bare
asphalt because lidar geometry is unconstrained there, so I gated autonomous mode
on a drift estimate."* Measure things, keep the plots, write the failure modes
down.
