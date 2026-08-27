# xArm7 Shape Tracer — Avatar Robotics Code Challenge

ROS 2 (Humble) software that commands a simulated UFactory xArm 7 to trace a list
of 2D shapes in the air, each on its own 3D plane. Built and verified inside the
supplied `avatarrobotics/ros-humble-xarm:20250602` container.

One package, one build, two entry points:

| Command | What it does |
|---|---|
| `ros2 launch avatar_challenge start.launch.py` | **The challenge.** Traces `config/shapes.json` automatically. No web server. |
| `ros2 launch avatar_challenge designer.launch.py` | **Optional demo.** Same robot stack, with a browser designer for authoring shapes. |

Both drive the same tracing engine and the same JSON schema.

### Bonus features

- RViz visualisation of the target, planned and executed paths
- Circular arcs
- Clamped B-splines evaluated with De Boor's algorithm
- Continuous Cartesian trajectory blending
- Per-shape trajectory-speed scaling
- Optional browser shape designer

---

## 1. Build

**No Dockerfile, no image build, no second container.** The supplied
`avatarrobotics/ros-humble-xarm:20250602` image already has ROS 2 Humble, MoveIt,
the xArm packages, Python and NumPy. One container, one clone, one `colcon
build`, and `start.launch.py`.

Avatar's original Docker command is sufficient for the challenge workflow:

```bash
docker run --name xarm-container \
  --platform linux/amd64 \
  -p 5566:3389 \
  avatarrobotics/ros-humble-xarm:20250602
```

Connect over RDP to `localhost:5566` as user `dev`. Then, in a terminal inside
that desktop:

```bash
source /home/dev/xarm_ws/install/setup.bash

cd /home/dev/dev_ws/src
rm -rf avatar_challenge                       # the image ships a starter package
git clone https://github.com/dhruvah/avatar-challenge.git avatar_challenge

cd /home/dev/dev_ws
colcon build --packages-select avatar_challenge
source install/setup.bash
```

Removing `avatar_challenge` is appropriate **only in a fresh evaluation
container**, where it is the unmodified starter package. The repository root is
the ROS package, so the clone lands `package.xml` exactly where the starter one
was and the pre-built workspace reconfigures cleanly.

> `~/.bashrc` sources only `dev_ws`. `xarm_moveit_config`, `xarm_planner` and
> `xarm_msgs` live in `xarm_ws`, so it must be sourced first or the launch fails
> with *"package 'xarm_moveit_config' not found"*.

## 2. Run the included shapes

```bash
ros2 launch avatar_challenge start.launch.py
```

RViz opens with MoveIt and the arm traces the four sample shapes in
`config/shapes.json`: a rotated square, a triangle on a **vertical** plane, an
**arc**-based outline, and a **B-spline** curve at 30% speed. No browser, no HTTP
server, nothing to configure.

The RViz layout is included, so every display is already on:

| Display | Shows |
|---|---|
| Target shapes | Intended outlines, green |
| Actual tool path | Where the tool really went, orange, one marker per shape |
| MotionPlanning | The planned trajectory |

The node **holds after tracing** (`hold_after_trace`, default true) because the
marker publishers are latched — RViz keeps the outlines only while their
publisher is alive, so exiting immediately would clear the screen at the moment
there is something to look at. Ctrl-C when finished.

> **Running more than one of these containers at once?** ROS 2 discovery crosses
> container boundaries, so a second container's `move_group` will answer service
> calls meant for the first. Give each its own `ROS_DOMAIN_ID`. This looks
> exactly like a planning bug and is not one.

## 3. Use your own shapes

Pass any file to the launch, without editing or rebuilding:

```bash
ros2 launch avatar_challenge start.launch.py \
  shapes_file:=/absolute/path/to/my_shapes.json
```

Editing `config/shapes.json` and rebuilding is equally valid, and is what the
default launch traces.

This JSON file is the stable robot-facing interface.

```json
{
  "shapes": [
    {
      "name": "square_45deg",
      "vertices": [[0.0, 0.0], [0.0, 0.100], [0.100, 0.100], [0.100, 0.0]],
      "closed": true,
      "speed": 0.5,
      "start_pose": {
        "position": [0.300, -0.050, 0.250],
        "rpy": [0.0, 0.0, 0.7854]
      }
    }
  ]
}
```

| Field | Meaning |
|---|---|
| `vertices` | 2D points in the shape's own frame. **The first must be `(0, 0)`** — it is what `start_pose.position` pins. |
| `start_pose.position` | `[x, y, z]` in **metres**, in the robot's base frame |
| `start_pose.rpy` | `[roll, pitch, yaw]` in **radians**, REP-103 / tf2 `setRPY` (`Rz·Ry·Rx`) |
| `closed` | *(default `true`)* return to the first vertex to close the outline |
| `speed` | *(default `1.0`)* fraction of planned speed, in `(0, 1]` |

Metres and radians throughout, matching `geometry_msgs/Pose` and the xArm MoveIt
config, so nothing in the pipeline converts units and nothing can forget to.

**`speed` applies to the blended backend only.** It re-times the Cartesian
trajectory after planning; `xarm_planner`'s per-edge services expose no speed
control. With `blend:=false` the node warns, naming every shape whose `speed` it
is ignoring, rather than silently obeying half the file.

### Arcs and B-splines

A vertex entry may be a point, an arc, or a B-spline segment:

```jsonc
// arc from the previous vertex to arc_end, about arc_center
{ "arc_center": [0.080, 0.020], "arc_end": [0.100, 0.020], "clockwise": false }

// clamped B-spline; the previous vertex is the implicit first control point
{ "control_points": [[0.040, 0.090], [0.110, 0.070], [0.130, 0.0]], "degree": 3 }
```

Both tessellate into `arc_segments` (default 16) straight sub-segments.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `shapes_file` | — | Path to the shape JSON (required) |
| `lift_height` | `0.03` | Pen-up clearance along the plane normal |
| `arc_segments` | `16` | Tessellation resolution for curves |
| `closed` | `true` | Default for shapes that don't say |
| `blend` | `true` | One continuous trajectory vs. per-edge moves |
| `blend_max_step` | `0.005` | Cartesian interpolation step (m) |
| `home_on_start` | `true` | Move to the ready pose before tracing |
| `hold_after_trace` | `true` | Keep the node alive so RViz retains the markers |
| `service_timeout_sec` | `120.0` | Planner / IK service wait |

## 4. Optional: the visual designer

Not needed for the challenge workflow. Same build, different launch.

### Browser inside the RDP desktop — no extra port

```bash
ros2 launch avatar_challenge designer.launch.py
```

Open **<http://localhost:8080>** in a browser inside the RDP desktop. The server
binds loopback only, so nothing is exposed outside the container and the
original `docker run` above is sufficient.

### Browser on your host — one extra published port

Create the container with one additional host-loopback mapping:

```bash
docker run --name xarm-container \
  --platform linux/amd64 \
  -p 5566:3389 \
  -p 127.0.0.1:8080:8080 \
  avatarrobotics/ros-humble-xarm:20250602
```

Inside the container:

```bash
ros2 launch avatar_challenge designer.launch.py bind_address:=0.0.0.0
```

On your host, open **<http://localhost:8080>**.

This uses the **original Avatar image**. There is no `docker build`, no
Dockerfile, and no second application container. Binding the host side as
`127.0.0.1:8080:8080` keeps it off the LAN, and the server must bind `0.0.0.0`
*inside* the container so Docker's published port can reach it — which is why
the argument exists rather than being the default. `port:=` moves it if 8080 is
taken.

### If your container already exists without port 8080

Docker cannot add a published port to an existing container. The options are:

1. Use the browser inside the RDP desktop (no extra port needed).
2. Recreate the container with the mapping above.
3. Advanced fallback: `tools/docker_tunnel.py` forwards the port over
   `docker exec`. It needs Python and Docker CLI access on the host plus a host
   copy of that script, so it is a last resort rather than part of the workflow.

![The shape designer](docs/shape_designer.png)

Draw on a millimetre grid, watch the workspace shading, press **Send to robot**,
and the trace animates live over your drawing while the arm moves in RViz.

Orientation is set the way you would place a sheet of paper — **Tilt** how far
it leans, **Facing** which way it leans, **Spin** the drawing on the sheet. Roll,
pitch and yaw are derived from those and shown read-only, because they are the
same three degrees of freedom in the representation the JSON uses, not six
independent numbers.

The designer homes **once at startup and again before every submitted trace**, so
repeated sends of the same design produce the same motion instead of starting
from wherever the previous one stopped. Homing *between shapes within* one
submission is deliberately not done — it was measured as strictly worse.

**Send to robot is disabled while any point is out of reach**, with a line
naming the shape and how many of its points are outside the workspace. Marginal
and near-singular ground do not block — they are reachable.

The preview orbits by dragging and zooms by scrolling, with Iso / Top / Front /
Side presets and an Expand button when the small pane is not enough.

The shading has four bands and **only one of them blocks anything**:

| Band | Meaning |
|---|---|
| Grey — out of reach | No IK solution. Refused before the arm moves. |
| Red — near-singular | Reachable, but small tool motions need large joint speeds, so the arm lurches. |
| Amber — marginal | Reachable, less comfortable. |
| Clear — good | Clean working area. |

It comes from a sweep of 969 poses scored by manipulability, with the bands at
the measured 5th and 25th percentiles. It is a design-time guide — it assumes the
tool points along the plane normal and treats reach as a surface of revolution.
The node's IK preflight is what actually decides.

**Copy shapes.json** produces exactly the format in section 3.

## 5. Both entry points share one engine

`start.launch.py` and `designer.launch.py` bring up the same MoveIt stack, the
same `xarm_planner`, the same RViz layout and the same `ShapeTracerNode`. They
differ only in what feeds it shapes:

```
config/shapes.json ──┐
                     ├──► ShapeTracerNode ──► MoveIt ──► the arm
HTTP designer     ───┘
```

Only one runs at a time — they are separate launch files, and neither starts the
other's frontend. A fix to the geometry, the schema, the preflight or the
execution path applies to both because there is only one of each.

---

## Approach

### From a 2D shape to a 3D path

Each shape's plane is the local XY plane (`z = 0`) of the frame given by
`start_pose`. A vertex `(x, y)` maps to `position + R(rpy) · [x, y, 0]`, so
`(0, 0)` lands exactly on `start_pose.position` by construction.

**Tool orientation is constant across a shape**, equal to the plane frame's
rotation *flipped 180° about X* so the tool points **into** the plane — a pen
against a surface rather than away from it. Not cosmetic: un-flipped, a
horizontal shape is traced with the end-effector pointing straight up, which is
both backwards and measurably less reachable.

### Order of operations

Home to the ready pose → **then** per shape: preflight IK on every waypoint →
approach pen-up above the first vertex → descend → trace → lift.

Homing happens first, before any preflight, so the arm starts from a known
configuration; the reachability check runs before each shape's *own* motion. The
hover waypoints are what stop the tool dragging a line between shapes.

`start.launch.py` homes once per run. The designer homes at startup and again
before each submitted trace, since it accepts many traces in one session.

Homing retries a *rejection* — the trajectory controller can still be activating
while the planner's services already answer. It never retries a *timeout*: that
means the service did not answer, so it is unknown whether the motion started,
and re-commanding an arm that may already be moving is the wrong response.

### Two execution backends

**`blend: false`** — each edge is one `xarm_straight_plan` + `xarm_exec_plan`.
The `xarm_planner` node wraps `MoveGroupInterface` behind plain services, which
matters because this container ships **no Python MoveIt bindings** — there is no
supported Python route to `MoveGroupInterface`. Exact corners, but the arm stops
at every vertex.

**`blend: true` (default)** — the whole waypoint list goes to
`/compute_cartesian_path` in one call, producing one continuously
time-parameterised trajectory executed via `/execute_trajectory`. The arm never
stops mid-shape.

Two guards on that path:

- `jump_threshold` is deliberately **disabled**. It spuriously truncates valid
  paths near the 7-DoF wrist's redundant configurations.
- Because of that, completeness is enforced strictly instead: the service's
  error code must be success **and** `fraction` must be 1.0 to within 1e-9. A
  99.5% path is three and a bit sides of a square, so it is refused rather than
  drawn and reported as done.

### Deterministic motion

The free-space approach tries a straight line first and falls back to the
sampling planner only if that cannot be planned; across the sample shapes the
fallback is never needed. This matters because the xArm MoveIt config defaults
to **RRTConnect**, which produced a different meandering approach on every run.
Measured after the change: total joint travel over a full four-shape run was
identical on three consecutive runs.

### Preflight and failure handling

Every waypoint is checked against `/compute_ik` with collision avoidance before
that shape moves; an out-of-workspace shape is rejected naming the exact
coordinates. An IK service that does not *answer* raises a distinct error —
"MoveIt is down" and "that pose is unreachable" need different fixes.

A failed shape stops the run and is logged with the reason; remaining shapes are
not attempted. It deliberately does **not** command a "safe lift" afterwards:
the controller's state is unknown after an abort, and blind motion from an
unknown state is worse than stopping. Recovery is re-running, which begins by
homing.

If an execution goal is accepted and then never completes, the node cancels it
rather than exiting — a dead node does not stop the controller.

### Singularity reporting

`kinematics.py` derives FK, the geometric Jacobian and Yoshikawa manipulability
straight from the URDF in plain numpy. Every trace logs its minimum
manipulability and warns below `0.010`. A pose can have a perfectly good IK
solution and still sit where small tool motions demand large joint speeds.

---

## Verification

| What | Result |
|---|---|
| `kinematics.py` FK vs MoveIt `/compute_fk` | **0.000000 mm** |
| Path deviation from the requested outline | **0.04 mm** mean, 0.73 mm max |
| Vertex accuracy, `blend: false` | **0.01 mm** |
| Vertex accuracy, `blend: true` | **2.20 mm** — blending rounds corners |
| Run-to-run repeatability | identical joint travel across 3 runs |
| Trajectory suite | **240/240** planned, 0 unreachable, 0 failures |

```bash
colcon test --packages-select avatar_challenge
colcon test-result --all --verbose      # 5 suites, 98 tests
```

The suite is deliberately small and needs neither ROS nor a robot: the geometry
properties, three synthetic kinematics checks, and a regression for each bug
actually found — a partial Cartesian plan never reaching the controller, an
accepted goal that times out requesting cancellation, re-timed nanoseconds being
normalised, invalid shape names rejected, two simultaneous designer submissions
admitting exactly one, invalid input preserving the last valid design, and an
aborted trace reporting failure rather than 100%.

The trajectory sweep, the browser audit (`tools/ui/run_ui_audit.py`, 99 checks)
and the TF-based path measurements are tools rather than tests: they need a
running robot, so `colcon test` does not run them.

### What the 240-trajectory sweep showed

6 shape types × 2 sizes × 5 positions × 4 orientations:

- **Plane tilt dominates cost** — 0.76° of joint motion per mm of tool motion on
  a table, 1.30° on a wall.
- **Shape type barely matters** (0.79–0.95), so curves are not the difficulty.
- **60% of trajectories sit within 0.1° of the same maximum joint step**, which
  is the time parameteriser running joints at their velocity limit. The motion
  is already time-optimal; lowering `speed` is what smooths it, not adding
  waypoints.
- Worst manipulability observed was 0.0091, close to the base on a vertical
  plane.

### The challenge PDF's example pose is unreachable

The illustrative square starts at `(0.050, 0, 0.150)` — inside the robot's own
base column, with no IK solution at any tool orientation tested. The sample
shapes keep the PDF's geometry (100 mm square, 45° about Z) at a reachable
`x = 0.300`.

---

## Assumptions and limitations

- "The robot's 3D coordinate space" means the base frame. `world` and
  `link_base` are related by identity in this MoveIt config (checked with
  `tf2_echo`), so both readings coincide.
- Tool orientation is constant per shape; a plane whose normal varies along the
  path is out of scope.
- No collision checking *between* shapes beyond MoveIt's own planning scene.
- The workspace sweep covered three tilts (0°, 45°, 90°). It characterises the
  workspace; it does not certify an individual pose. The IK preflight does that.
- The designer holds one "smooth" flag per shape, so importing a shape built
  from several separate B-spline runs merges them — it warns when it does.
- Measurements were taken in simulation, in this container, on an amd64 image
  under emulation. Timings are indicative; geometric results are not.

---

## Layout

The repository root *is* the ROS package, so cloning it straight into
`dev_ws/src/avatar_challenge` puts `package.xml` exactly where the supplied
starter package was.

```
package.xml, CMakeLists.txt
avatar_challenge/         the Python package
  geometry.py             rotations, plane transform, arc + B-spline tessellation
  shapes_io.py            JSON loading and strict validation
  blended_path.py         Cartesian planning, re-timing, execution
  kinematics.py           FK, Jacobian, manipulability from the URDF
  shape_tracer_node.py    preflight, motion strategy, RViz output
  designer_server.py      HTTP frontend; everything browser-facing lives here
config/shapes.json        the four sample shapes
launch/                   start.launch.py, designer.launch.py
rviz/shape_tracer.rviz    layout with the displays already added
web/shape_designer.html   the designer page
test/                     98 tests, no robot required
tools/
  verify_path.py          records link_eef from TF, measures deviation from target
  path_fidelity.py        executed vs requested path, in the units you drew in
  trajectory_suite.py     plans a batch of shapes and scores execution quality
  workspace_quality.py    the manipulability sweep behind the overlay
  lift_check.py           confirms the pen lifts between disconnected shapes
  docker_tunnel.py        optional: forward the port to a host browser
  ui/run_ui_audit.py      drives the designer's real handlers against a fake DOM
```

Generated sweep outputs are not committed; re-run the tools to reproduce them.

## Possible next steps

- Corner blending with an explicit geometric fillet and a TOPP-RA pass, so the
  corner radius is specified rather than falling out of the interpolation.
- Sweeping the workspace at finer tilt resolution.
