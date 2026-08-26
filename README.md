# xArm7 Shape Tracer — Avatar Robotics Code Challenge

ROS 2 (Humble) software that commands a simulated UFactory xArm 7 to trace a list
of 2D shapes in the air, each on its own 3D plane. Built and verified against the
`avatarrobotics/ros-humble-xarm:20250602` challenge container.

Every number quoted below was measured in that container. Where something is an
assumption or a limitation, it says so.

---

## Quick start

```bash
docker run --name xarm-container --platform linux/amd64 \
  -v "$(pwd)/avatar_challenge:/home/dev/dev_ws/src/avatar_challenge" \
  -p 5566:3389 avatarrobotics/ros-humble-xarm:20250602
```

Connect over RDP to `localhost:5566` (user `dev`, password `ecstatic-robots`), then
in a terminal **inside** that desktop:

```bash
# ~/.bashrc sources only dev_ws; xarm_moveit_config, xarm_planner and xarm_msgs
# live in xarm_ws, so it must be sourced first or the launch cannot find them.
source /home/dev/xarm_ws/install/setup.bash
cd /home/dev/dev_ws
colcon build --packages-select avatar_challenge
source install/setup.bash

ros2 launch avatar_challenge start.launch.py
```

RViz opens and the arm traces every shape in `config/shapes.json`.

> **Running more than one of these containers?** ROS 2 discovery crosses container
> boundaries, so a second container's `move_group` will answer service calls meant
> for the first and you will see duplicate nodes. Give each its own
> `ROS_DOMAIN_ID`. This cost real debugging time and looks exactly like a planning
> bug.

### RViz displays

| Topic | Shows |
|---|---|
| `shape_tracer/target_shapes` | Intended outlines (green), latched |
| `/display_planned_path` | The planned trajectory, animated by the MotionPlanning display |

`tools/verify_path.py` records the tool path from TF and measures it against the
target, which is the rigorous version of the same comparison.

---

## Input format

Shapes live in `config/shapes.json`, selected by the `shapes_file` parameter.

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

- `vertices` — 2D points in the shape's local frame. **The first must be `(0, 0)`**;
  it is what `start_pose.position` pins. Units are **metres**.
- `start_pose.position` — `[x, y, z]` in metres, in the robot's base frame.
- `start_pose.rpy` — `[roll, pitch, yaw]` in radians, REP-103 / tf2 `setRPY`
  convention (`Rz(yaw) · Ry(pitch) · Rx(roll)`).
- `closed` *(default `true`)* — return to the first vertex to close the outline.
- `speed` *(default `1.0`)* — fraction of planned speed, in `(0, 1]`.

Metres and radians throughout, because that is what `geometry_msgs/Pose` and the
xArm MoveIt config natively use — so there is no unit conversion anywhere in the
pipeline and nowhere for one to be forgotten.

### Segment types

A vertex entry can be a point, an **arc**, or a **B-spline**:

```jsonc
{ "arc_center": [0.080, 0.020], "arc_end": [0.100, 0.020], "clockwise": false }
{ "control_points": [[0.040, 0.090], [0.110, 0.070], [0.130, 0.0]], "degree": 3 }
```

Both are tessellated into `arc_segments` (default 16) straight sub-segments.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `shapes_file` | — | Path to the shape JSON |
| `lift_height` | `0.03` | Pen-up clearance along the plane normal |
| `arc_segments` | `16` | Tessellation resolution for curves |
| `closed` | `true` | Default for shapes that don't say |
| `blend` | `true` | One continuous trajectory vs. per-edge moves |
| `blend_min_fraction` | `0.99` | Reject a Cartesian plan below this completeness |
| `home_on_start` | `true` | Move to the ready pose before tracing |
| `service_timeout_sec` | `120.0` | Planner / IK service wait |

---

## Approach

### From a 2D shape to a 3D path

Each shape's plane is the local XY plane (`z = 0`) of the frame given by
`start_pose`. A vertex `(x, y)` maps to `position + R(rpy) · [x, y, 0]`, so `(0,0)`
lands exactly on `start_pose.position` by construction.

**Tool orientation is constant across a shape**, equal to the plane frame's rotation
*flipped 180° about X* so the tool points **into** the plane — a pen against a
surface rather than away from it. This is not cosmetic: with the un-flipped
orientation a horizontal shape is traced with the end-effector pointing straight
up, which is both backwards and measurably less reachable.

### Motion strategy

Ready pose → approach pen-up above the first vertex → descend → trace → lift.
The hover waypoints are what stop the tool dragging a line between shapes.

### Two execution backends

**`blend: false`** — each edge is one `xarm_straight_plan` + `xarm_exec_plan`. The
`xarm_planner` node wraps `MoveGroupInterface` behind plain services, which matters
because this container ships **no Python MoveIt bindings** — there is no supported
Python path to `MoveGroupInterface` directly. Exact, but the arm stops at every
vertex.

**`blend: true` (default)** — the whole waypoint list goes to
`/compute_cartesian_path` in one call, producing one continuously
time-parameterised trajectory executed via `/execute_trajectory`. The arm never
stops mid-shape. `jump_threshold` is disabled because it spuriously truncates
valid paths near the 7-DoF wrist's redundant configurations; the plan is instead
rejected outright below `blend_min_fraction`, so a partial path is never executed
as though complete.

### Speed

`GetCartesianPath` has no velocity-scaling field in Humble, so the trajectory is
uniformly re-timed after planning. Stretching every timestamp by the same factor
leaves the geometric path untouched and only changes traversal rate. Verified on
the robot: 30% speed took a trace from ~1.65 s to 5.52 s.

### Preflight checks

Before any motion, every waypoint is checked against `/compute_ik` with collision
avoidance on. A shape outside the workspace is rejected up front, naming the exact
offending coordinates, rather than failing mid-trace and leaving the arm parked
half-drawn. An IK service that does not answer raises a *distinct* error, because
"MoveIt is down" and "that pose is unreachable" need different fixes.

The node also reports manipulability per shape and warns below `0.010`.

### Recovery policy

A failed shape stops the run and is logged with the reason; remaining shapes are
not attempted. It deliberately does **not** command a "safe lift" afterwards — the
controller's state is unknown after an abort, and blind motion from an unknown
state is worse than stopping. Recovery is re-running, which begins by homing.

---

## Interactive shape designer (`demo-designer` branch)

A browser-based designer — draw shapes on a millimetre grid, see the workspace
shaded by measured manipulability, send them straight to the arm over HTTP, and
watch the trace animate live — lives on the **`demo-designer`** branch, together
with its ROS HTTP service, a `docker exec` port forwarder, and a fake-DOM UI test
harness (99 checks).

It is kept off `master` deliberately. This submission is about the robotics, and
1,400 lines of browser code would sit in front of that rather than beside it. The
measurement underneath it — the workspace sweep in `tools/workspace_quality.py` —
is on `master`, because that part is robotics rather than UI.

```bash
git checkout demo-designer
```

## Verification

| What | Result |
|---|---|
| Path deviation from the requested outline | **0.04 mm** mean, 0.73 mm max |
| Vertex accuracy, `blend: false` | **0.01 mm** |
| Vertex accuracy, `blend: true` | **2.20 mm** (corners rounded by blending) |
| Run-to-run repeatability | **identical** joint travel across 3 runs |
| Trajectory suite | **240/240** planned, 0 unreachable, 0 failures |
| `kinematics.py` FK vs MoveIt `/compute_fk` | **0.000000 mm** |

```bash
colcon test --packages-select avatar_challenge   # 106 tests
```

### What the trajectory sweep showed

240 cases across 6 shape types, 2 sizes, 5 positions and 4 orientations:

- **Plane tilt dominates cost.** 0.76° of joint motion per mm of tool motion on a
  table, 1.30° on a wall — a vertical plane makes the arm work ~70% harder.
- **Shape type barely matters** (0.79–0.95), so curves are not the difficulty.
- **60% of trajectories sit within 0.1° of the same maximum joint step**, which is
  the time parameteriser running joints at their velocity limit. The motion is
  already time-optimal; lowering `speed` is what smooths it, not more waypoints.
- Worst manipulability seen was 0.0091, close to the base on a vertical plane.

### The challenge PDF's example pose is unreachable

The illustrative square starts at `(0.050, 0, 0.150)` — inside the robot's own base
column, with no IK solution at any tool orientation tested. The example shapes keep
the PDF's geometry (100 mm square, 45° about Z) at a reachable `x = 0.300`.

---

## Bonus features

- **RViz visualisation** — target outlines, actual traced path, and the planned
  trajectory.
- **Circular arcs** — centre + endpoint + direction, validated for radius
  consistency.
- **B-splines** — clamped uniform, arbitrary degree, De Boor evaluation. The clamped
  knot vector makes the curve interpolate its endpoints, which is what lets spline
  segments chain with points and arcs.
- **Blending** — one continuous Cartesian trajectory per shape.
- **Per-shape speed** and **singularity reporting** — every trace logs its
  minimum manipulability and warns when it passes close to a singularity.

---

## Assumptions and limitations

- "The robot's 3D coordinate space" means the base frame. `world` and `link_base`
  are related by identity in this MoveIt config (checked with `tf2_echo`), so the
  two readings coincide.
- Tool orientation is constant per shape; a plane whose normal varies along the
  path is out of scope.
- No collision checking *between* shapes beyond MoveIt's own planning scene.
- The workspace sweep covered three tilts (0°, 45°, 90°); it characterises the
  workspace, it does not certify any individual pose. The IK preflight does that.

---

## Layout

```
avatar_challenge/
  avatar_challenge/
    geometry.py          rotations, plane transform, arc + B-spline tessellation
    shapes_io.py         JSON loading and strict validation
    blended_path.py      Cartesian planning, re-timing, execution
    kinematics.py        FK, Jacobian, manipulability from the URDF
    shape_tracer_node.py the node: preflight, motion strategy, RViz output
  test/                  106 tests, no robot required
tools/                   measurement scripts behind the numbers above
```

| Tool | Purpose |
|---|---|
| `verify_path.py` | Records `link_eef` from TF and measures deviation from target |
| `workspace_quality.py` | The 969-pose manipulability sweep behind the overlay |
| `trajectory_suite.py` | Plans a large batch of shapes and scores execution quality |

Generated sweep outputs are not committed; re-run the tools to regenerate them.

## Possible next steps

- Corner blending with an explicit geometric fillet and a TOPP-RA pass, so the
  corner radius is specified rather than falling out of the interpolation.
- Sweeping the workspace map at finer tilt resolution.
