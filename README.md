# xArm7 Shape Tracer — Avatar Robotics Code Challenge

ROS 2 (Humble) software that commands a simulated UFactory xArm 7 to trace a
list of 2D shapes in the air, each on its own 3D plane. Built and verified
against the `avatarrobotics/ros-humble-xarm:20250602` challenge container.

Every result quoted in this README was measured in that container, not assumed —
see [Verification](#verification).

## Running it

1. Start the challenge container per the [challenge README](https://github.com/AvatarRobotics/ros-humble-xarm/blob/20250602/README.md),
   bind-mounting this repo's `avatar_challenge/` over the stub package so edits
   are picked up without copying files by hand:

   ```bash
   docker run --name xarm-container --platform linux/amd64 \
     -v "$(pwd)/avatar_challenge:/home/dev/dev_ws/src/avatar_challenge" \
     -p 5566:3389 \
     avatarrobotics/ros-humble-xarm:20250602
   ```

2. Connect via RDP to `localhost:5566` as user `dev` (password `ecstatic-robots`).

3. In a terminal inside the container:

   ```bash
   # The container's ~/.bashrc sources only dev_ws/install/setup.bash, which
   # does NOT chain in xarm_ws (where xarm_moveit_config, xarm_planner and
   # xarm_msgs live). Source xarm_ws first or the launch fails with
   # "package 'xarm_moveit_config' not found".
   source /home/dev/xarm_ws/install/setup.bash

   cd /home/dev/dev_ws
   colcon build --packages-select avatar_challenge
   source install/setup.bash

   ros2 launch avatar_challenge start.launch.py
   ```

RViz opens with MoveIt and the arm traces each shape in `config/shapes.json` in
sequence. To see the intended outlines overlaid on the arm's actual motion, add
a **MarkerArray** display on topic `/shape_tracer/target_shapes`.

> **If you run more than one of these containers at once**, note that ROS 2's
> default DDS discovery crosses container boundaries. A second container running
> its own `move_group` / `xarm_planner_node` will answer service calls meant for
> the first, and you will see duplicate nodes in `ros2 node list`. Give each
> container its own `ROS_DOMAIN_ID` (e.g. `export ROS_DOMAIN_ID=42`) before
> launching. This bit us during development and is easy to misdiagnose as a
> planning bug.

## Input format

Shapes live in `avatar_challenge/config/shapes.json`, selected by the
`shapes_file` parameter. To use a different file:

```bash
ros2 launch avatar_challenge start.launch.py   # edit config/shapes.json, or:
ros2 run avatar_challenge shape_tracer_node.py --ros-args \
  -p shapes_file:=/path/to/your_shapes.json
```

```json
{
  "shapes": [
    {
      "name": "square_45deg",
      "vertices": [[0.0, 0.0], [0.0, 0.100], [0.100, 0.100], [0.100, 0.0]],
      "closed": true,
      "start_pose": {
        "position": [0.300, -0.050, 0.250],
        "rpy": [0.0, 0.0, 0.7854]
      }
    }
  ]
}
```

- `vertices` — 2D points in the shape's local frame, **first vertex always
  `(0, 0)`** (validated on load). Units are **meters**. An entry may also be an
  arc or B-spline segment (below).
- `start_pose.position` — `[x, y, z]` in meters, in the robot's base frame.
- `start_pose.rpy` — `[roll, pitch, yaw]` in radians, ROS REP-103 / tf2
  `setRPY` convention (`Rz(yaw) · Ry(pitch) · Rx(roll)`).
- `closed` *(optional, default `true`)* — return to the first vertex to close
  the outline.

Units are meters and radians throughout, because that is what
`geometry_msgs/Pose` and the xArm MoveIt config natively use — so there is no
unit conversion anywhere in the pipeline and no place for one to be forgotten.

### Segment types

A vertex entry can be a plain point, an **arc**, or a **B-spline**:

```jsonc
// circular arc from the previous vertex to arc_end, centered at arc_center
{ "arc_center": [0.080, 0.020], "arc_end": [0.100, 0.020], "clockwise": false }

// clamped B-spline; the previous vertex is the implicit first control point
{ "control_points": [[0.040, 0.090], [0.110, 0.070], [0.130, 0.0]], "degree": 3 }
```

Both are tessellated into `arc_segments` (default 16) straight sub-segments.
Arc specs are validated for consistency — a center that is not equidistant from
the arc's start and end is rejected rather than silently distorted.

### Tunable parameters

| Parameter | Default | Meaning |
|---|---|---|
| `shapes_file` | *(required)* | Path to the shape JSON |
| `lift_height` | `0.03` | Pen-up clearance along the plane normal |
| `arc_segments` | `16` | Tessellation resolution for arcs and B-splines |
| `closed` | `true` | Default for shapes that don't specify `closed` |
| `blend` | `true` | Single continuous trajectory vs. per-edge moves |
| `blend_max_step` | `0.005` | Cartesian interpolation resolution (m) |
| `blend_min_fraction` | `0.99` | Reject a Cartesian plan below this completeness |
| `service_timeout_sec` | `120.0` | Planner/IK service wait |

## Approach

### From a 2D shape to a 3D path

Each shape's plane is the local XY plane (`z = 0`) of the frame given by
`start_pose`. A 2D vertex `(x, y)` maps into the base frame as
`position + R(rpy) · [x, y, 0]`, so `(0, 0)` lands exactly on
`start_pose.position` by construction.

**Tool orientation is held constant across a shape**, and equals the plane
frame's rotation *flipped 180° about X* so the tool's +Z points **into** the
plane — a pen held against a surface rather than pointing away from it. This
matters concretely: with the un-flipped orientation a horizontal shape is traced
with the end-effector pointing straight up, which is both physically backwards
and measurably less reachable (see the workspace probe below). The challenge's
example rotates only about Z and gives no per-vertex orientation, so holding
orientation fixed is the natural reading of "draw the shape according to the
rotation."

### Motion strategy: pen-up, travel, pen-down, draw, lift

1. Free-space plan (`xarm_pose_plan`) to a hover pose offset `lift_height` along
   the plane normal above the first vertex — safe from wherever the arm is.
2. Descend, trace the outline, and lift back off the plane.
3. Repeat for the next shape.

The hover approach is what stops the tool from dragging a spurious line between
the end of one shape and the start of the next.

### Two execution backends

**`blend: false` — per-edge.** Each segment is one
`xarm_straight_plan` + `xarm_exec_plan` pair. The `xarm_planner` node wraps
`MoveGroupInterface` behind plain services, which matters because this container
ships **no Python MoveIt bindings** (no `moveit_commander`, no `moveit_py`), so
there is no supported Python path to `MoveGroupInterface` directly. Exact, but
the arm decelerates to a stop at every vertex.

**`blend: true` (default) — one trajectory per shape.** The whole waypoint list
goes to MoveIt's `/compute_cartesian_path` in a single call, producing one
continuously time-parameterized `RobotTrajectory` executed via the
`/execute_trajectory` action. The arm never stops mid-shape; corners are rounded
by the interpolation and time parameterization. `jump_threshold` is disabled
(`0.0`) because the check spuriously truncates otherwise-valid paths near the
7-DoF wrist's redundant configurations; the plan is instead rejected outright if
`fraction < blend_min_fraction`, so a partial path is never executed as if it
were complete.

This is a genuine accuracy/smoothness tradeoff, and it is measured rather than
hand-waved — see below.

### Preflight reachability check

Before any motion, every waypoint is checked against MoveIt's `/compute_ik`
(with collision avoidance on). A shape that is out of the workspace is rejected
up front, naming the offending points and their coordinates. Without this, the
planner's own failure surfaces only once the arm is already mid-shape, leaving
it parked in a half-drawn pose with an opaque "planner reported failure".

## Verification

Correctness is measured, not assumed. A test harness records `link_eef` from TF
at 200 Hz during a run and compares the **actual** path against every target
vertex. To be independent of sampling rate, it measures each vertex's distance
to the *polyline through consecutive samples*, not to the nearest sample point.

```bash
# in one terminal, record for 90s while a trace runs in another
python3 tools/verify_path.py 90 <path-to-shapes.json>
python3 tools/workspace_probe.py     # the reachability grid below
```

Across all four example shapes:

| Mode | Worst-case deviation from target vertices |
|---|---|
| `blend: false` (per-edge) | **0.01 mm** |
| `blend: true` (single trajectory) | **2.20 mm** |

That is the tradeoff in one line: per-edge motion hits vertices essentially
exactly but stops at each one; blended motion moves continuously and cuts
corners by ~2 mm.

### The challenge PDF's example pose is unreachable

The PDF's illustrative square starts at `(0.050, 0, 0.150)` — 50 mm in front of
the base, 150 mm up. That pose is **inside the robot's own base column and has
no IK solution.** A workspace probe over a grid of positions and three tool
orientations confirmed it:

```
=== tool_down (Rx180) ===         x=0.05  0.15  0.25  0.35  0.45  0.55
z=0.10                              --    ok    ok    ok    ok    ok
z=0.15                              --    ok    ok    ok    ok    ok
z=0.25                              --    ok    ok    ok    ok    ok
z=0.35                              ok    ok    ok    ok    ok    ok
```

`x = 0.05` fails at low `z` for *every* orientation tested. The example shapes
therefore keep the PDF's geometry (the 100 mm square, 45° about Z) but place it
at a reachable `x = 0.300`. The preflight IK check exists so this class of
problem reports itself clearly instead of appearing as a planning failure.

## Bonus features implemented

- **RViz visualization** — each shape's target outline is published as a
  `LINE_STRIP` `MarkerArray` on `/shape_tracer/target_shapes`, with
  `TRANSIENT_LOCAL` (latched) QoS so RViz still receives it when it subscribes
  after the node has already published.
- **Circular arcs** — specified by center + endpoint + direction, validated for
  radius consistency, tessellated into straight sub-segments.
- **B-splines** — clamped uniform B-splines of arbitrary degree, evaluated with
  De Boor's algorithm. The clamped knot vector makes the curve interpolate its
  first and last control points, which is what lets spline segments chain
  seamlessly with points and arcs. Unit-checked for endpoint interpolation and
  for degree-1 reducing to a straight line.
- **Blending between adjacent segments** — the `blend` mode above: one
  continuous Cartesian trajectory per shape rather than a stop at every vertex.

## Assumptions

- "The robot's 3D coordinate space" means the arm's base frame. `world` and
  `link_base` are related by identity in this MoveIt config (verified with
  `tf2_echo`), so the base-frame and planning-frame interpretations coincide and
  nothing hinges on the choice.
- Shapes are traced in the order listed, with no collision checking *between*
  shapes beyond what MoveIt's own planning scene provides.
- Tool orientation is constant per shape; a shape whose plane normal varies
  along the path is out of scope.

## Code layout

```
avatar_challenge/
  avatar_challenge/
    geometry.py         # rotations, plane transform, arc + B-spline tessellation
    shapes_io.py        # JSON loading and validation
    blended_path.py     # /compute_cartesian_path + /execute_trajectory backend
    shape_tracer_node.py# rclpy node: preflight IK, motion strategy, RViz markers
  scripts/shape_tracer_node.py  # executable entry point
  launch/start.launch.py        # MoveIt/RViz + xarm_planner_node + tracer
  config/shapes.json            # four example shapes (square, triangle, arc, B-spline)
```

## Possible next steps

- Corner blending with an explicit geometric fillet and a TOPP-RA
  time-parameterization pass, which would let the corner radius be specified
  directly rather than falling out of the interpolation.
- Velocity/acceleration scaling exposed as parameters for tuning trace speed.
