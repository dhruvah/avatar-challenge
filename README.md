# xArm7 Shape Tracer — Avatar Robotics Code Challenge

ROS2 (Humble) software that commands a simulated UFactory xArm7 to trace a
list of 2D shapes in the air, each on its own 3D plane. Built against the
`avatarrobotics/ros-humble-xarm:20250602` challenge container.

## Running it

1. Pull and start the challenge container per the [challenge README](https://github.com/AvatarRobotics/ros-humble-xarm/blob/20250602/README.md),
   but bind-mount this repo's `avatar_challenge/` over the stub package instead
   of copying files in by hand, so edits here are picked up immediately:

   ```bash
   docker run --name xarm-container --platform linux/amd64 \
     -v "$(pwd)/avatar_challenge:/home/dev/dev_ws/src/avatar_challenge" \
     -p 5566:3389 \
     avatarrobotics/ros-humble-xarm:20250602
   ```

2. Connect via RDP (Remmina / Windows App) to `localhost:5566`, user `dev`.

3. In a terminal inside the container:

   ```bash
   # NOTE: the container's ~/.bashrc only sources dev_ws/install/setup.bash,
   # which does NOT chain in xarm_ws (where xarm_moveit_config, xarm_planner,
   # and xarm_msgs actually live). Source xarm_ws first, or `ros2 launch`
   # will fail with "package 'xarm_moveit_config' not found".
   source /home/dev/xarm_ws/install/setup.bash

   cd /home/dev/dev_ws
   colcon build --packages-select avatar_challenge
   source install/setup.bash

   ros2 launch avatar_challenge start.launch.py
   ```

   RViz opens with MoveIt; you'll see the xArm7 trace each shape in
   `config/shapes.json` in sequence, and a green outline of each shape's
   target path published as `visualization_msgs/MarkerArray` on
   `/shape_tracer/target_shapes` (add a Marker Array display in RViz, topic
   `shape_tracer/target_shapes`, to see it).

## Input format

Shapes are defined in `avatar_challenge/config/shapes.json`, pointed to by
the `shapes_file` launch parameter (edit the path in `start.launch.py`, or
override it: `ros2 launch avatar_challenge start.launch.py
shape_tracer_node:shapes_file:=/path/to/other.json` — or simpler, just edit
`config/shapes.json` directly and re-run, since `colcon build` picks up the
mounted file).

```json
{
  "shapes": [
    {
      "name": "square_45deg",
      "vertices": [[0.0, 0.0], [0.0, 0.100], [0.100, 0.100], [0.100, 0.0]],
      "start_pose": {
        "position": [0.050, 0.0, 0.150],
        "rpy": [0.0, 0.0, 0.7854]
      }
    }
  ]
}
```

- `vertices`: 2D points in the shape's local frame, **first vertex always
  `(0, 0)`**, units are meters. A vertex entry can also be an arc segment
  (bonus feature, see below).
- `start_pose.position`: `[x, y, z]` in meters, in the xArm's base frame
  (`link_base`).
- `start_pose.rpy`: `[roll, pitch, yaw]` in radians (ROS REP-103 / tf2
  `setRPY` convention — intrinsic rotations applied as
  `Rz(yaw) * Ry(pitch) * Rx(roll)`).
- `closed` (optional, per-shape, default `true`): if `true`, the tracer
  returns to the first vertex after the last one, closing the outline.
- Top-level launch parameters (`lift_height`, `arc_segments`,
  `service_timeout_sec`) tune pen-up clearance, arc tessellation
  resolution, and how long to wait on `xarm_planner`'s services.

Units are meters and radians throughout — chosen because that's what
`geometry_msgs/Pose` and the xArm's MoveIt config natively use, so no
unit conversion is needed anywhere in the pipeline.

## Approach

**How a shape becomes a trajectory.** The shape's plane is defined as the
local XY plane (z=0) of a 3D frame given by `start_pose` (`position` +
`rpy`). Each 2D vertex `(x, y)` is mapped into the base frame as
`position + R(rpy) @ [x, y, 0]`. The end-effector's orientation is held
**constant** for the whole shape, equal to `R(rpy)` — i.e. the tool's local
+Z axis is assumed to point along the plane's normal, like a pen held
perpendicular to a whiteboard, and doesn't rotate as it traces the outline.
That assumption is baked into the example format (the example in the
challenge PDF rotates the shape only about Z and gives no separate
per-vertex orientation), and it's the simplest interpretation of "draw the
shape according to the rotation."

**Why `xarm_planner`, not raw MoveGroup.** The container ships ROS2 Humble
without any Python MoveIt bindings (no `moveit_commander`, no `moveit_py`).
Rather than write a raw `rclpy` action client against MoveGroup's action
interface, the xArm repo already ships `xarm_planner_node`, which wraps
`MoveGroupInterface` behind plain ROS2 services purpose-built for exactly
this: `xarm_pose_plan` (free-space plan to a target pose), `xarm_straight_plan`
(Cartesian straight-line plan from the current pose to a target pose,
preserving orientation), and `xarm_exec_plan` (execute the last planned
trajectory). `start.launch.py` brings this node up alongside the existing
MoveIt/RViz launch by including `xarm_planner`'s `_robot_planner.launch.py`.

**Trajectory strategy per shape**: pen-up, travel, pen-down, draw, pen-up.
1. Approach a "hover" pose above the first vertex (offset along the plane
   normal by `lift_height`, default 3cm) using a free-space plan
   (`xarm_pose_plan`) — safe regardless of where the arm currently is.
2. Move straight down onto the first vertex (`xarm_straight_plan`).
3. Trace each subsequent vertex in order with straight-line Cartesian moves
   (one `xarm_straight_plan` + `xarm_exec_plan` per edge — the planner only
   accepts point-to-point Cartesian goals, so a polygon's edges are executed
   as a sequence of individual straight-line moves rather than one combined
   trajectory).
4. If `closed`, return to the first vertex to close the outline.
5. Lift straight back up off the plane to the hover pose before moving on to
   the next shape.

This avoids the pen dragging a spurious line between the end of one shape
and the start of the next, without needing anything fancier than the two
service calls the planner already exposes.

**Assumptions**:
- Shape coordinates and poses are expressed directly in the xArm's base
  link frame (`link_base`), i.e. "the robot's 3D coordinate space" from the
  prompt is taken to mean the base frame, not a task/world frame requiring
  an extra static transform.
- The 2D → 3D projection assumes the vertex list always starts at `(0, 0)`
  and that `(0, 0)` maps exactly onto `start_pose.position` — verified by
  construction, not just assumed.

## Bonus features implemented

- **RViz visualization**: each shape's outline is published as a
  `visualization_msgs/MarkerArray` (`LINE_STRIP`) on
  `/shape_tracer/target_shapes`, so the intended path is visible alongside
  the arm's actual motion for visual comparison.
- **Circular arcs**: a vertex entry can be
  `{"arc_center": [cx, cy], "arc_end": [x, y], "clockwise": bool}` instead
  of a plain `[x, y]` point — an arc from the previous vertex to `arc_end`,
  centered at `arc_center`. Since the Cartesian planner only accepts
  straight-line point-to-point goals, arcs are tessellated into
  `arc_segments` (default 16) short straight segments — a standard
  technique for planners limited to straight-line moves. See the
  `rounded_tab` shape in `config/shapes.json` for an example.

**Not implemented** (out of scope for the time budget): B-splines, and
blending between segments. Blending in particular would require replacing
the per-segment plan-then-execute-to-a-stop pattern with a single merged
`JointTrajectory` (e.g. via MoveIt's Cartesian path API with a velocity
scaling pass, or manual `TOPP-RA` time-parameterization across all segments)
rather than `xarm_planner`'s one-target-at-a-time services — noted here as
the natural next step rather than attempted partially.

## Code layout

```
avatar_challenge/
  avatar_challenge/           # Python package (installed via ament_cmake_python)
    geometry.py                # RPY<->quaternion/matrix math, waypoint + arc tessellation
    shapes_io.py                # JSON shape-list loading/validation
    shape_tracer_node.py        # rclpy node: calls xarm_planner services, publishes RViz markers
  scripts/shape_tracer_node.py  # thin executable entry point
  launch/start.launch.py        # MoveIt/RViz + xarm_planner_node + shape_tracer_node
  config/shapes.json             # example shape list (square from the prompt, triangle, arc shape)
  package.xml / CMakeLists.txt
```
