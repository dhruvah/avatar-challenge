"""Rotation math and shape-to-waypoint conversion.

All rotation math is implemented directly with numpy (roll/pitch/yaw ->
quaternion / rotation matrix, ROS REP-103 convention: intrinsic rotations
applied in the order Rz(yaw) * Ry(pitch) * Rx(roll)) instead of pulling in
tf_transformations/scipy, since neither is guaranteed present in the
challenge container and this math is small enough to own directly.
"""

from dataclasses import dataclass, field
from typing import List, Sequence, Union

import numpy as np


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert roll/pitch/yaw (radians) to a (x, y, z, w) quaternion.

    Matches tf2's Quaternion::setRPY / ROS REP-103 convention.
    """
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def quaternion_multiply(a, b):
    """Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


# Rotation of pi about X: flips the tool's +Z to point *into* the shape's plane
# (the plane normal is the frame's +Z, and a pen points opposite its surface
# normal). Without this a horizontal shape is traced with the tool pointing
# straight up, which is both physically backwards and less reachable.
TOOL_FLIP_QUATERNION = (1.0, 0.0, 0.0, 0.0)


def quaternion_to_matrix(q) -> np.ndarray:
    """(x, y, z, w) quaternion -> 3x3 rotation matrix."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


@dataclass
class Frame:
    """A 3D pose (position + orientation) that a shape's 2D plane is drawn in."""

    position: np.ndarray  # (3,)
    quaternion: tuple      # (x, y, z, w)
    rotation: np.ndarray = field(init=False)  # (3, 3)

    def __post_init__(self):
        self.rotation = quaternion_to_matrix(self.quaternion)

    def point_3d(self, x: float, y: float) -> np.ndarray:
        """Map a 2D point in the shape's local XY plane (z=0) into 3D."""
        local = np.array([x, y, 0.0])
        return self.position + self.rotation @ local

    def normal(self) -> np.ndarray:
        """World-frame unit normal of the shape's plane (local +Z)."""
        return self.rotation @ np.array([0.0, 0.0, 1.0])

    def tool_quaternion(self) -> tuple:
        """End-effector orientation used to trace this shape.

        The tool's +Z is aligned with the plane's *inward* normal, so the arm
        approaches the plane like a pen held against a surface.
        """
        return quaternion_multiply(self.quaternion, TOOL_FLIP_QUATERNION)


@dataclass
class Waypoint:
    position: np.ndarray   # (3,)
    quaternion: tuple       # (x, y, z, w), constant per-shape
    is_travel: bool = False  # True for pen-up transit moves (not part of the drawn shape)


ArcSpec = dict  # {"arc_center": [x, y], "arc_end": [x, y], "clockwise": bool}


def _tessellate_arc(prev_xy, arc: ArcSpec, segments: int) -> List[Sequence[float]]:
    """Approximate a circular arc as a polyline of `segments` straight segments.

    The arc goes from `prev_xy` to arc["arc_end"], centered at arc["arc_center"].
    MoveIt's Cartesian ("straight line") planner only accepts point-to-point
    goals, so arcs/curves are sampled into short straight segments -- a
    standard technique for any planner limited to straight-line moves.
    """
    if segments < 1:
        raise ValueError(f"arc segments must be >= 1, got {segments}")
    center = np.array(arc["arc_center"], dtype=float)
    end = np.array(arc["arc_end"], dtype=float)
    clockwise = bool(arc.get("clockwise", False))
    if np.linalg.norm(np.array(prev_xy, dtype=float) - end) < 1e-9:
        # Ambiguous without a sweep field: zero-length arc, or full circle?
        raise ValueError(
            f"Arc start {prev_xy} and end {end.tolist()} coincide; split a full "
            f"circle into two arcs"
        )

    start_vec = np.array(prev_xy, dtype=float) - center
    end_vec = end - center
    r_start, r_end = np.linalg.norm(start_vec), np.linalg.norm(end_vec)
    if r_start < 1e-9 or abs(r_start - r_end) > 1e-3 * max(r_start, 1e-9):
        raise ValueError(
            f"Arc center {center.tolist()} is not equidistant from start "
            f"{prev_xy} (r={r_start:.5f}) and end {end.tolist()} (r={r_end:.5f})"
        )

    start_ang = np.arctan2(start_vec[1], start_vec[0])
    end_ang = np.arctan2(end_vec[1], end_vec[0])
    delta = end_ang - start_ang
    if clockwise:
        while delta > 0:
            delta -= 2 * np.pi
    else:
        while delta < 0:
            delta += 2 * np.pi

    points = []
    for i in range(1, segments + 1):
        ang = start_ang + delta * (i / segments)
        pt = center + r_start * np.array([np.cos(ang), np.sin(ang)])
        points.append((float(pt[0]), float(pt[1])))
    return points


def _de_boor(t: float, degree: int, knots: np.ndarray, control: np.ndarray) -> np.ndarray:
    """Evaluate a B-spline at parameter t via De Boor's algorithm."""
    n = len(control) - 1
    # Locate the knot span containing t.
    if t >= knots[n + 1]:
        span = n
    else:
        span = degree
        while span < n and knots[span + 1] <= t:
            span += 1

    d = [control[span - degree + j].astype(float).copy() for j in range(degree + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            i = span - degree + j
            denom = knots[i + degree - r + 1] - knots[i]
            alpha = 0.0 if denom < 1e-12 else (t - knots[i]) / denom
            d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]
    return d[degree]


def _tessellate_bspline(prev_xy, spec: dict, segments: int) -> List[Sequence[float]]:
    """Sample a clamped B-spline into a polyline of `segments` straight segments.

    The previous vertex is prepended as the first control point, so the curve
    starts exactly where the last segment ended. A clamped (repeated end) knot
    vector makes the curve interpolate its first and last control points, which
    is what lets B-spline segments chain with plain points and arcs.
    """
    if segments < 1:
        raise ValueError(f"spline segments must be >= 1, got {segments}")
    control = np.array([list(prev_xy)] + [list(p) for p in spec["control_points"]], dtype=float)
    degree = int(spec.get("degree", 3))
    if degree < 1:
        raise ValueError(f"B-spline degree must be >= 1, got {degree}")
    if len(control) <= degree:
        raise ValueError(
            f"B-spline of degree {degree} needs at least {degree + 1} control points "
            f"(including the implicit start point), got {len(control)}"
        )

    n = len(control) - 1
    # Clamped uniform knot vector: degree+1 zeros, interior, degree+1 ones.
    interior = n - degree
    knots = np.concatenate([
        np.zeros(degree + 1),
        np.arange(1, interior + 1) / (interior + 1) if interior > 0 else np.empty(0),
        np.ones(degree + 1),
    ])

    points = []
    for i in range(1, segments + 1):
        pt = _de_boor(i / segments, degree, knots, control)
        points.append((float(pt[0]), float(pt[1])))
    return points


def flatten_vertices(vertices: Sequence[Union[Sequence[float], ArcSpec]], arc_segments: int) -> List[Sequence[float]]:
    """Expand a vertex list (points and/or arc specs) into a plain polyline of 2D points."""
    # range(1, segments + 1) is empty for segments <= 0, which would silently
    # drop every arc and spline instead of failing.
    if not isinstance(arc_segments, int) or isinstance(arc_segments, bool) or arc_segments < 1:
        raise ValueError(f"arc_segments must be an integer >= 1, got {arc_segments!r}")
    if not vertices:
        return []
    first = vertices[0]
    if isinstance(first, dict):
        raise ValueError("The first vertex must be a plain [x, y] point, not an arc")
    flat: List[Sequence[float]] = [tuple(float(v) for v in first)]
    for v in vertices[1:]:
        if isinstance(v, dict) and "control_points" in v:
            flat.extend(_tessellate_bspline(flat[-1], v, arc_segments))
        elif isinstance(v, dict):
            flat.extend(_tessellate_arc(flat[-1], v, arc_segments))
        else:
            flat.append(tuple(float(c) for c in v))
    return flat


def build_shape_waypoints(
    vertices: Sequence[Union[Sequence[float], ArcSpec]],
    position: Sequence[float],
    rpy: Sequence[float],
    closed: bool,
    lift_height: float,
    arc_segments: int = 16,
) -> List[Waypoint]:
    """Turn a shape definition into an ordered list of end-effector waypoints.

    - The shape's plane is the local XY plane (z=0) of the frame defined by
      (position, rpy); vertices are expressed in that local frame.
    - The tool orientation is held constant for the whole shape, equal to the
      frame's orientation flipped so the tool points into the plane.
    - Output starts and ends with a "hover" waypoint offset along the plane
      normal by `lift_height`, so the arm approaches/departs pen-up rather
      than dragging through free space at drawing height.
    """
    frame = Frame(position=np.array(position, dtype=float), quaternion=rpy_to_quaternion(*rpy))
    points_2d = flatten_vertices(vertices, arc_segments)
    if closed and points_2d and points_2d[0] != points_2d[-1]:
        points_2d = list(points_2d) + [points_2d[0]]

    quat = frame.tool_quaternion()
    hover_offset = frame.normal() * lift_height

    waypoints: List[Waypoint] = []
    hover_start = frame.point_3d(*points_2d[0]) + hover_offset
    waypoints.append(Waypoint(position=hover_start, quaternion=quat, is_travel=True))
    for (x, y) in points_2d:
        waypoints.append(Waypoint(position=frame.point_3d(x, y), quaternion=quat, is_travel=False))
    hover_end = frame.point_3d(*points_2d[-1]) + hover_offset
    waypoints.append(Waypoint(position=hover_end, quaternion=quat, is_travel=True))
    return waypoints
