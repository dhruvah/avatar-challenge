"""Load and validate the shape-list input file (JSON).

Validation is deliberately strict and happens *before* any ROS client is used,
so a bad file fails with a message naming the offending field rather than as a
KeyError deep in the geometry code or an opaque planner failure once the arm is
already moving.
"""

import json
import math
from dataclasses import dataclass
from typing import List, Sequence, Union


@dataclass
class ShapeDef:
    name: str
    vertices: List[Union[Sequence[float], dict]]
    position: Sequence[float]
    rpy: Sequence[float]
    closed: bool
    speed: float = 1.0


def _reject_non_finite(value, where: str):
    """json.load accepts NaN/Infinity by default; those poison every downstream
    transform silently, so refuse them at the boundary."""
    raise ValueError(f"{where}: {value} is not a finite number")


def _num(value, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{where}: expected a number, got {value!r}")
    if not math.isfinite(value):
        _reject_non_finite(value, where)
    return float(value)


def _point(value, where: str, dims: int = 2) -> List[float]:
    if not isinstance(value, (list, tuple)) or len(value) != dims:
        raise ValueError(f"{where}: expected {dims} numbers, got {value!r}")
    return [_num(v, f"{where}[{i}]") for i, v in enumerate(value)]


def _strict_bool(value, where: str) -> bool:
    # bool("false") is True -- a JSON string here would silently invert intent.
    if not isinstance(value, bool):
        raise ValueError(f"{where}: expected true or false, got {value!r}")
    return value


def _validate_segment(seg: dict, where: str, prev: List[float]):
    """Validate one arc or B-spline segment dict and return its end point."""
    is_arc = "arc_center" in seg or "arc_end" in seg
    is_spline = "control_points" in seg

    if is_arc and is_spline:
        raise ValueError(
            f"{where}: segment has both arc and B-spline keys; it is ambiguous "
            f"which was intended"
        )
    if is_spline:
        cps = seg["control_points"]
        if not isinstance(cps, (list, tuple)) or not cps:
            raise ValueError(f"{where}: control_points must be a non-empty list")
        pts = [_point(p, f"{where}.control_points[{i}]") for i, p in enumerate(cps)]
        degree = seg.get("degree", 3)
        if not isinstance(degree, int) or isinstance(degree, bool) or degree < 1:
            raise ValueError(f"{where}: degree must be an integer >= 1, got {degree!r}")
        # +1 for the implicit start point contributed by the previous vertex.
        if len(pts) + 1 <= degree:
            raise ValueError(
                f"{where}: a degree-{degree} B-spline needs at least {degree} "
                f"control_points, got {len(pts)}"
            )
        return pts[-1]

    if not is_arc:
        raise ValueError(
            f"{where}: segment dict must contain either arc_center/arc_end or "
            f"control_points"
        )
    if "arc_center" not in seg or "arc_end" not in seg:
        raise ValueError(f"{where}: an arc needs both arc_center and arc_end")

    center = _point(seg["arc_center"], f"{where}.arc_center")
    end = _point(seg["arc_end"], f"{where}.arc_end")
    if "clockwise" in seg:
        _strict_bool(seg["clockwise"], f"{where}.clockwise")

    r_start = math.dist(prev, center)
    r_end = math.dist(end, center)
    if r_start < 1e-9:
        raise ValueError(f"{where}: arc start coincides with its center")
    if abs(r_start - r_end) > 1e-3 * r_start:
        raise ValueError(
            f"{where}: arc_center is not equidistant from the arc's start "
            f"(r={r_start:.6f}) and end (r={r_end:.6f})"
        )
    if math.dist(prev, end) < 1e-9:
        # Ambiguous: a zero-length arc, or a full circle? The schema cannot say.
        raise ValueError(
            f"{where}: arc start and end coincide; split a full circle into two "
            f"arcs, as the format has no sweep-angle field"
        )
    return end


def load_shapes(path: str, default_closed: bool = True) -> List[ShapeDef]:
    with open(path, "r") as f:
        # parse_constant fires for NaN / Infinity / -Infinity.
        data = json.load(f, parse_constant=lambda c: _reject_non_finite(c, "file"))

    if not isinstance(data, dict) or "shapes" not in data:
        raise ValueError("Input file must be an object with a 'shapes' key")
    if not isinstance(data["shapes"], list) or not data["shapes"]:
        raise ValueError("'shapes' must be a non-empty list")

    shapes: List[ShapeDef] = []
    for i, s in enumerate(data["shapes"]):
        if not isinstance(s, dict):
            raise ValueError(f"shapes[{i}]: expected an object, got {s!r}")
        name = s.get("name", f"shape_{i}")
        where = f"Shape '{name}'"

        vertices = s.get("vertices")
        if not isinstance(vertices, list) or not vertices:
            raise ValueError(f"{where}: vertices must be a non-empty list")

        first = vertices[0]
        if isinstance(first, dict):
            raise ValueError(f"{where}: the first vertex must be a plain [x, y] point")
        if _point(first, f"{where}.vertices[0]") != [0.0, 0.0]:
            raise ValueError(f"{where}: first vertex must be (0, 0), got {first}")

        cursor = [0.0, 0.0]
        for j, v in enumerate(vertices[1:], start=1):
            loc = f"{where}.vertices[{j}]"
            if isinstance(v, dict):
                cursor = _validate_segment(v, loc, cursor)
            else:
                pt = _point(v, loc)
                if math.dist(pt, cursor) < 1e-9:
                    raise ValueError(
                        f"{loc}: duplicates the previous point {cursor}; "
                        f"zero-length edges cannot be traced"
                    )
                cursor = pt
        if len(vertices) < 2:
            raise ValueError(f"{where}: a shape needs at least two vertices to trace")

        start_pose = s.get("start_pose")
        if not isinstance(start_pose, dict) or "position" not in start_pose:
            raise ValueError(f"{where}: start_pose must be an object with a position")
        position = _point(start_pose["position"], f"{where}.start_pose.position", 3)
        rpy = _point(start_pose.get("rpy", [0.0, 0.0, 0.0]), f"{where}.start_pose.rpy", 3)

        closed = (
            _strict_bool(s["closed"], f"{where}.closed")
            if "closed" in s else bool(default_closed)
        )

        speed = _num(s.get("speed", 1.0), f"{where}.speed")
        if not 0.0 < speed <= 1.0:
            raise ValueError(f"{where}: speed must be in (0, 1], got {speed}")

        shapes.append(ShapeDef(name=name, vertices=vertices, position=position,
                               rpy=rpy, closed=closed, speed=speed))
    return shapes
