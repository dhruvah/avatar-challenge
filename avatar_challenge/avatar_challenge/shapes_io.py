"""Load and validate the shape-list input file (JSON)."""

import json
from dataclasses import dataclass
from typing import List, Sequence, Union


@dataclass
class ShapeDef:
    name: str
    vertices: List[Union[Sequence[float], dict]]
    position: Sequence[float]
    rpy: Sequence[float]
    closed: bool


def load_shapes(path: str, default_closed: bool = True) -> List[ShapeDef]:
    with open(path, "r") as f:
        data = json.load(f)

    shapes_raw = data["shapes"]
    shapes: List[ShapeDef] = []
    for i, s in enumerate(shapes_raw):
        name = s.get("name", f"shape_{i}")
        vertices = s["vertices"]
        if not vertices:
            raise ValueError(f"Shape '{name}': vertices must be non-empty")
        first = vertices[0]
        if isinstance(first, dict) or list(first) != [0.0, 0.0]:
            if isinstance(first, dict) or [float(c) for c in first] != [0.0, 0.0]:
                raise ValueError(f"Shape '{name}': first vertex must be (0, 0), got {first}")

        start_pose = s["start_pose"]
        position = start_pose["position"]
        rpy = start_pose.get("rpy", [0.0, 0.0, 0.0])
        if len(position) != 3:
            raise ValueError(f"Shape '{name}': start_pose.position must have 3 elements")
        if len(rpy) != 3:
            raise ValueError(f"Shape '{name}': start_pose.rpy must have 3 elements")

        closed = bool(s.get("closed", default_closed))
        shapes.append(ShapeDef(name=name, vertices=vertices, position=position, rpy=rpy, closed=closed))
    return shapes
