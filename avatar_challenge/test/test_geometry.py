"""Unit tests for the pure-numpy geometry and the JSON loader.

These need neither ROS nor a running robot -- geometry.py and shapes_io.py
import only numpy and the standard library, which is why the rotation and
tessellation maths lives there rather than inside the node.

Run:  python3 -m pytest avatar_challenge/test/test_geometry.py -q
"""

import json
import math

import numpy as np
import pytest

from avatar_challenge.geometry import (  # noqa: E402
    Frame, build_shape_waypoints, flatten_vertices, quaternion_multiply,
    quaternion_to_matrix, rpy_to_quaternion, _tessellate_arc, _tessellate_bspline,
)
from avatar_challenge.shapes_io import load_shapes  # noqa: E402


# --------------------------------------------------------------------------
# rotations
# --------------------------------------------------------------------------

def _rpy_matrix_reference(roll, pitch, yaw):
    """Rz(yaw) @ Ry(pitch) @ Rx(roll), built explicitly for comparison."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


@pytest.mark.parametrize("rpy", [
    (0, 0, 0.7854), (0.3, -0.7, 1.1), (math.pi, math.pi / 2, -math.pi),
])
def test_rpy_quaternion_matches_matrix_convention(rpy):
    """The hand-rolled quaternion must agree with the REP-103 matrix."""
    q = rpy_to_quaternion(*rpy)
    assert np.allclose(quaternion_to_matrix(q), _rpy_matrix_reference(*rpy), atol=1e-12)
    assert np.isclose(np.linalg.norm(q), 1.0)


def test_quaternion_multiply_is_hamilton_product():
    a = rpy_to_quaternion(0.3, -0.2, 1.0)
    b = rpy_to_quaternion(-0.7, 0.4, 0.2)
    # matrix of the product == product of the matrices, in the same order
    assert np.allclose(
        quaternion_to_matrix(quaternion_multiply(a, b)),
        quaternion_to_matrix(a) @ quaternion_to_matrix(b),
        atol=1e-12,
    )


def test_quaternion_multiply_identity():
    q = rpy_to_quaternion(0.4, 0.1, -0.9)
    assert np.allclose(quaternion_multiply(q, (0, 0, 0, 1)), q)


def test_tool_points_into_the_plane():
    """Tool +Z must oppose the plane normal -- a pen, not an antenna."""
    for rpy in [(0, 0, 0), (0, 0, 1.2), (math.pi / 2, 0, 0), (0.4, -0.3, 2.0)]:
        f = Frame(position=np.zeros(3), quaternion=rpy_to_quaternion(*rpy))
        tool_z = quaternion_to_matrix(f.tool_quaternion()) @ np.array([0.0, 0.0, 1.0])
        assert np.isclose(tool_z @ f.normal(), -1.0, atol=1e-12)


# --------------------------------------------------------------------------
# planar mapping
# --------------------------------------------------------------------------

def test_first_vertex_lands_exactly_on_start_pose():
    pos = [0.30, -0.05, 0.25]
    wps = build_shape_waypoints([[0, 0], [0, 0.1], [0.1, 0.1]], pos,
                                [0, 0, 0.7854], True, 0.03)
    drawn = [w for w in wps if not w.is_travel]
    assert np.allclose(drawn[0].position, pos)


def test_all_drawn_points_are_coplanar_and_hover_is_off_plane():
    pos, rpy = [0.30, 0.0, 0.25], [0.4, -0.2, 1.0]
    wps = build_shape_waypoints([[0, 0], [0.1, 0], [0.1, 0.1]], pos, rpy, True, 0.03)
    f = Frame(position=np.array(pos), quaternion=rpy_to_quaternion(*rpy))
    for w in wps:
        offset = (w.position - f.position) @ f.normal()
        assert np.isclose(offset, 0.03 if w.is_travel else 0.0, atol=1e-12)


def test_square_side_lengths_survive_rotation():
    wps = build_shape_waypoints([[0, 0], [0, 0.1], [0.1, 0.1], [0.1, 0]],
                                [0.3, 0, 0.25], [0, 0, 0.7854], True, 0.03)
    pts = [w.position for w in wps if not w.is_travel]
    for a, b in zip(pts, pts[1:]):
        assert np.isclose(np.linalg.norm(b - a), 0.1, atol=1e-12)


def test_orientation_is_constant_across_a_shape():
    wps = build_shape_waypoints([[0, 0], [0.1, 0], [0.1, 0.1]], [0.3, 0, 0.25],
                                [0, 0, 0.5], True, 0.03)
    assert all(w.quaternion == wps[0].quaternion for w in wps)


# --------------------------------------------------------------------------
# arcs
# --------------------------------------------------------------------------

def test_arc_points_stay_on_the_circle():
    arc = {"arc_center": [1.0, 0.0], "arc_end": [2.0, 0.0], "clockwise": False}
    pts = _tessellate_arc((0.0, 0.0), arc, 32)
    for p in pts:
        assert np.isclose(math.dist(p, (1.0, 0.0)), 1.0, atol=1e-12)
    assert np.allclose(pts[-1], [2.0, 0.0])


def test_arc_direction_is_respected():
    ccw = _tessellate_arc((0.0, 0.0), {"arc_center": [1, 0], "arc_end": [2, 0],
                                       "clockwise": False}, 8)
    cw = _tessellate_arc((0.0, 0.0), {"arc_center": [1, 0], "arc_end": [2, 0],
                                      "clockwise": True}, 8)
    # opposite sweeps must bulge to opposite sides of the chord
    assert np.sign(ccw[3][1]) == -np.sign(cw[3][1])


@pytest.mark.parametrize("start,end,cw,expect_deg", [
    ((1.0, 0.0), (0.0, 1.0), False, 90.0),      # simple quarter
    ((1.0, 0.0), (0.0, 1.0), True, 270.0),      # the long way round
    ((-1.0, 0.0), (0.0, -1.0), False, 90.0),    # crosses +/-pi
])
def test_arc_sweep_direction_across_the_pi_wraparound(start, end, cw, expect_deg):
    """atan2 wraps at +/-pi; the sweep must still take the requested direction."""
    pts = _tessellate_arc(start, {"arc_center": [0.0, 0.0],
                                  "arc_end": list(end), "clockwise": cw}, 72)
    total = 0.0
    prev = np.array(start)
    for p in pts:
        p = np.array(p)
        cross = prev[0] * p[1] - prev[1] * p[0]
        dot = prev @ p
        total += math.atan2(cross, dot)
        prev = p
    assert abs(math.degrees(abs(total)) - expect_deg) < 1e-6
    assert (total < 0) == cw
    assert np.allclose(pts[-1], end, atol=1e-12)


@pytest.mark.parametrize("degree,n_ctrl", [(1, 4), (3, 8), (5, 5)])
def test_bspline_accepts_any_valid_degree_and_control_count(degree, n_ctrl):
    ctrl = [[0.1 * (i + 1), 0.05 * ((i % 3) - 1)] for i in range(n_ctrl)]
    pts = _tessellate_bspline((0.0, 0.0), {"control_points": ctrl,
                                           "degree": degree}, 24)
    assert len(pts) == 24
    assert np.allclose(pts[-1], ctrl[-1], atol=1e-9)
    assert all(np.isfinite(p).all() for p in pts)


def test_arc_rejects_inconsistent_radius():
    with pytest.raises(ValueError, match="equidistant"):
        _tessellate_arc((0.0, 0.0), {"arc_center": [1, 0], "arc_end": [5, 0]}, 8)


# --------------------------------------------------------------------------
# B-splines
# --------------------------------------------------------------------------

def test_clamped_spline_interpolates_its_endpoints():
    spec = {"control_points": [[0.05, 0.08], [0.10, -0.02], [0.15, 0.06]], "degree": 3}
    pts = _tessellate_bspline((0.0, 0.0), spec, 40)
    assert np.allclose(pts[-1], [0.15, 0.06], atol=1e-12)


def test_degree_one_spline_is_a_straight_line():
    pts = _tessellate_bspline((0.0, 0.0), {"control_points": [[1.0, 1.0]],
                                           "degree": 1}, 4)
    assert np.allclose(pts, [[0.25, 0.25], [0.5, 0.5], [0.75, 0.75], [1.0, 1.0]])


def test_spline_stays_inside_control_polygon_hull():
    """Convex-hull property -- a strong check that De Boor is correct."""
    ctrl = [[0.1, 0.3], [0.4, -0.1], [0.6, 0.5]]
    pts = _tessellate_bspline((0.0, 0.0), {"control_points": ctrl, "degree": 3}, 60)
    allpts = np.array([[0.0, 0.0]] + ctrl)
    for p in pts:
        assert allpts[:, 0].min() - 1e-9 <= p[0] <= allpts[:, 0].max() + 1e-9
        assert allpts[:, 1].min() - 1e-9 <= p[1] <= allpts[:, 1].max() + 1e-9


def test_spline_rejects_too_few_control_points():
    with pytest.raises(ValueError, match="degree"):
        _tessellate_bspline((0.0, 0.0), {"control_points": [[1, 1]], "degree": 3}, 8)


# --------------------------------------------------------------------------
# flattening
# --------------------------------------------------------------------------

def test_mixed_segment_types_flatten_in_order():
    verts = [[0, 0], [0.08, 0],
             {"arc_center": [0.08, 0.02], "arc_end": [0.10, 0.02], "clockwise": False},
             [0.0, 0.08]]
    flat = flatten_vertices(verts, 16)
    assert flat[0] == (0.0, 0.0)
    assert np.allclose(flat[-1], [0.0, 0.08])
    assert len(flat) == 2 + 16 + 1


@pytest.mark.parametrize("bad", [0, -1])
def test_zero_or_negative_arc_segments_is_rejected(bad):
    """segments <= 0 would silently drop arcs rather than fail."""
    with pytest.raises(ValueError, match="arc_segments"):
        flatten_vertices([[0, 0], [0.1, 0]], bad)


def test_first_vertex_may_not_be_a_segment_dict():
    with pytest.raises(ValueError):
        flatten_vertices([{"arc_center": [0, 0], "arc_end": [1, 1]}], 8)


# --------------------------------------------------------------------------
# loader validation
# --------------------------------------------------------------------------

def _write(tmp_path, shape):
    p = tmp_path / "shapes.json"
    p.write_text(json.dumps({"shapes": [shape]}))
    return str(p)


GOOD = {"name": "s", "vertices": [[0, 0], [0.1, 0], [0.1, 0.1]],
        "start_pose": {"position": [0.3, 0, 0.25], "rpy": [0, 0, 0]}}


def test_valid_shape_loads(tmp_path):
    shapes = load_shapes(_write(tmp_path, GOOD))
    assert shapes[0].name == "s" and shapes[0].speed == 1.0


def test_collinear_points_are_valid_geometry(tmp_path):
    s = dict(GOOD, vertices=[[0, 0], [0.05, 0], [0.1, 0], [0.1, 0.1]])
    assert len(load_shapes(_write(tmp_path, s))[0].vertices) == 4


@pytest.mark.parametrize("shape,match", [
    (dict(GOOD, vertices=[[0.1, 0], [0.2, 0]]), "first vertex"),
    (dict(GOOD, vertices=[[0, 0]]), "at least two"),
    (dict(GOOD, vertices=[[0, 0], [0.1, 0, 0.2]]), "expected 2 numbers"),
    (dict(GOOD, closed="false"), "true or false"),
    (dict(GOOD, speed=1.5), "speed"),
    (dict(GOOD, start_pose={"position": [0.3, 0]}), "expected 3 numbers"),
    (dict(GOOD, vertices=[[0, 0], {"arc_center": [1, 0], "arc_end": [5, 0]}]),
     "equidistant"),
    (dict(GOOD, vertices=[[0, 0], {"arc_center": [0.5, 0], "arc_end": [0, 0]}]),
     "coincide"),
    (dict(GOOD, vertices=[[0, 0], {"control_points": [[1, 1]], "arc_end": [1, 1],
                                   "arc_center": [0, 1]}]), "ambiguous"),
    (dict(GOOD, vertices=[[0, 0], {"nonsense": 1}]), "must contain"),
])
def test_malformed_shapes_are_rejected(tmp_path, shape, match):
    with pytest.raises(ValueError, match=match):
        load_shapes(_write(tmp_path, shape))


@pytest.mark.parametrize("bad_name", [5, {"a": 1}, None, ""])
def test_invalid_names_are_rejected(tmp_path, bad_name):
    """A non-string name loads fine and only fails when it is formatted, which
    on the designer path happens after the arm has already moved."""
    with pytest.raises(ValueError, match="name"):
        load_shapes(_write(tmp_path, dict(GOOD, name=bad_name)))


@pytest.mark.parametrize("ok_name", ["square", "carré_45°", "\u5f62", "a" * 120])
def test_valid_names_including_unicode_are_accepted(tmp_path, ok_name):
    assert load_shapes(_write(tmp_path, dict(GOOD, name=ok_name)))[0].name == ok_name


def test_absurdly_long_names_are_rejected(tmp_path):
    with pytest.raises(ValueError, match="maximum"):
        load_shapes(_write(tmp_path, dict(GOOD, name="a" * 500)))


def test_non_finite_numbers_are_rejected(tmp_path):
    p = tmp_path / "nan.json"
    p.write_text('{"shapes":[{"name":"s","vertices":[[0,0],[NaN,0]],'
                 '"start_pose":{"position":[0.3,0,0.25]}}]}')
    with pytest.raises(ValueError, match="finite"):
        load_shapes(str(p))


def test_missing_shapes_key_is_rejected(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text('{"nope": []}')
    with pytest.raises(ValueError, match="shapes"):
        load_shapes(str(p))


# --------------------------------------------------------------------------
# live-progress projection (world -> the shape's own 2D frame)
# --------------------------------------------------------------------------

def _project(frame, world_pts):
    """Mirror of ShapeTracerNode.progress_snapshot's projection."""
    R, origin = frame.rotation, frame.position
    return [(R.T @ (np.asarray(p) - origin))[:2] * 1000 for p in world_pts]


@pytest.mark.parametrize("rpy", [(0, 0, 0.7854), (0.4, -0.3, 1.9)])
def test_progress_projection_inverts_the_plane_transform(rpy):
    """Points sent to the robot must come back as the same 2D coordinates."""
    pos = [0.30, -0.05, 0.28]
    local_mm = [(0, 0), (100, 0), (100, 80), (0, 80), (37.5, 12.25)]
    frame = Frame(position=np.array(pos), quaternion=rpy_to_quaternion(*rpy))
    world = [frame.point_3d(x / 1000, y / 1000) for x, y in local_mm]
    back = _project(frame, world)
    for (x, y), got in zip(local_mm, back):
        assert np.allclose(got, [x, y], atol=1e-6)


def test_progress_projection_drops_the_out_of_plane_component():
    """A hover point above the plane projects onto the same 2D spot as its
    footprint -- the live overlay should not jump during pen-up moves."""
    pos, rpy = [0.30, 0.0, 0.25], [0.0, 0.0, 0.5]
    frame = Frame(position=np.array(pos), quaternion=rpy_to_quaternion(*rpy))
    on_plane = frame.point_3d(0.05, 0.02)
    hovering = on_plane + frame.normal() * 0.03
    assert np.allclose(_project(frame, [on_plane])[0],
                       _project(frame, [hovering])[0], atol=1e-9)
