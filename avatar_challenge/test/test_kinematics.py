"""Kinematics tests against synthetic URDFs with known closed-form answers.

Using hand-built two- and three-joint chains rather than the xArm URDF means the
expected pose and Jacobian can be written down exactly, so these tests check the
maths rather than restating whatever the code currently produces.
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from avatar_challenge.kinematics import Chain  # noqa: E402


def urdf(joints, base="base", tip=None):
    """Build a minimal serial URDF from (name, type, xyz, rpy, axis, parent, child)."""
    links = {base}
    body = []
    for name, kind, xyz, rpy, axis, parent, child in joints:
        links |= {parent, child}
        axis_tag = f'<axis xyz="{axis}"/>' if kind != "fixed" else ""
        body.append(
            f'<joint name="{name}" type="{kind}">'
            f'<parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{xyz}" rpy="{rpy}"/>{axis_tag}'
            f'<limit lower="-3.14" upper="3.14"/></joint>'
        )
    link_tags = "".join(f'<link name="{n}"/>' for n in sorted(links))
    return f'<robot name="t">{link_tags}{"".join(body)}</robot>'


# --------------------------------------------------------------------------
# a planar 2R arm: closed-form FK and Jacobian
# --------------------------------------------------------------------------

L1, L2 = 0.5, 0.3

PLANAR = urdf([
    ("j1", "revolute", "0 0 0", "0 0 0", "0 0 1", "base", "l1"),
    ("j2", "revolute", f"{L1} 0 0", "0 0 0", "0 0 1", "l1", "l2"),
    ("tip", "fixed", f"{L2} 0 0", "0 0 0", "0 0 1", "l2", "tool"),
])


@pytest.mark.parametrize("q", [(0.0, 0.0), (0.3, 0.0), (0.0, 0.7),
                               (1.1, -0.4), (-2.0, 2.5)])
def test_planar_2r_forward_kinematics(q):
    chain = Chain.from_urdf(PLANAR, base="base", tip="tool")
    got = chain.fk(q)[:3, 3]
    a, b = q
    expect = [L1 * math.cos(a) + L2 * math.cos(a + b),
              L1 * math.sin(a) + L2 * math.sin(a + b), 0.0]
    assert np.allclose(got, expect, atol=1e-12)


@pytest.mark.parametrize("q", [(0.4, 0.2), (-1.0, 0.9), (2.2, -1.3)])
def test_planar_2r_jacobian_matches_closed_form(q):
    chain = Chain.from_urdf(PLANAR, base="base", tip="tool")
    a, b = q
    s1, c1 = math.sin(a), math.cos(a)
    s12, c12 = math.sin(a + b), math.cos(a + b)
    expect_linear = np.array([
        [-L1 * s1 - L2 * s12, -L2 * s12],
        [L1 * c1 + L2 * c12, L2 * c12],
        [0.0, 0.0],
    ])
    J = chain.jacobian(q)
    assert np.allclose(J[:3], expect_linear, atol=1e-12)
    # both joints spin about +Z, so the angular rows are constant
    assert np.allclose(J[3:], np.array([[0, 0], [0, 0], [1, 1]]), atol=1e-12)


def _positional_det(chain, q):
    """Determinant of the in-plane position Jacobian.

    A 2-joint arm is only ever singular in *position*: it can still rotate about
    Z at any pose, so the full 6-D Jacobian keeps rank 2 even fully extended.
    The planar position block is what actually loses rank.
    """
    return float(np.linalg.det(chain.jacobian(q)[:2, :]))


@pytest.mark.parametrize("q", [(0.3, 0.0), (0.0, 0.0), (-1.2, 0.0)])
def test_planar_2r_is_singular_when_fully_extended(q):
    chain = Chain.from_urdf(PLANAR, base="base", tip="tool")
    assert abs(_positional_det(chain, q)) < 1e-12


@pytest.mark.parametrize("q", [(0.3, math.pi / 2), (0.0, 1.0), (-1.2, -0.8)])
def test_planar_2r_is_well_conditioned_when_bent(q):
    chain = Chain.from_urdf(PLANAR, base="base", tip="tool")
    assert abs(_positional_det(chain, q)) > 0.01


def test_folded_back_is_also_singular():
    chain = Chain.from_urdf(PLANAR, base="base", tip="tool")
    assert abs(_positional_det(chain, (0.0, math.pi))) < 1e-12


# --------------------------------------------------------------------------
# prismatic joints
# --------------------------------------------------------------------------

PRISM = urdf([
    ("p1", "prismatic", "0 0 0", "0 0 0", "0 0 1", "base", "l1"),
    ("tip", "fixed", "0.1 0 0", "0 0 0", "0 0 1", "l1", "tool"),
])


@pytest.mark.parametrize("d", [0.0, 0.25, -0.4])
def test_prismatic_translates_along_its_axis(d):
    chain = Chain.from_urdf(PRISM, base="base", tip="tool")
    assert np.allclose(chain.fk([d])[:3, 3], [0.1, 0.0, d], atol=1e-12)


def test_prismatic_jacobian_is_pure_translation():
    chain = Chain.from_urdf(PRISM, base="base", tip="tool")
    J = chain.jacobian([0.2])
    assert np.allclose(J[:3, 0], [0, 0, 1], atol=1e-12)
    assert np.allclose(J[3:, 0], [0, 0, 0], atol=1e-12)


# --------------------------------------------------------------------------
# origin rotations, limits, and malformed input
# --------------------------------------------------------------------------

def test_joint_origin_rotation_is_applied():
    """A 90 deg roll on the origin turns a +Z axis into -Y."""
    chain = Chain.from_urdf(urdf([
        ("j1", "revolute", "0 0 0", f"{math.pi/2} 0 0", "0 0 1", "base", "l1"),
        ("tip", "fixed", "0 0 0.4", "0 0 0", "0 0 1", "l1", "tool"),
    ]), base="base", tip="tool")
    assert np.allclose(chain.fk([0.0])[:3, 3], [0.0, -0.4, 0.0], atol=1e-12)


def test_joint_limits_are_read_and_margin_is_reported():
    chain = Chain.from_urdf(PLANAR, base="base", tip="tool")
    assert chain.joint_limit_margin([0.0, 0.0]) == pytest.approx(0.5, abs=1e-9)
    at_limit = chain.joint_limit_margin([-3.14, 0.0])
    assert at_limit == pytest.approx(0.0, abs=1e-9)


def test_fixed_joints_do_not_consume_joint_values():
    chain = Chain.from_urdf(PLANAR, base="base", tip="tool")
    assert len(chain.actuated) == 2
    assert [j.name for j in chain.actuated] == ["j1", "j2"]


def test_disconnected_chain_is_rejected():
    broken = urdf([
        ("j1", "revolute", "0 0 0", "0 0 0", "0 0 1", "base", "l1"),
        ("j2", "revolute", "0 0 0", "0 0 0", "0 0 1", "orphan", "tool"),
    ])
    with pytest.raises(ValueError, match="no path"):
        Chain.from_urdf(broken, base="base", tip="tool")


def test_unknown_tip_is_rejected():
    with pytest.raises(ValueError):
        Chain.from_urdf(PLANAR, base="base", tip="nope")


def test_manipulability_is_positive_for_a_short_chain():
    """With fewer joints than task dimensions the Gram form must still be used;
    det(J J^T) would be identically zero and report every pose as singular."""
    chain = Chain.from_urdf(PLANAR, base="base", tip="tool")
    assert chain.manipulability((0.3, math.pi / 2)) > 0.0
    assert chain.manipulability((0.3, 0.0)) > 0.0


def test_manipulability_falls_as_a_redundant_arm_extends():
    """On a 6+ DOF chain the standard measure applies and must drop near a
    singular configuration."""
    six = urdf([
        ("j1", "revolute", "0 0 0.1", "0 0 0", "0 0 1", "base", "l1"),
        ("j2", "revolute", "0 0 0", f"{-math.pi/2} 0 0", "0 0 1", "l1", "l2"),
        ("j3", "revolute", "0.4 0 0", "0 0 0", "0 0 1", "l2", "l3"),
        ("j4", "revolute", "0.3 0 0", f"{math.pi/2} 0 0", "0 0 1", "l3", "l4"),
        ("j5", "revolute", "0 0 0.2", f"{-math.pi/2} 0 0", "0 0 1", "l4", "l5"),
        ("j6", "revolute", "0 0 0", f"{math.pi/2} 0 0", "0 0 1", "l5", "l6"),
        ("tip", "fixed", "0 0 0.1", "0 0 0", "0 0 1", "l6", "tool"),
    ])
    chain = Chain.from_urdf(six, base="base", tip="tool")
    bent = chain.manipulability([0.0, 0.6, -1.2, 0.0, 0.7, 0.0])
    flat = chain.manipulability([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert bent > flat
