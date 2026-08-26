"""Kinematics tests against synthetic URDFs with known closed-form answers.

Using hand-built two- and three-joint chains rather than the xArm URDF means the
expected pose and Jacobian can be written down exactly, so these tests check the
maths rather than restating whatever the code currently produces.
"""

import math

import numpy as np
import pytest

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


# A planar 2R arm, whose pose and Jacobian can be written down exactly.
L1, L2 = 0.5, 0.3

PLANAR = urdf([
    ("j1", "revolute", "0 0 0", "0 0 0", "0 0 1", "base", "l1"),
    ("j2", "revolute", f"{L1} 0 0", "0 0 0", "0 0 1", "l1", "l2"),
    ("tip", "fixed", f"{L2} 0 0", "0 0 0", "0 0 1", "l2", "tool"),
])


@pytest.mark.parametrize("q", [(0.3, 0.0), (1.1, -0.4)])
def test_forward_kinematics_matches_closed_form(q):
    chain = Chain.from_urdf(PLANAR, base="base", tip="tool")
    a, b = q
    expect = [L1 * math.cos(a) + L2 * math.cos(a + b),
              L1 * math.sin(a) + L2 * math.sin(a + b), 0.0]
    assert np.allclose(chain.fk(q)[:3, 3], expect, atol=1e-12)


def test_jacobian_matches_finite_differences():
    """The Jacobian is what the manipulability numbers rest on, so check it
    against a numerical derivative of the FK rather than against itself."""
    chain = Chain.from_urdf(PLANAR, base="base", tip="tool")
    q = np.array([0.4, 0.9])
    J = chain.jacobian(q)
    eps = 1e-7
    for i in range(len(q)):
        step = q.copy()
        step[i] += eps
        numeric = (chain.fk(step)[:3, 3] - chain.fk(q)[:3, 3]) / eps
        assert np.allclose(J[:3, i], numeric, atol=1e-6)


def test_a_chain_that_does_not_reach_the_base_is_rejected():
    broken = urdf([
        ("j1", "revolute", "0 0 0", "0 0 0", "0 0 1", "base", "l1"),
        ("j2", "revolute", "0 0 0", "0 0 0", "0 0 1", "orphan", "tool"),
    ])
    with pytest.raises(ValueError, match="no path"):
        Chain.from_urdf(broken, base="base", tip="tool")
