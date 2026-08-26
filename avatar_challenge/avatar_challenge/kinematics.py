"""Forward kinematics, geometric Jacobian and manipulability, from the URDF.

Reachability is not the only thing that makes a pose a bad place to draw. A pose
can have a perfectly good IK solution and still sit near a *singularity*, where
the Jacobian loses rank: the arm can no longer move the tool freely in every
direction, and small Cartesian motions demand enormous joint velocities. That
shows up as the arm lurching, or the controller aborting.

Everything here is plain numpy parsed straight from the URDF, so it runs without
MoveIt and fast enough to sweep thousands of poses.

Measure used: Yoshikawa manipulability, w = sqrt(det(J @ J.T)). It is the volume
of the velocity ellipsoid -- zero exactly at a singularity, larger where the arm
has more freedom. We also expose the smallest singular value of J, which is the
more direct "how close to rank loss" number.
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


@dataclass
class Joint:
    name: str
    kind: str                 # "revolute" | "prismatic" | "fixed" | ...
    origin_xyz: np.ndarray
    origin_rot: np.ndarray
    axis: np.ndarray
    parent: str
    child: str
    lower: float = -np.pi
    upper: float = np.pi


class Chain:
    """A serial kinematic chain extracted from a URDF."""

    def __init__(self, joints: List[Joint], base: str, tip: str):
        self.joints = joints
        self.base = base
        self.tip = tip
        self.actuated = [j for j in joints if j.kind in ("revolute", "continuous", "prismatic")]

    # -- construction ---------------------------------------------------------
    @classmethod
    def from_urdf(cls, urdf_text: str, base: str = "link_base", tip: str = "link_eef"):
        root = ET.fromstring(urdf_text)
        by_child = {}
        for j in root.findall("joint"):
            parent = j.find("parent").get("link")
            child = j.find("child").get("link")
            org = j.find("origin")
            xyz = np.array([float(v) for v in (org.get("xyz", "0 0 0").split())]) if org is not None \
                else np.zeros(3)
            rpy = [float(v) for v in (org.get("rpy", "0 0 0").split())] if org is not None else [0, 0, 0]
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None \
                else np.array([0.0, 0.0, 1.0])
            lim = j.find("limit")
            lo = float(lim.get("lower")) if lim is not None and lim.get("lower") else -np.pi
            hi = float(lim.get("upper")) if lim is not None and lim.get("upper") else np.pi
            by_child[child] = Joint(j.get("name"), j.get("type"), xyz, _rpy_matrix(*rpy),
                                    axis, parent, child, lo, hi)

        # walk backwards from the tip to the base, then reverse
        seq, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"no path from {tip} back to {base}: stuck at {link}")
            j = by_child[link]
            seq.append(j)
            link = j.parent
        seq.reverse()
        return cls(seq, base, tip)

    # -- kinematics -----------------------------------------------------------
    def fk_all(self, q: Sequence[float]):
        """Transforms of every joint frame, plus the tip. Returns (origins, axes, tip_T)."""
        q = list(q)
        T = np.eye(4)
        origins, axes = [], []
        qi = 0
        for j in self.joints:
            local = np.eye(4)
            local[:3, :3] = j.origin_rot
            local[:3, 3] = j.origin_xyz
            T = T @ local
            if j.kind in ("revolute", "continuous"):
                theta = q[qi]; qi += 1
                axis = j.axis / np.linalg.norm(j.axis)
                K = np.array([[0, -axis[2], axis[1]],
                              [axis[2], 0, -axis[0]],
                              [-axis[1], axis[0], 0]])
                R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
                rot = np.eye(4); rot[:3, :3] = R
                origins.append(T[:3, 3].copy())
                axes.append((T[:3, :3] @ axis).copy())
                T = T @ rot
            elif j.kind == "prismatic":
                d = q[qi]; qi += 1
                axis = j.axis / np.linalg.norm(j.axis)
                tr = np.eye(4); tr[:3, 3] = axis * d
                origins.append(T[:3, 3].copy())
                axes.append((T[:3, :3] @ axis).copy())
                T = T @ tr
        return origins, axes, T

    def fk(self, q) -> np.ndarray:
        """4x4 pose of the tip."""
        return self.fk_all(q)[2]

    def jacobian(self, q) -> np.ndarray:
        """6xN geometric Jacobian of the tip, in the base frame."""
        origins, axes, T = self.fk_all(q)
        p_tip = T[:3, 3]
        cols = []
        for j, (o, a) in zip(self.actuated, zip(origins, axes)):
            if j.kind == "prismatic":
                cols.append(np.concatenate([a, np.zeros(3)]))
            else:
                cols.append(np.concatenate([np.cross(a, p_tip - o), a]))
        return np.array(cols).T

    # -- singularity measures --------------------------------------------------
    def manipulability(self, q) -> float:
        """Yoshikawa measure: sqrt(det(J J^T)). Zero at a singularity."""
        J = self.jacobian(q)
        val = np.linalg.det(J @ J.T)
        return float(np.sqrt(max(val, 0.0)))

    def sigma_min(self, q) -> float:
        """Smallest singular value of J -- the direct distance to rank loss."""
        return float(np.linalg.svd(self.jacobian(q), compute_uv=False)[-1])

    def condition(self, q) -> float:
        """Ratio of largest to smallest singular value; blows up near singularities."""
        s = np.linalg.svd(self.jacobian(q), compute_uv=False)
        return float(s[0] / s[-1]) if s[-1] > 1e-12 else float("inf")

    def joint_limit_margin(self, q) -> float:
        """Smallest normalised distance to a joint limit, in [0, 0.5]."""
        worst = 1.0
        for j, v in zip(self.actuated, q):
            span = j.upper - j.lower
            if span <= 0:
                continue
            worst = min(worst, min(v - j.lower, j.upper - v) / span)
        return float(worst)
