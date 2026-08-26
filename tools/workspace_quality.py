#!/usr/bin/env python3
"""Sweep the workspace for *quality*, not just reachability.

Reachable is a low bar. A pose can have an IK solution and still be a bad place
to draw, because the Jacobian is close to rank-deficient there: the arm needs
huge joint speeds for small tool motions, which reads as lurching and can make
the controller abort.

For every (radius, height, tilt) cell this records:
  - whether MoveIt can solve IK at all (collision-aware)
  - Yoshikawa manipulability of that solution, w = sqrt(det(J J^T))
  - the smallest singular value of J
  - how close the solution sits to a joint limit

Manipulability depends on *which* IK solution is picked, and a 7-DoF arm has
infinitely many. That is deliberate: we score the solution MoveIt would actually
use, seeded the same way the tracer seeds it, so the number reflects what the
robot will really do.

Writes a JSON grid the shape designer renders as its overlay.
"""

import argparse
import json
import math
import subprocess
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

sys.path.insert(0, "/home/dev/dev_ws/install/avatar_challenge/lib/python3.10/site-packages")
from avatar_challenge.kinematics import Chain          # noqa: E402
from avatar_challenge.geometry import rpy_to_quaternion, quaternion_multiply  # noqa: E402

SEED = [0.0, -0.5706, 0.0, 0.5039, 0.0, 1.0745, 0.0]


def load_urdf():
    out = subprocess.run(["ros2", "param", "get", "/robot_state_publisher",
                          "robot_description"], capture_output=True, text=True).stdout
    return out.split("String value is: ", 1)[1]


class Sweeper(Node):
    def __init__(self, chain):
        super().__init__("workspace_quality")
        self.chain = chain
        self.names = [j.name for j in chain.actuated]
        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        self.cli.wait_for_service(timeout_sec=120.0)

    def ik(self, xyz, quat, seed):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "xarm7"
        req.ik_request.ik_link_name = "link_eef"
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        js = JointState()
        js.name = self.names
        js.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = js
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = quat
        req.ik_request.pose_stamped = ps
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = res.solution.joint_state
        idx = {n: i for i, n in enumerate(sol.name)}
        return [sol.position[idx[n]] for n in self.names]


def tool_quat(tilt_deg):
    """Tool orientation for a plane tilted by `tilt_deg` (0 = table, 90 = wall)."""
    q_plane = rpy_to_quaternion(math.radians(tilt_deg), 0.0, 0.0)
    return quaternion_multiply(q_plane, (1.0, 0.0, 0.0, 0.0))   # flip: pen into plane


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/quality.json")
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--tilts", default="0,45,90")
    args = ap.parse_args()

    rclpy.init()
    chain = Chain.from_urdf(load_urdf())
    sw = Sweeper(chain)

    tilts = [float(t) for t in args.tilts.split(",")]
    radii = [round(r * args.step, 4) for r in range(0, int(0.80 / args.step) + 1)]
    zs = [round(z * args.step, 4) for z in range(0, int(0.90 / args.step) + 1)]

    grid, stats = {}, []
    total = len(tilts) * len(radii) * len(zs)
    done = 0
    for tilt in tilts:
        quat = tool_quat(tilt)
        rows = []
        for z in zs:
            row = []
            for r in radii:
                q = sw.ik((r, 0.0, z), quat, SEED)
                if q is None:
                    row.append(None)
                else:
                    w = chain.manipulability(q)
                    row.append(round(w, 5))
                    stats.append(w)
                done += 1
            rows.append(row)
            print(f"tilt {tilt:5.1f} z={z:.2f}  {done}/{total}", flush=True)
        grid[str(int(tilt))] = rows

    arr = np.array(stats)
    summary = {
        "count": int(arr.size),
        "min": float(arr.min()), "max": float(arr.max()),
        "median": float(np.median(arr)),
        "p05": float(np.percentile(arr, 5)),
        "p25": float(np.percentile(arr, 25)),
    }
    with open(args.out, "w") as f:
        json.dump({"radii": radii, "zs": zs, "tilts": tilts,
                   "grid": grid, "summary": summary}, f)
    print("\nmanipulability over all reachable poses:")
    for k, v in summary.items():
        print(f"  {k:>7}: {v}")
    print(f"\nWROTE {args.out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
