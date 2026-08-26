#!/usr/bin/env python3
"""Plan a large, varied batch of shapes and score how well the arm executes them.

Answers the questions you actually care about before trusting a shape:
  - does the preflight IK accept every waypoint?
  - does the Cartesian planner complete the whole path, or truncate?
  - does the resulting trajectory pass near a singularity?
  - does it move far more in joint space than the tool moves in Cartesian space
    (a proxy for "the arm is flailing to draw a small shape")?
  - do big per-step joint jumps appear (elbow flips / discontinuities)?

Planning only by default -- it is the planner, solver and waypoint configuration
under test, and planning exercises all three without spending minutes moving.
Pass --execute to also run them on the arm.
"""

import argparse
import json
import math
import subprocess
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from sensor_msgs.msg import JointState

sys.path.insert(0, "/home/dev/dev_ws/install/avatar_challenge/lib/python3.10/site-packages")
from avatar_challenge.geometry import build_shape_waypoints        # noqa: E402
from avatar_challenge.kinematics import Chain                      # noqa: E402

SEED = [0.0, -0.5706, 0.0, 0.5039, 0.0, 1.0745, 0.0]


# --------------------------------------------------------------------------
# shape catalogue
# --------------------------------------------------------------------------
def square(s):     return [[0, 0], [0, s], [s, s], [s, 0]]
def triangle(s):   return [[0, 0], [s, 0], [s / 2, s * 0.87]]
def hexagon(s):
    r = s / 2
    return [[0, 0]] + [[r * math.cos(a) - r, r * math.sin(a)]
                       for a in (np.arange(1, 6) / 6 * 2 * math.pi)]
def star(s):
    r = s / 2
    pts = [[0, 0]]
    for i in range(1, 10):
        rr = r if i % 2 == 0 else r * 0.45
        a = i / 10 * 2 * math.pi
        pts.append([rr * math.cos(a) - r, rr * math.sin(a)])
    return pts
def circle(s):
    r = s / 2
    return [[0, 0],
            {"arc_center": [-r, 0], "arc_end": [-2 * r, 0], "clockwise": False},
            {"arc_center": [-r, 0], "arc_end": [0, 0], "clockwise": False}]
def blob(s):
    return [[0, 0], {"control_points": [[s * .6, s * .3], [s, s * .8],
                                        [s * .2, s], [0, 0]], "degree": 3}]

SHAPES = {"square": square, "triangle": triangle, "hexagon": hexagon,
          "star": star, "circle": circle, "spline": blob}


def load_urdf():
    out = subprocess.run(["ros2", "param", "get", "/robot_state_publisher",
                          "robot_description"], capture_output=True, text=True).stdout
    return out.split("String value is: ", 1)[1]


class Suite(Node):
    def __init__(self, chain):
        super().__init__("trajectory_suite")
        self.chain = chain
        self.names = [j.name for j in chain.actuated]
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.cart = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.ik.wait_for_service(timeout_sec=120.0)
        self.cart.wait_for_service(timeout_sec=120.0)

    def _seed_state(self):
        js = JointState()
        js.name = self.names
        js.position = [float(v) for v in SEED]
        return js

    def reachable(self, pos, quat):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "xarm7"
        req.ik_request.ik_link_name = "link_eef"
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        req.ik_request.robot_state.joint_state = self._seed_state()
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = quat
        req.ik_request.pose_stamped = ps
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        res = fut.result()
        return res is not None and res.error_code.val == 1

    def plan(self, waypoints):
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = "xarm7"
        req.link_name = "link_eef"
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.start_state.joint_state = self._seed_state()
        for w in waypoints:
            p = Pose()
            p.position.x, p.position.y, p.position.z = (float(v) for v in w.position)
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = (float(v) for v in w.quaternion)
            req.waypoints.append(p)
        fut = self.cart.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        return fut.result()


def score(chain, traj, cart_len):
    pts = traj.joint_trajectory.points
    Q = np.array([p.positions for p in pts])
    dq = np.abs(np.diff(Q, axis=0))
    w = [chain.manipulability(p.positions) for p in pts]
    sig = [chain.sigma_min(p.positions) for p in pts]
    lim = [chain.joint_limit_margin(p.positions) for p in pts]
    travel = float(np.degrees(dq.sum()))
    return {
        "points": len(pts),
        "w_min": float(min(w)),
        "sigma_min": float(min(sig)),
        "limit_margin": float(min(lim)),
        "max_step_deg": float(np.degrees(dq.max())) if dq.size else 0.0,
        "joint_travel_deg": travel,
        # degrees of joint motion per mm of tool motion: "how hard is the arm
        # working for this shape"
        "deg_per_mm": travel / max(cart_len * 1000, 1e-6),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/suite.json")
    ap.add_argument("--sizes", default="0.06,0.12")
    ap.add_argument("--limit", type=int, default=0, help="cap the number of cases")
    args = ap.parse_args()

    rclpy.init()
    chain = Chain.from_urdf(load_urdf())
    suite = Suite(chain)

    sizes = [float(v) for v in args.sizes.split(",")]
    cases = []
    for name, fn in SHAPES.items():
        for size in sizes:
            for (x, y, z) in [(0.30, -0.05, 0.25), (0.42, 0.00, 0.30),
                              (0.25, 0.18, 0.35), (0.35, -0.20, 0.20),
                              (0.30, 0.00, 0.50)]:
                for tilt, yaw in [(0.0, 0.0), (0.0, 0.785), (90.0, 0.0), (45.0, 0.0)]:
                    cases.append({"shape": name, "size": size,
                                  "pos": (x, y, z), "tilt": tilt, "yaw": yaw,
                                  "verts": fn(size)})
    if args.limit:
        cases = cases[:args.limit]

    print(f"planning {len(cases)} trajectories\n")
    print(f"{'shape':<9}{'sz':>5}{'pos':>20}{'tilt':>6}{'yaw':>6}  "
          f"{'result':<12}{'w_min':>8}{'maxstep':>9}{'deg/mm':>8}")
    results = []
    for i, c in enumerate(cases):
        rpy = [math.radians(c["tilt"]), 0.0, c["yaw"]]
        wps = build_shape_waypoints(c["verts"], list(c["pos"]), rpy, True, 0.03, 16)
        cart_len = sum(float(np.linalg.norm(np.array(wps[k + 1].position) -
                                            np.array(wps[k].position)))
                       for k in range(len(wps) - 1))
        rec = dict(c); rec.pop("verts"); rec["pos"] = list(c["pos"])

        unreachable = [k for k, w in enumerate(wps)
                       if not suite.reachable(w.position, w.quaternion)]
        if unreachable:
            rec.update(result="unreachable", n_bad=len(unreachable))
            results.append(rec)
            print(f"{c['shape']:<9}{c['size']:>5.2f}{str(c['pos']):>20}"
                  f"{c['tilt']:>6.0f}{math.degrees(c['yaw']):>6.0f}  "
                  f"{'unreachable':<12}{'-':>8}{'-':>9}{'-':>8}")
            continue

        res = suite.plan(wps[1:])
        if res is None or res.fraction < 0.99:
            rec.update(result="plan_failed",
                       fraction=0.0 if res is None else float(res.fraction))
            results.append(rec)
            print(f"{c['shape']:<9}{c['size']:>5.2f}{str(c['pos']):>20}"
                  f"{c['tilt']:>6.0f}{math.degrees(c['yaw']):>6.0f}  "
                  f"{'plan ' + ('%.0f%%' % ((res.fraction if res else 0)*100)):<12}"
                  f"{'-':>8}{'-':>9}{'-':>8}")
            continue

        m = score(chain, res.solution, cart_len)
        rec.update(result="ok", fraction=float(res.fraction), **m)
        results.append(rec)
        print(f"{c['shape']:<9}{c['size']:>5.2f}{str(c['pos']):>20}"
              f"{c['tilt']:>6.0f}{math.degrees(c['yaw']):>6.0f}  "
              f"{'ok':<12}{m['w_min']:>8.4f}{m['max_step_deg']:>9.1f}"
              f"{m['deg_per_mm']:>8.2f}")

    with open(args.out, "w") as f:
        json.dump(results, f, indent=1)

    ok = [r for r in results if r["result"] == "ok"]
    print(f"\n{'='*76}")
    print(f"planned {len(cases)}: {len(ok)} ok, "
          f"{sum(1 for r in results if r['result']=='unreachable')} unreachable, "
          f"{sum(1 for r in results if r['result']=='plan_failed')} plan failures")
    if ok:
        w = np.array([r["w_min"] for r in ok])
        st = np.array([r["max_step_deg"] for r in ok])
        dm = np.array([r["deg_per_mm"] for r in ok])
        print(f"manipulability w_min : min {w.min():.4f}  p05 {np.percentile(w,5):.4f}  "
              f"median {np.median(w):.4f}  max {w.max():.4f}")
        print(f"max joint step (deg) : median {np.median(st):.1f}  p95 {np.percentile(st,95):.1f}  "
              f"max {st.max():.1f}")
        print(f"joint deg per tool mm: median {np.median(dm):.2f}  p95 {np.percentile(dm,95):.2f}  "
              f"max {dm.max():.2f}")
    print(f"WROTE {args.out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
