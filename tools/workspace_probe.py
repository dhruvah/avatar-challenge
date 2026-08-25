"""Probe xArm7 reachability over a grid of poses using MoveIt's /compute_ik."""
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * .5), math.sin(roll * .5)
    cp, sp = math.cos(pitch * .5), math.sin(pitch * .5)
    cy, sy = math.cos(yaw * .5), math.sin(yaw * .5)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


class Probe(Node):
    def __init__(self):
        super().__init__("ik_probe")
        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        self.cli.wait_for_service(timeout_sec=60.0)

    def reachable(self, xyz, quat):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "xarm7"
        req.ik_request.ik_link_name = "link_eef"
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = quat
        req.ik_request.pose_stamped = ps
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        return res is not None and res.error_code.val == 1


def main():
    rclpy.init()
    p = Probe()

    orientations = {
        "tool_down (Rx180)": rpy_to_quat(math.pi, 0.0, 0.0),
        "tool_up  (identity)": rpy_to_quat(0.0, 0.0, 0.0),
        "tool_forward (Ry90)": rpy_to_quat(0.0, math.pi / 2, 0.0),
    }

    for label, q in orientations.items():
        print(f"\n=== {label} ===")
        print("      " + "".join(f"{x:6.2f}" for x in [0.05, 0.15, 0.25, 0.35, 0.45, 0.55]))
        for z in [0.10, 0.15, 0.25, 0.35, 0.45, 0.55]:
            row = f"z={z:4.2f}"
            for x in [0.05, 0.15, 0.25, 0.35, 0.45, 0.55]:
                row += f"{'  ok  ' if p.reachable((x, 0.0, z), q) else '  --  '}"
            print(row)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
