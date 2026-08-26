#!/usr/bin/env python3
"""Entry point: shape designer web UI wired to the xArm."""

import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory

from avatar_challenge.designer_server import DesignerServer
from avatar_challenge.shape_tracer_node import ShapeTracerNode


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    server = None
    try:
        node = ShapeTracerNode()
        node.declare_parameter("port", 8080)
        share = get_package_share_directory("avatar_challenge")
        page = os.path.join(share, "web", "shape_designer.html")
        config = os.path.join(share, "config", "designer_shapes.json")

        server = DesignerServer(node, page, config,
                                port=node.get_parameter("port").value)
        server.start()
        if node.home_on_start:
            node.get_logger().info("Moving to the ready pose")
            node.go_home()
        server.spin(rclpy)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as exc:  # noqa: BLE001
        if node is not None:
            node.get_logger().error(f"designer_server failed: {exc}")
        else:
            print(f"designer_server failed during startup: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        if server is not None:
            server.stop()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
