"""Record link_eef poses from TF while the tracer runs, then report how closely
the actual path passed through each target vertex."""
import json
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, "/home/dev/dev_ws/install/avatar_challenge/lib/python3.10/site-packages")
from avatar_challenge.geometry import build_shape_waypoints
from avatar_challenge.shapes_io import load_shapes


def point_to_polyline(p, path):
    """Shortest distance from p to the polyline through `path` samples.

    Measuring to the segments rather than the sample points makes the metric
    independent of how fast the recorder happened to sample TF.
    """
    a, b = path[:-1], path[1:]
    ab = b - a
    denom = np.einsum("ij,ij->i", ab, ab)
    denom[denom < 1e-15] = 1e-15
    t = np.clip(np.einsum("ij,ij->i", p - a, ab) / denom, 0.0, 1.0)
    closest = a + t[:, None] * ab
    return float(np.linalg.norm(closest - p, axis=1).min())


class Recorder(Node):
    def __init__(self):
        super().__init__("verify_path")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.samples = []
        self.create_timer(0.005, self.sample)

    def sample(self):
        try:
            t = self.buffer.lookup_transform("world", "link_eef", rclpy.time.Time())
        except Exception:
            return
        tr = t.transform.translation
        self.samples.append((tr.x, tr.y, tr.z))


def main():
    duration = float(sys.argv[1])
    shapes_file = sys.argv[2]

    rclpy.init()
    rec = Recorder()
    end = rec.get_clock().now().nanoseconds + duration * 1e9
    while rclpy.ok() and rec.get_clock().now().nanoseconds < end:
        rclpy.spin_once(rec, timeout_sec=0.1)

    path = np.array(rec.samples)
    print(f"\nrecorded {len(path)} eef samples")
    if len(path) == 0:
        return
    print(f"eef bounding box: x[{path[:,0].min():.3f},{path[:,0].max():.3f}] "
          f"y[{path[:,1].min():.3f},{path[:,1].max():.3f}] "
          f"z[{path[:,2].min():.3f},{path[:,2].max():.3f}]")

    worst = 0.0
    for shape in load_shapes(shapes_file):
        wps = build_shape_waypoints(shape.vertices, shape.position, shape.rpy,
                                    shape.closed, 0.03, 16)
        targets = [w for w in wps if not w.is_travel]
        errs = [point_to_polyline(w.position, path) for w in targets]
        worst = max(worst, max(errs))
        print(f"\n[{shape.name}] {len(targets)} target points")
        print(f"   max miss distance: {max(errs)*1000:.2f} mm")
        print(f"   mean miss distance: {np.mean(errs)*1000:.2f} mm")

    print(f"\nWORST vertex miss across all shapes: {worst*1000:.2f} mm")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
