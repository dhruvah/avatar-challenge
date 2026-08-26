"""Serves the shape designer and traces what it sends, over HTTP.

This closes the loop: draw in the browser, press Send to robot, and the arm
moves -- no copying files into the container by hand.

Two design points worth stating up front:

*Threading.* rclpy calls must all happen on one thread, so the HTTP handler
never touches ROS. It reserves the robot under a lock, puts a job on a queue and
blocks on an Event; the main thread owns the node, drains the queue, and posts
the result back. Reservation is atomic, so two browsers pressing Send at the
same instant cannot both be accepted.

*Separation.* Everything the browser needs -- live tool path, progress, joint
angles -- lives here rather than in ShapeTracerNode, which is byte-identical to
the one on the lean default branch. This file subscribes for what it needs.
"""

import json
import os
import queue
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from avatar_challenge.geometry import Frame, build_shape_waypoints, rpy_to_quaternion
from avatar_challenge.kinematics import Chain
from avatar_challenge.shapes_io import load_shapes

MAX_BODY = 1 << 20          # 1 MB is far more than any plausible shape list
PROGRESS_PATH_CAP = 400     # points returned per poll; the canvas cannot show more
TRACE_TIMEOUT = 600.0
# The tool is "drawing" only while it is on the shape's plane. Travel moves run
# at the pen-up hover height (lift_height, 3 cm by default), so this separates
# the drawn outline from the approach without needing a hook into the tracer.
ON_PLANE_TOLERANCE_M = 0.003

IDLE, TRACING, SUCCEEDED, FAILED = "idle", "tracing", "succeeded", "failed"


class Job:
    def __init__(self, payload):
        self.payload = payload
        self.done = threading.Event()
        self.result = None
        # Set when the HTTP side gives up waiting. The worker checks it before
        # starting, so a request that timed out can never move the arm later.
        self.abandoned = False


class _Handler(BaseHTTPRequestHandler):
    server_version = "PlaneAndPen/1.0"

    def _send(self, code, body, ctype="application/json"):
        raw = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def log_message(self, fmt, *args):
        self.server.owner.node.get_logger().debug(fmt % args)

    def do_GET(self):
        owner = self.server.owner
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            try:
                with open(owner.page_path, "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            except OSError as exc:
                return self._send(500, json.dumps({"error": f"page missing: {exc}"}))
        if path == "/api/progress":
            return self._send(200, json.dumps(owner.progress_snapshot()))
        if path == "/api/status":
            return self._send(200, json.dumps({
                "connected": True, "busy": owner.is_busy(), "robot": "xArm7",
            }))
        return self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        owner = self.server.owner
        if self.path.split("?")[0] != "/api/trace":
            return self._send(404, json.dumps({"error": "not found"}))
        try:
            length = int(self.headers.get("Content-Length", 0))
        except ValueError:
            return self._send(400, json.dumps({"error": "bad Content-Length"}))
        if length <= 0 or length > MAX_BODY:
            return self._send(413, json.dumps({"error": "body too large or empty"}))

        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            return self._send(400, json.dumps({"error": f"invalid JSON: {exc}"}))

        job = owner.reserve(payload)
        if job is None:
            return self._send(409, json.dumps({
                "error": "the arm is already tracing -- wait for it to finish"}))

        if not job.done.wait(timeout=TRACE_TIMEOUT):
            owner.abandon(job)
            return self._send(504, json.dumps({"error": "trace timed out"}))
        ok = job.result.get("ok", False)
        return self._send(200 if ok else 400, json.dumps(job.result))


class DesignerServer:
    """Owns the HTTP server and everything the browser sees."""

    def __init__(self, node, page_path, config_path, port=8080,
                 bind_address="127.0.0.1"):
        self.node = node
        self.page_path = page_path
        self.config_path = config_path
        self.port = port
        self.bind_address = bind_address

        self.jobs = queue.Queue()
        self._lock = threading.Lock()
        self._reserved = False              # guarded by _lock
        self._httpd = None

        # live state for the UI
        self._state = IDLE
        self._error = None
        self._shape = None
        self._index = 0
        self._total = 0
        self._frame = None
        self._target_len = 0.0
        self._path = []
        self._recording = False
        self._chain = None
        self._joints = None
        self._started = time.time()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, "/robot_description", self._on_urdf, latched)
        node.create_subscription(JointState, "/joint_states", self._on_joints, 10)

    # -- ROS callbacks (main thread) ------------------------------------------
    def _on_urdf(self, msg):
        if self._chain is None:
            try:
                self._chain = Chain.from_urdf(msg.data)
            except Exception as exc:                       # noqa: BLE001
                self.node.get_logger().warn(f"URDF parse failed: {exc}")

    def _on_joints(self, msg):
        if not msg.name or not msg.position or self._chain is None:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            q = [msg.position[idx[j.name]] for j in self._chain.actuated]
        except KeyError:
            return
        self._joints = [round(float(v), 5) for v in q]
        if self._recording and self._frame is not None:
            world = self._chain.fk(q)[:3, 3]
            local = self._frame.rotation.T @ (world - self._frame.position)
            # Skip the approach and the pen-up lift: otherwise the overlay draws
            # a line from wherever the arm started, across the canvas, to the
            # shape -- which reads as part of the drawing.
            if abs(float(local[2])) > ON_PLANE_TOLERANCE_M:
                return
            self._path.append([round(float(local[0]) * 1000, 2),
                               round(float(local[1]) * 1000, 2)])

    # -- reservation -----------------------------------------------------------
    def reserve(self, payload):
        """Atomically claim the robot. Returns a queued Job, or None if busy."""
        with self._lock:
            if self._reserved:
                return None
            self._reserved = True
        job = Job(payload)
        self.jobs.put(job)
        return job

    def abandon(self, job):
        job.abandoned = True

    def is_busy(self):
        with self._lock:
            return self._reserved

    # -- server ----------------------------------------------------------------
    def start(self):
        httpd = ThreadingHTTPServer((self.bind_address, self.port), _Handler)
        httpd.owner = self
        self._httpd = httpd
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        self.node.get_logger().info(
            f"Shape designer live on http://{self.bind_address}:{self.port} "
            f"-- draw a shape and press 'Send to robot'")

    def stop(self):
        if self._httpd:
            self._httpd.shutdown()

    def spin(self, rclpy):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                job = self.jobs.get_nowait()
            except queue.Empty:
                continue
            try:
                if job.abandoned:
                    self.node.get_logger().warn(
                        "dropping a trace request whose client gave up waiting")
                    continue
                try:
                    job.result = self._run(job.payload)
                except Exception as exc:                   # noqa: BLE001
                    self.node.get_logger().error(f"trace failed: {exc}")
                    self._state, self._error = FAILED, str(exc)
                    job.result = {"ok": False, "error": str(exc),
                                  "detail": traceback.format_exc(limit=3)}
            finally:
                with self._lock:
                    self._reserved = False
                job.done.set()

    # -- the trace --------------------------------------------------------------
    def _run(self, payload):
        """Validate, persist, then trace. Never persist before validating.

        Writing first would let a rejected design destroy the last good one, so
        the payload is staged beside the target, validated by the real loader,
        and only then moved into place. os.replace is atomic.
        """
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        staged = self.config_path + ".staged"
        with open(staged, "w") as f:
            json.dump(payload, f, indent=2)
        try:
            shapes = load_shapes(staged)
        except Exception:
            os.unlink(staged)
            raise
        os.replace(staged, self.config_path)

        self.node.shapes = shapes
        self.node.publish_markers()
        self.node.clear_actual()

        self._state, self._error = TRACING, None
        # Every submission starts from the ready pose, not from wherever the
        # previous one happened to stop. The approach is planned from the
        # current pose and the Cartesian solver seeds its IK from it, so an
        # arbitrary end configuration makes the next trace start in a different
        # -- sometimes contorted -- branch. Homing once per request keeps
        # repeated submissions identical. Homing between *shapes* within a
        # request was measured as strictly worse and is deliberately not done.
        if getattr(self.node, "home_on_start", True):
            self.node.get_logger().info("Returning to the ready pose")
            self.node.go_home()

        self._total, self._index = len(shapes), 0
        names = [s.name for s in shapes]
        try:
            for i, shape in enumerate(shapes):
                self._begin(shape, i)
                self.node.trace_shape(shape)
        finally:
            self._recording = False

        self._state = SUCCEEDED
        self.node.get_logger().info(f"Traced {len(shapes)} shape(s) from the designer")
        return {"ok": True, "traced": names,
                "message": f"Traced {len(names)} shape(s): {', '.join(names)}"}

    def _begin(self, shape, index):
        self._shape, self._index = shape.name, index
        self._frame = Frame(position=np.array(shape.position, dtype=float),
                            quaternion=rpy_to_quaternion(*shape.rpy))
        drawn = [w for w in build_shape_waypoints(
            shape.vertices, shape.position, shape.rpy, shape.closed, 0.0)
            if not w.is_travel]
        self._target_len = 1000.0 * sum(
            float(np.linalg.norm(drawn[i + 1].position - drawn[i].position))
            for i in range(len(drawn) - 1))
        self._path = []
        self._started = time.time()
        self._recording = True

    # -- what the browser reads --------------------------------------------------
    def progress_snapshot(self):
        """Progress measured by distance covered, not elapsed time.

        Length along the drawing is what the user is watching, and it stays
        honest if the trajectory runs slower than planned. 100% is reported only
        once execution has actually succeeded.
        """
        pts = self._path
        if len(pts) > PROGRESS_PATH_CAP:
            stride = len(pts) / PROGRESS_PATH_CAP
            keep = sorted({int(i * stride) for i in range(PROGRESS_PATH_CAP)}
                          | {len(pts) - 1})
            pts = [pts[i] for i in keep]

        if self._state == SUCCEEDED:
            fraction = 1.0
        elif self._state == IDLE:
            fraction = 0.0
        else:
            # TRACING or FAILED: however far it actually got, never 100%
            fraction = min(self._covered(), 0.999)

        return {
            "state": self._state,
            "tracing": self._state == TRACING,
            "error": self._error,
            "shape": self._shape,
            "shape_index": self._index,
            "shape_total": self._total,
            "fraction": round(fraction, 4),
            "elapsed": round(time.time() - self._started, 2),
            "joints": self._joints,
            "path": pts,
            "busy": self.is_busy(),
        }

    def _covered(self):
        if self._target_len <= 0 or len(self._path) < 2:
            return 0.0
        walked = sum(
            ((self._path[i + 1][0] - self._path[i][0]) ** 2 +
             (self._path[i + 1][1] - self._path[i][1]) ** 2) ** 0.5
            for i in range(len(self._path) - 1))
        return min(1.0, walked / self._target_len)
