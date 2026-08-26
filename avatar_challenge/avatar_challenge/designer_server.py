"""Serves the shape designer and traces what it sends, over HTTP.

This closes the loop: draw in the browser, press Send to robot, and the arm
moves -- no copying files into the container by hand.

Threading model: rclpy service calls must all happen on one thread, so the HTTP
handler never touches ROS directly. It puts a job on a queue and blocks on an
Event; the main thread owns the node, drains the queue, and sets the Event with
the result. That keeps every rclpy call on the thread that created the node.
"""

import json
import os
import queue
import threading
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MAX_BODY = 1 << 20  # 1 MB is far more than any plausible shape list


class Job:
    def __init__(self, payload):
        self.payload = payload
        self.done = threading.Event()
        self.result = None


class _Handler(BaseHTTPRequestHandler):
    server_version = "PlaneAndPen/1.0"

    # -- helpers ------------------------------------------------------------
    def _send(self, code, body, ctype="application/json"):
        raw = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def log_message(self, fmt, *args):           # keep ROS logs readable
        self.server.node.get_logger().debug(fmt % args)

    # -- routes -------------------------------------------------------------
    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            try:
                with open(self.server.page_path, "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            except OSError as exc:
                return self._send(500, json.dumps({"error": f"page missing: {exc}"}))
        if path == "/api/status":
            return self._send(200, json.dumps({
                "connected": True,
                "busy": self.server.busy.is_set(),
                "robot": "xArm7",
            }))
        return self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
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

        if self.server.busy.is_set():
            return self._send(409, json.dumps({
                "error": "the arm is already tracing -- wait for it to finish"}))

        job = Job(payload)
        self.server.jobs.put(job)
        # Generous: a long shape list legitimately takes a while to trace.
        if not job.done.wait(timeout=600.0):
            return self._send(504, json.dumps({"error": "trace timed out"}))
        ok = job.result.get("ok", False)
        return self._send(200 if ok else 400, json.dumps(job.result))


class DesignerServer:
    """Owns the HTTP server; the ROS node stays on the caller's thread."""

    def __init__(self, node, page_path, config_path, port=8080):
        self.node = node
        self.page_path = page_path
        self.config_path = config_path
        self.port = port
        self.jobs = queue.Queue()
        self.busy = threading.Event()
        self._httpd = None

    def start(self):
        httpd = ThreadingHTTPServer(("0.0.0.0", self.port), _Handler)
        httpd.node = self.node
        httpd.page_path = self.page_path
        httpd.jobs = self.jobs
        httpd.busy = self.busy
        self._httpd = httpd
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        self.node.get_logger().info(
            f"Shape designer live on http://localhost:{self.port} "
            f"-- draw a shape and press 'Send to robot'"
        )

    def stop(self):
        if self._httpd:
            self._httpd.shutdown()

    def spin(self, rclpy):
        """Main loop: pump ROS, and run any queued trace on this thread."""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                job = self.jobs.get_nowait()
            except queue.Empty:
                continue
            self.busy.set()
            try:
                job.result = self._run(job.payload)
            except Exception as exc:                     # noqa: BLE001
                self.node.get_logger().error(f"trace failed: {exc}")
                job.result = {"ok": False, "error": str(exc),
                              "detail": traceback.format_exc(limit=3)}
            finally:
                self.busy.clear()
                job.done.set()

    def _run(self, payload):
        """Persist what the browser sent, then trace it."""
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        with open(self.config_path, "w") as f:
            json.dump(payload, f, indent=2)

        # load_shapes does the real validation and raises with a precise message
        shapes = self.node.reload(self.config_path)
        names = [s.name for s in shapes]
        self.node.publish_markers()
        for shape in shapes:
            self.node.trace_shape(shape)
        self.node.get_logger().info(f"Traced {len(shapes)} shape(s) from the designer")
        return {"ok": True, "traced": names,
                "message": f"Traced {len(names)} shape(s): {', '.join(names)}"}
