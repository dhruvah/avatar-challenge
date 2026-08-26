#!/usr/bin/env python3
"""Forward a local TCP port into a running Docker container.

The challenge container publishes only the RDP port, and published ports cannot
be added to a container after it is created. Rather than recreate the container
(and drop your RDP session), this relays each connection through `docker exec`,
which needs nothing installed on either side beyond Python.

    python3 tools/docker_tunnel.py                 # localhost:8080 -> container:8080
    python3 tools/docker_tunnel.py --port 9000

If you would rather have it natively, recreate the container with
`-p 8080:8080` and skip this entirely.
"""

import argparse
import socket
import subprocess
import sys
import threading

# Runs inside the container: bridge stdin/stdout to the local service socket.
INNER = """
import socket, sys, threading
s = socket.create_connection(("127.0.0.1", {port}))
def up():
    try:
        while True:
            d = sys.stdin.buffer.read1(65536)
            if not d:
                break
            s.sendall(d)
    except Exception:
        pass
    try:
        s.shutdown(socket.SHUT_WR)
    except Exception:
        pass
threading.Thread(target=up, daemon=True).start()
try:
    while True:
        d = s.recv(65536)
        if not d:
            break
        sys.stdout.buffer.write(d)
        sys.stdout.buffer.flush()
except Exception:
    pass
"""


def pump(src, dst, close):
    try:
        while True:
            data = src(65536)
            if not data:
                break
            dst(data)
    except Exception:
        pass
    finally:
        close()


def serve_one(conn, container, port):
    proc = subprocess.Popen(
        ["docker", "exec", "-i", container, "python3", "-u", "-c",
         INNER.format(port=port)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
    )

    def shut():
        try:
            conn.close()
        except Exception:
            pass
        if proc.poll() is None:
            proc.kill()

    def to_container(data):
        # subprocess stdin is block-buffered; without the flush the request
        # sits in the pipe buffer and the browser waits forever.
        proc.stdin.write(data)
        proc.stdin.flush()

    threading.Thread(target=pump, args=(conn.recv, to_container, shut),
                     daemon=True).start()
    pump(proc.stdout.read1, conn.sendall, shut)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--container", default="xarm-container")
    ap.add_argument("--port", type=int, default=8080,
                    help="port inside the container, also used locally")
    ap.add_argument("--local-port", type=int, default=None)
    args = ap.parse_args()
    local = args.local_port or args.port

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind(("127.0.0.1", local))
    except OSError as exc:
        sys.exit(f"cannot bind localhost:{local} -- {exc}")
    srv.listen(16)
    print(f"http://localhost:{local}  ->  {args.container}:{args.port}")
    print("Ctrl-C to stop.")
    try:
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=serve_one,
                             args=(conn, args.container, args.port),
                             daemon=True).start()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        srv.close()


if __name__ == "__main__":
    main()
