#!/usr/bin/env python3
"""Single-camera HTTPS ingress for a local SLF1 SLAM producer (Python 3.9+)."""

import argparse
import hmac
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import select
import signal
import ssl
import struct
import subprocess
import threading
import time


class InputError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


class CameraSession:
    def __init__(self, command, width, height, fps, idle_timeout=15, write_timeout=30,
                 exit_timeout=10):
        if not (0 < width <= 1920 and 0 < height <= 1920 and width * height <= 2073600):
            raise ValueError("camera dimensions exceed the 1080p pixel budget")
        if not 0 < fps <= 240 or any(not math.isfinite(t) or t <= 0 for t in (idle_timeout, write_timeout, exit_timeout)):
            raise ValueError("FPS and timeouts must be positive and FPS at most 240")
        self.command = command
        self.width, self.height, self.fps = width, height, fps
        self.idle_timeout, self.write_timeout, self.exit_timeout = idle_timeout, write_timeout, exit_timeout
        self.lock = threading.Lock()
        self.process = None
        self.lease = None
        self.state = "ready"
        self.detail = "Ready to capture"
        self.last_activity = time.monotonic()
        self.last_id = self.last_timestamp = None
        self.frames = 0

    @property
    def frame_bytes(self):
        return 28 + self.width * self.height

    def status(self):
        with self.lock:
            if self.state == "streaming" and self.process.poll() is not None:
                self._finish("SLAM producer exited unexpectedly", failed=True)
            return dict(width=self.width, height=self.height, fps=self.fps,
                        state=self.state, detail=self.detail, frames=self.frames)

    def start(self):
        with self.lock:
            if self.state != "ready":
                raise InputError("This host session is already used; restart host and Rust streamer", 409)
            try:
                self.process = subprocess.Popen(self.command, stdin=subprocess.PIPE, bufsize=0)
                os.set_blocking(self.process.stdin.fileno(), False)
            except OSError as error:
                self.state, self.detail = "failed", f"Cannot start SLAM producer: {error}"
                if self.process is not None:
                    self._finish(self.detail, failed=True)
                raise InputError(self.detail, 503) from error
            self.lease = secrets.token_urlsafe(24)
            self.state, self.detail = "streaming", "Waiting for camera frames"
            self.last_activity = time.monotonic()
            return self.lease

    def _authorize(self, lease):
        if not self.lease or not hmac.compare_digest(lease.encode(), self.lease.encode()):
            raise InputError("Invalid camera session", 403)
        if self.state != "streaming":
            raise InputError(self.detail + "; restart host and Rust streamer", 409)

    def frame(self, lease, data):
        with self.lock:
            self._authorize(lease)
            if len(data) != self.frame_bytes:
                raise InputError("Frame size does not match configured capture dimensions")
            magic, width, height, frame_id, timestamp = struct.unpack(">4sIIQQ", data[:28])
            if magic != b"SLF1" or (width, height) != (self.width, self.height):
                raise InputError("Invalid frame version or calibrated dimensions")
            if frame_id > 2**53 - 1 or timestamp > 2**63 - 1:
                raise InputError("Frame ID or timestamp out of range")
            if self.last_id is not None and (frame_id <= self.last_id or timestamp <= self.last_timestamp):
                raise InputError("Frame ID and capture timestamp must increase")
            deadline = time.monotonic() + self.write_timeout
            pending = memoryview(data)
            try:
                while pending:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or self.process.poll() is not None:
                        raise OSError("SLAM producer stopped or input timed out")
                    fd = self.process.stdin.fileno()
                    if not select.select([], [fd], [], remaining)[1]:
                        raise OSError("SLAM producer input timed out")
                    try:
                        count = os.write(fd, pending)
                    except BlockingIOError:
                        continue
                    if count <= 0:
                        raise OSError("SLAM producer pipe closed")
                    pending = pending[count:]
            except (OSError, ValueError) as error:
                self._finish(str(error), failed=True)
                raise InputError(self.detail, 503) from error
            self.last_id, self.last_timestamp = frame_id, timestamp
            self.last_activity = time.monotonic()
            self.frames += 1
            self.detail = "Receiving camera frames"

    def _finish(self, reason, failed=False):
        # Caller holds the session lock. A partially written frame is never retried.
        if self.process is not None:
            self.process.stdin.close()
            try:
                code = self.process.wait(timeout=self.exit_timeout)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
                code = self.process.returncode
                failed = True
                reason += "; producer required forced shutdown"
            if code != 0:
                failed = True
                reason += f"; producer exit={code}"
        self.state, self.detail = ("failed" if failed else "ended"), reason

    def stop(self, lease):
        with self.lock:
            self._authorize(lease)
            self._finish("Camera stopped")

    def expire(self):
        with self.lock:
            if self.state == "streaming" and time.monotonic() - self.last_activity >= self.idle_timeout:
                self._finish("Camera disconnected or idle timeout")

    def close(self):
        with self.lock:
            if self.state == "streaming":
                self._finish("Host stopped")


class CameraServer(HTTPServer):
    # Single request at a time: no unbounded request threads or image queue.
    allow_reuse_address = True

    def __init__(self, address, session, token):
        self.session, self.token = session, token
        self.tls = None
        super().__init__(address, Handler)

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(10)
        if self.tls:
            try:
                connection = self.tls.wrap_socket(connection, server_side=True)
            except Exception:
                connection.close()
                raise
        return connection, address


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        self.request.settimeout(10)
        super().setup()

    def log_message(self, *_args):
        # Pairing credentials and arbitrary request URLs must not enter logs.
        pass

    def reply(self, status, body, content_type="application/json"):
        if isinstance(body, dict):
            body = json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; media-src 'self' blob:; frame-ancestors 'none'")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        self.wfile.write(body)

    def authorized(self):
        return hmac.compare_digest(self.headers.get("Authorization", "").encode(),
                                   ("Bearer " + self.server.token).encode())

    def do_GET(self):
        files = {"/": ("index.html", "text/html; charset=utf-8"),
                 "/capture.js": ("capture.js", "text/javascript; charset=utf-8"),
                 "/style.css": ("style.css", "text/css; charset=utf-8")}
        if self.path in files:
            name, mime = files[self.path]
            self.reply(200, (Path(__file__).parent / name).read_bytes(), mime)
        elif self.path == "/status":
            if not self.authorized():
                self.reply(401, dict(error="Invalid pairing token"))
            else:
                self.reply(200, self.server.session.status())
        else:
            self.reply(404, dict(error="Not found"))

    def do_POST(self):
        try:
            if not self.authorized():
                raise InputError("Invalid pairing token", 401)
            if self.path not in ("/start", "/frame", "/stop"):
                raise InputError("Not found", 404)
            lengths = self.headers.get_all("Content-Length", [])
            if self.headers.get("Transfer-Encoding") or len(lengths) != 1:
                raise InputError("One Content-Length is required; chunked input is unsupported", 411)
            expected = self.server.session.frame_bytes if self.path == "/frame" else 0
            if lengths[0] != str(expected):
                raise InputError("Unexpected request size", 413)
            if self.path == "/frame" and self.headers.get("Content-Type") != "application/octet-stream":
                raise InputError("Expected binary Gray8 frame", 415)
            data = self.rfile.read(expected)
            if len(data) != expected:
                raise InputError("Truncated frame")
            lease = self.headers.get("X-Slam-Session", "")
            if self.path == "/start":
                result = dict(session=self.server.session.start())
            elif self.path == "/frame":
                self.server.session.frame(lease, data)
                result = dict(accepted=True)
            else:
                self.server.session.stop(lease)
                result = self.server.session.status()
            self.reply(200, result)
        except InputError as error:
            self.reply(error.status, dict(error=str(error)))
        except (TimeoutError, ConnectionError):
            self.close_connection = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    parser.add_argument("--producer", required=True)
    parser.add_argument("--vocabulary", required=True)
    parser.add_argument("--settings", required=True)
    parser.add_argument("--slam-socket", required=True)
    parser.add_argument("--camera-id", required=True)
    parser.add_argument("--width", required=True, type=int)
    parser.add_argument("--height", required=True, type=int)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--idle-timeout", type=float, default=15)
    args = parser.parse_args()
    if bool(args.cert) != bool(args.key):
        parser.error("--cert and --key must be supplied together")
    if not args.cert and args.bind != "127.0.0.1":
        parser.error("TLS certificate and key are required for non-loopback capture")
    if not args.camera_id.strip():
        parser.error("camera ID must not be empty")
    session_id = args.camera_id + "-" + secrets.token_hex(12)
    command = [args.producer, args.vocabulary, args.settings, args.slam_socket, session_id, args.camera_id]
    session = CameraSession(command, args.width, args.height, args.fps, args.idle_timeout)
    token = secrets.token_urlsafe(32)
    server = CameraServer((args.bind, args.port), session, token)
    if args.cert:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(args.cert, args.key)
        server.tls = context
    done = threading.Event()

    def watchdog():
        while not done.wait(0.5):
            session.expire()

    monitor = threading.Thread(target=watchdog, daemon=True)
    monitor.start()
    scheme = "https" if args.cert else "http"
    print(f"Open {scheme}://HOST:{server.server_port}/#token={token}", flush=True)
    print(f"Camera={args.camera_id} session={session_id}; one camera session per host launch", flush=True)
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        done.set()
        server.server_close()
        session.close()
        monitor.join()


if __name__ == "__main__":
    main()
