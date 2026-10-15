import hashlib
import http.client
import json
from pathlib import Path
import ssl
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from server import CameraServer, CameraSession, InputError


def frame(frame_id=0, timestamp=0, width=2, height=2):
    return struct.pack(">4sIIQQ", b"SLF1", width, height, frame_id, timestamp) + bytes(width * height)


def session(expected=b"", **kwargs):
    digest = hashlib.sha256(expected).hexdigest()
    command = [sys.executable, "-c", "import sys,hashlib; "
               f"sys.exit(0 if hashlib.sha256(sys.stdin.buffer.read()).hexdigest() == '{digest}' else 4)"]
    return CameraSession(command, 2, 2, 30, **kwargs)


class SessionTests(unittest.TestCase):
    def test_exact_forwarding_order_and_exclusive_ownership(self):
        data = frame() + frame(9, 999)
        camera = session(data)
        self.addCleanup(camera.close)
        lease = camera.start()
        with self.assertRaises(InputError):
            camera.start()
        with self.assertRaises(InputError):
            camera.frame("different-phone", frame())
        camera.frame(lease, frame())
        for invalid in (frame(), frame(9, 0), frame(0, 99), frame(2**53, 5),
                        frame(9, 2**63), b"SLF2" + frame()[4:], frame()[:-1]):
            with self.assertRaises(InputError):
                camera.frame(lease, invalid)
        camera.frame(lease, frame(9, 999))
        camera.stop(lease)
        self.assertEqual(camera.process.returncode, 0)
        self.assertEqual(camera.status()["state"], "ended")
        self.assertEqual(camera.status()["frames"], 2)
        with self.assertRaises(InputError):
            camera.start()

    def test_disconnect_expires_and_reaps(self):
        camera = session(idle_timeout=.01)
        self.addCleanup(camera.close)
        camera.start()
        camera.last_activity -= 1
        camera.expire()
        self.assertEqual(camera.process.returncode, 0)
        self.assertEqual(camera.state, "ended")
        self.assertIn("idle timeout", camera.detail)

    def test_producer_failure_and_missing_executable(self):
        missing = CameraSession(["/nonexistent/slam-producer"], 2, 2, 30)
        with self.assertRaises(InputError):
            missing.start()
        self.assertEqual(missing.state, "failed")
        camera = CameraSession([sys.executable, "-c", "raise SystemExit(7)"], 2, 2, 30)
        self.addCleanup(camera.close)
        camera.start()
        camera.process.wait(timeout=3)
        self.assertEqual(camera.status()["state"], "failed")
        self.assertIn("exit=7", camera.detail)

    def test_slow_consumer_bounds_pipe_and_reaps(self):
        camera = CameraSession([sys.executable, "-c", "import time; time.sleep(60)"],
                               640, 480, 30, write_timeout=.05, exit_timeout=.05)
        self.addCleanup(camera.close)
        lease = camera.start()
        started = time.monotonic()
        with self.assertRaises(InputError):
            camera.frame(lease, frame(width=640, height=480))
        self.assertLess(time.monotonic() - started, 3)
        self.assertIsNotNone(camera.process.returncode)
        self.assertEqual(camera.state, "failed")

    def test_dimensions_and_timeout_validation(self):
        for width, height in ((0, 2), (1920, 1920), (2**32, 1)):
            with self.assertRaises(ValueError):
                CameraSession([], width, height, 30)
        for timeout in (0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                session(idle_timeout=timeout)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.camera = session(frame())
        self.server = CameraServer(("127.0.0.1", 0), self.camera, "test-token")
        self.thread = None

    def serve(self):
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        if self.thread:
            self.server.shutdown()
            self.thread.join()
        self.server.server_close()
        self.camera.close()

    def request(self, path, data=b"", headers=None, context=None, method="POST"):
        client = (http.client.HTTPSConnection("127.0.0.1", self.server.server_port, context=context, timeout=5)
                  if context else http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=5))
        try:
            fields = {"Authorization": "Bearer test-token"}
            fields.update(headers or {})
            client.request(method, path, data, fields)
            response = client.getresponse()
            payload = response.read()
            return response.status, json.loads(payload) if response.getheader("Content-Type") == "application/json" else payload
        finally:
            client.close()

    def test_http_auth_limits_lifecycle_and_static_assets(self):
        self.serve()
        self.assertEqual(self.request("/start", headers={"Authorization": "wrong"})[0], 401)
        self.assertEqual(self.request("/status", headers={"Authorization": "Bearer é"}, method="GET")[0], 401)
        self.assertEqual(self.request("/", method="GET")[0], 200)
        self.assertEqual(self.request("/../server.py", method="GET")[0], 404)
        code, started = self.request("/start")
        self.assertEqual(code, 200)
        self.assertEqual(self.request("/start")[0], 409)
        headers = {"X-Slam-Session": started["session"], "Content-Type": "application/octet-stream"}
        self.assertEqual(self.request("/frame", b"x", headers)[0], 413)
        with socket.create_connection(("127.0.0.1", self.server.server_port), timeout=3) as connection:
            connection.sendall(("POST /frame HTTP/1.0\r\nAuthorization: Bearer test-token\r\n"
                                f"X-Slam-Session: {started['session']}\r\n"
                                "Content-Type: application/octet-stream\r\nContent-Length: 32\r\n\r\nSLF1").encode())
            connection.shutdown(socket.SHUT_WR)
            self.assertIn(b"400", connection.recv(1024).split(b"\r\n")[0])
        self.assertEqual(self.camera.frames, 0)
        self.assertEqual(self.request("/frame", frame(), dict(headers, **{"Transfer-Encoding": "chunked"}))[0], 411)
        self.assertEqual(self.request("/frame", frame(), headers)[0], 200)
        code, result = self.request("/stop", headers=headers)
        self.assertEqual(code, 200)
        self.assertEqual(result["state"], "ended")
        self.assertEqual(self.camera.process.returncode, 0)

    def test_tls_session(self):
        with tempfile.TemporaryDirectory(prefix="slam-tls-") as directory:
            cert, key = Path(directory) / "cert.pem", Path(directory) / "key.pem"
            subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
                            "-keyout", str(key), "-out", str(cert), "-days", "1",
                            "-subj", "/CN=127.0.0.1", "-addext", "subjectAltName=IP:127.0.0.1"],
                           check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.server.tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            self.server.tls.load_cert_chain(cert, key)
            context = ssl.create_default_context(cafile=str(cert))
            self.serve()
            code, started = self.request("/start", context=context)
            self.assertEqual(code, 200)
            headers = {"X-Slam-Session": started["session"], "Content-Type": "application/octet-stream"}
            self.assertEqual(self.request("/frame", frame(), headers, context)[0], 200)
            self.assertEqual(self.request("/stop", headers=headers, context=context)[1]["state"], "ended")


if __name__ == "__main__":
    unittest.main()
