#!/usr/bin/env python3
"""Exercise trusted HTTPS -> streamed ORB-SLAM3 -> Rust boundary using TUM frames."""

import argparse
from decimal import Decimal
import http.client
import json
from pathlib import Path
import re
import ssl
import struct
import subprocess
import tempfile
import threading
import time

import cv2

from server import CameraServer, CameraSession


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--producer", required=True)
    parser.add_argument("--orb-root", required=True, type=Path)
    parser.add_argument("--sequence", required=True, type=Path)
    parser.add_argument("--boundary-dump", required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="slam-phone-", dir="/private/tmp") as directory:
        root = Path(directory)
        socket = root / "boundary.sock"
        cert, key = root / "cert.pem", root / "key.pem"
        subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
                        "-keyout", str(key), "-out", str(cert), "-days", "1",
                        "-subj", "/CN=127.0.0.1", "-addext", "subjectAltName=IP:127.0.0.1"],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        validator = subprocess.Popen([args.boundary_dump, "--socket", str(socket), "--allow-pointcloud"],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        camera = CameraSession([args.producer, str(args.orb_root / "Vocabulary/ORBvoc.txt"),
                                str(args.orb_root / "Examples/Monocular/TUM1.yaml"), str(socket),
                                root.name, "https-tum-camera"], 640, 480, 30)
        server = CameraServer(("127.0.0.1", 0), camera, "local-test-token")
        server.tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        server.tls.load_cert_chain(cert, key)
        context = ssl.create_default_context(cafile=str(cert))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        lease = ""

        def post(endpoint, body=b""):
            connection = http.client.HTTPSConnection("127.0.0.1", server.server_port, context=context, timeout=60)
            try:
                connection.request("POST", endpoint, body, {"Authorization": "Bearer local-test-token",
                                   "X-Slam-Session": lease, "Content-Type": "application/octet-stream"})
                response = connection.getresponse()
                result = json.loads(response.read())
                if response.status != 200:
                    raise RuntimeError(f"{endpoint}: {response.status} {result}")
                return result
            finally:
                connection.close()

        try:
            deadline = time.monotonic() + 10
            while not socket.exists():
                if validator.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("Rust boundary did not become ready")
                time.sleep(.02)
            lease = post("/start")["session"]
            count = 0
            with (args.sequence / "rgb.txt").open() as index:
                for line in index:
                    if not line.strip() or line.lstrip().startswith("#"):
                        continue
                    timestamp, relative = line.split()
                    gray = cv2.imread(str(args.sequence / relative), cv2.IMREAD_GRAYSCALE)
                    if gray is None or gray.shape != (480, 640):
                        raise RuntimeError(f"invalid input image: {relative}")
                    payload = struct.pack(">4sIIQQ", b"SLF1", 640, 480, count,
                                          int(Decimal(timestamp) * 1_000_000_000)) + gray.tobytes()
                    post("/frame", payload)
                    count += 1
            result = post("/stop")
            if result["state"] != "ended" or result["frames"] != count:
                raise RuntimeError(f"unclean camera shutdown: {result}")
            output, _ = validator.communicate(timeout=10)
            print(output)
            summary = re.search(r"reason=camera_stream_eof frames=(\d+) poses=(\d+) pointcloud_deltas=(\d+)", output)
            if validator.returncode != 0 or not summary or int(summary[1]) != count or min(int(summary[2]), int(summary[3])) <= 0:
                raise RuntimeError("Rust validator did not confirm frames, poses, points and clean shutdown")
            print(f"HTTPS real SLAM replay passed: {count} frames")
        finally:
            server.shutdown()
            thread.join()
            server.server_close()
            camera.close()
            if validator.poll() is None:
                validator.terminate()
                try:
                    validator.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    validator.kill()
                    validator.wait()
            validator.stdout.close()


if __name__ == "__main__":
    main()
