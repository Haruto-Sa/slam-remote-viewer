#!/usr/bin/env python3
"""Run an actual RGB-D producer through the Rust boundary validator on TUM data."""
import argparse
import bisect
import math
from pathlib import Path
import re
import subprocess
import tempfile
import time


def read_index(path):
    entries = []
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 2:
            raise ValueError(f"invalid image index: {path}")
        stamp = float(fields[0])
        if not math.isfinite(stamp) or stamp < 0 or (entries and stamp <= entries[-1][0]):
            raise ValueError(f"unordered/nonfinite image index: {path}")
        image = (path.parent / fields[1]).resolve()
        if not image.is_file() or any(c.isspace() for c in str(image)):
            raise ValueError(f"missing image or unsupported whitespace in path: {image}")
        entries.append((stamp, image))
    return entries


def associate(rgb, depth, tolerance=0.02):
    """Nearest unused depth in increasing order; never reuse a depth image."""
    times = [entry[0] for entry in depth]
    next_depth = 0
    pairs = []
    for stamp, image in rgb:
        at = bisect.bisect_left(times, stamp, lo=next_depth)
        candidates = [i for i in (at - 1, at) if next_depth <= i < len(depth)]
        if not candidates:
            continue
        selected = min(candidates, key=lambda i: (abs(times[i] - stamp), i))
        if abs(times[selected] - stamp) <= tolerance:
            pairs.append((stamp, image, *depth[selected]))
            next_depth = selected + 1
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--producer", type=Path, required=True)
    parser.add_argument("--validator", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=0, help="0 = all associated pairs")
    parser.add_argument("--timeout", type=float, default=600)
    args = parser.parse_args()
    if args.frames < 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("invalid frame limit or timeout")
    pairs = associate(read_index(args.dataset / "rgb.txt"), read_index(args.dataset / "depth.txt"))
    if args.frames:
        pairs = pairs[:args.frames]
    if not pairs:
        raise ValueError("no associated RGB-D frames")
    # Keep Unix socket paths short even when macOS TMPDIR is deeply nested.
    output = Path(tempfile.mkdtemp(prefix="slam-rtabmap-test-", dir="/tmp"))
    associations = output / "associations.txt"
    associations.write_text("".join(f"{t:.9f} {rgb} {d:.9f} {depth}\n" for t, rgb, d, depth in pairs))
    socket = output / "boundary.sock"
    print(f"artifacts={output} pairs={len(pairs)}", flush=True)
    with (output / "boundary.log").open("w") as boundary_log, (output / "producer.log").open("w") as producer_log:
        validator = subprocess.Popen([str(args.validator.resolve()), "--socket", str(socket),
                                      "--allow-pointcloud"], stdout=boundary_log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while not socket.exists():
                if validator.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("boundary validator did not start")
                time.sleep(0.01)
            # TUM recommends the ROS default pinhole model for pre-registered
            # depth: do not independently undistort only the color images.
            subprocess.run([str(args.producer.resolve()), str(associations), str(socket),
                            "rtabmap-dataset-test", "tum-rgbd", "525", "525", "319.5", "239.5",
                            "5000", "30", str(len(pairs))], stdout=producer_log,
                           stderr=subprocess.STDOUT, timeout=args.timeout, check=True)
            if validator.wait(timeout=10) != 0:
                raise RuntimeError("Rust boundary validation failed")
        finally:
            if validator.poll() is None:
                validator.terminate()
                try:
                    validator.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    validator.kill()
                    validator.wait()
    summary = (output / "boundary.log").read_text()
    match = re.search(r"session_ended .*reason=shutdown frames=(\d+) poses=(\d+) pointcloud_deltas=(\d+) pointcloud_operations=(\d+)", summary)
    if not match:
        raise RuntimeError("no clean boundary session completion")
    frames, poses, deltas, operations = map(int, match.groups())
    if frames != len(pairs) or poses < frames * 0.9 or deltas < 1 or operations < 1:
        raise RuntimeError(f"insufficient tracking/map output: {match.group()}")
    print(match.group())
    print("PASS: real backend and Rust boundary; this is not a pose-accuracy or Unity rendering test")


if __name__ == "__main__":
    main()
