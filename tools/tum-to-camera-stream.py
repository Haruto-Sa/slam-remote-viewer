#!/usr/bin/env python3
"""Replay TUM rgb.txt images as SLF1 Gray8 frames; requires Python OpenCV."""

import argparse
from decimal import Decimal
from pathlib import Path
import struct
import sys

import cv2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence", type=Path)
    args = parser.parse_args()
    frame_id = 0
    with (args.sequence / "rgb.txt").open() as index:
        for line in index:
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            timestamp, relative_path = line.split()
            image = cv2.imread(str(args.sequence / relative_path), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f"cannot read frame {relative_path}")
            height, width = image.shape
            timestamp_ns = int(Decimal(timestamp) * 1_000_000_000)
            sys.stdout.buffer.write(struct.pack(">4sIIQQ", b"SLF1", width, height,
                                                frame_id, timestamp_ns))
            sys.stdout.buffer.write(image.tobytes())
            frame_id += 1
    sys.stdout.buffer.flush()
    print(f"streamed {frame_id} frames", file=sys.stderr)


if __name__ == "__main__":
    main()
