#!/usr/bin/env python3
"""Calibrate a pinhole monocular camera from checkerboard images."""

import argparse
import datetime
import glob
import pathlib
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", required=True, help="glob for checkerboard images")
    parser.add_argument("--device-id", required=True)
    parser.add_argument("--fps", required=True, type=int)
    parser.add_argument("--board-columns", required=True, type=int, help="inner corners")
    parser.add_argument("--board-rows", required=True, type=int, help="inner corners")
    parser.add_argument("--square-size-m", required=True, type=float)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.fps <= 0 or args.board_columns < 2 or args.board_rows < 2 or args.square_size_m <= 0:
        raise ValueError("FPS, board dimensions, and square size must be positive")
    try:
        import cv2
        import numpy as np
    except ImportError as error:
        print("OpenCV Python and NumPy are required: install an architecture-matched build", file=sys.stderr)
        print(error, file=sys.stderr)
        return 2

    paths = [pathlib.Path(value) for value in sorted(glob.glob(args.images))]
    if not paths:
        raise ValueError("image glob matched no files")

    object_template = np.zeros((args.board_columns * args.board_rows, 3), np.float32)
    object_template[:, :2] = np.mgrid[0 : args.board_columns, 0 : args.board_rows].T.reshape(-1, 2)
    object_template *= args.square_size_m
    object_points = []
    image_points = []
    image_size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"cannot read image: {path}")
        current_size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            raise ValueError(f"image dimensions differ: {path}")
        found, corners = cv2.findChessboardCorners(
            image, (args.board_columns, args.board_rows), None
        )
        if not found:
            continue
        refined = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), criteria)
        object_points.append(object_template.copy())
        image_points.append(refined)

    if len(image_points) < 10:
        raise ValueError(f"at least 10 usable checkerboard views are required; found {len(image_points)}")

    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    coefficients = distortion.ravel().tolist()
    if len(coefficients) < 5:
        coefficients.extend([0.0] * (5 - len(coefficients)))
    width, height = image_size
    calibrated_at = datetime.datetime.now(datetime.timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    content = f"""version=1
device_id={args.device_id}
width={width}
height={height}
fps={args.fps}
model=pinhole
fx={matrix[0, 0]:.17g}
fy={matrix[1, 1]:.17g}
cx={matrix[0, 2]:.17g}
cy={matrix[1, 2]:.17g}
distortion={','.join(f'{value:.17g}' for value in coefficients[:5])}
rms_reprojection_error_px={rms:.17g}
board_columns={args.board_columns}
board_rows={args.board_rows}
square_size_m={args.square_size_m:.17g}
calibrated_at_utc={calibrated_at}
source={args.images}
"""
    args.output.write_text(content, encoding="utf-8")
    print(f"wrote {args.output}: views={len(image_points)} size={width}x{height} rms_px={rms:.6f}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as error:
        print(f"calibration failed: {error}", file=sys.stderr)
        raise SystemExit(1)
