# SLAM adapter

This directory owns camera input and SLAM backend integration. Its public camera
contracts do not depend on ORB-SLAM3, OpenCV, AVFoundation, Rust, ZeroMQ, or
Unity types.

## Camera contracts

- `ImageFrame` owns immutable pixels plus a source frame ID, monotonic capture
  timestamp, dimensions, and explicit pixel format.
- `CameraCalibration` identifies the physical device and image dimensions and
  validates pinhole or fisheye intrinsics.
- `FrameSource` defines the lifecycle shared by recorded and live capture.
- `FrameSequenceValidator` rejects duplicate, reversed, or non-increasing frame
  IDs and timestamps without advancing its accepted reference.

Timestamps are nanoseconds from the source's monotonic clock. They are not wall
clock values and are not directly Protocol v1 session timestamps. The SLAM
session adapter establishes the session origin before publishing `t` seconds.

`Start`, `NextFrame`, and `Stop` belong to one capture thread. `RequestStop` is
the only operation required to be safe from another thread; an implementation
must use it to unblock a pending `NextFrame`. Frames own their storage, so a
backend may retain a frame after the source returns without referencing a
platform capture buffer.

The initial pixel formats are grayscale, BGR, and RGB with eight bits per
channel. Platform sources must convert other formats before constructing an
`ImageFrame`. ORB-SLAM3-specific `cv::Mat` conversion belongs in its backend
adapter, not in this contract.

## Build and test

The contract tests have no external dependencies:

```bash
cmake -S sender/slam -B sender/slam/build -DSLAM_BUILD_TESTS=ON
cmake --build sender/slam/build
ctest --test-dir sender/slam/build --output-on-failure
```

Build output under `sender/slam/build/` is ignored and must not be committed.

## Recorded monocular input

`RecordedFrameSource` reads a finite ordered sequence of binary PGM (`P5`)
files with 8-bit grayscale pixels. This deliberately dependency-free format is
the reference source for camera and SLAM tests before OpenCV is available.

Frame IDs are the zero-based position in the configured path list. Timestamps
are `frame_id * frame_period`, so playback never depends on filesystem metadata
or scheduler timing. Missing, malformed, truncated, or dimension-mismatched
files produce an observable recoverable error and do not prevent a later file
from being read. EOF, cancellation, timeout, and failure remain distinct.

Inspect a sequence after building:

```bash
./sender/slam/build/recorded_frame_dump \
  640 480 30 \
  path/to/000000.pgm path/to/000001.pgm
```

Expected output contains only deterministic metadata, for example:

```text
frame=0 timestamp_ns=0 size=640x480 bytes=307200
frame=1 timestamp_ns=33333333 size=640x480 bytes=307200
```

The diagnostic accepts placeholder pinhole intrinsics because it does not run
SLAM. A SLAM executable must instead load calibration produced and validated by
Issue #25.
