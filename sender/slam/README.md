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
