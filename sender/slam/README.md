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

## macOS live monocular input

On macOS, `MacosCameraSource` uses AVFoundation behind the common `FrameSource`
interface. It selects a camera by AVFoundation `uniqueID`, negotiates an exact
width, height, and FPS, receives callbacks on a serial dispatch queue, converts
BGRA buffers to owned BGR8 frames, and reports timestamps relative to the first
captured sample.

List devices and the current permission state:

```bash
./sender/slam/build/macos_camera_dump --list
```

Request permission only as an explicit user action:

```bash
./sender/slam/build/macos_camera_dump --request-permission
```

Then capture a finite diagnostic session:

```bash
./sender/slam/build/macos_camera_dump DEVICE_ID 1280 720 30 100
```

The diagnostic executable embeds `NSCameraUsageDescription`. A packaged or
sandboxed application must also declare the macOS Camera capability/entitlement.
Denied permission, a missing device, and an unsupported mode are reported before
capture starts.

The callback queue discards late AVFoundation frames. The application queue is
also bounded and drops its oldest frame when full, preserving the freshest data
for real-time SLAM. `dropped_frames()` counts both AVFoundation-reported late
frames and application queue overflow. `RequestStop()` cancels a waiting
consumer, and `Stop()` drains the callback queue before releasing capture
objects.

Issue #24 was verified after explicitly granting camera access. The built-in
FaceTime HD camera delivered ten BGR8 frames at negotiated `1280x720@30` with
monotonic IDs and timestamps, zero reported drops, and clean finite shutdown.
The diagnostic also enumerated a Continuity Camera without persisting either
device's machine-specific unique ID in the repository.

## Monocular calibration

Capture at least ten sharp checkerboard views at the exact device, resolution,
and pixel mode used for SLAM. Cover the full image area with varied board
distance and orientation; avoid using near-duplicate views. The board arguments
are inner-corner counts, and square size is measured in metres.

After installing architecture-matched Python OpenCV and NumPy, calculate a
pinhole calibration:

```bash
./tools/calibrate_monocular.py \
  --images 'calibration-images/*.png' \
  --device-id 'AVFOUNDATION_UNIQUE_ID' \
  --fps 30 \
  --board-columns 9 \
  --board-rows 6 \
  --square-size-m 0.024 \
  --output camera.calibration
```

The output records device ID, dimensions, FPS, intrinsics, distortion,
reprojection RMS, board geometry, UTC generation time, and source glob. Validate
it and generate an ORB-SLAM3 monocular YAML file:

```bash
./sender/slam/build/calibration_convert camera.calibration orb-camera.yaml
```

The loader rejects missing, duplicate, unknown, non-finite, and wrongly typed
fields. A live source must use the same device ID, dimensions, and FPS as the
document. `example.calibration` is a format fixture only and must not be used as
real camera calibration. Machine-specific calibration output is intentionally
not committed.

The generated YAML uses BGR input (`Camera.RGB: 0`) and explicit ORB extractor
defaults. Review feature settings during the ORB-SLAM3 dataset Issue; calibration
conversion does not claim that the defaults are optimal for every camera.
