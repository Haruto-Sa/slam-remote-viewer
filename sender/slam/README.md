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

## ORB-SLAM3 dependency build

On Apple Silicon, use `tools/bootstrap-orbslam3-macos.sh` from the repository
root. It creates a disposable native arm64 dependency graph outside the
repository, checks out the revisions in `orbslam3/dependencies.lock.sh`, and
verifies that the resulting libraries do not link against Intel Homebrew. The
upstream checkout and build products are intentionally not CMake targets of this
camera-contract project.

## ORB-SLAM3 pose adapter

`slam::TrackingResult` is the backend-independent output of monocular tracking.
It preserves the input frame ID and monotonic nanosecond timestamp, reports
initializing, tracking, lost, or relocalizing, and contains canonical `Twc` only
when ORB-SLAM3 reports valid tracking. Lost and relocalizing frames never reuse
the last valid transform. Positions are metres and quaternions use `[x,y,z,w]`.
ORB-SLAM3, Sophus, Eigen, and OpenCV types remain private to the optional adapter.

Build the adapter against the disposable arm64 tree created above:

```bash
cmake -S sender/slam -B /private/tmp/slam-pose-adapter \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DSLAM_ENABLE_ORB_SLAM3=ON \
  -DSLAM_ORB_SLAM3_ROOT=/private/tmp/slam-remote-viewer-orbslam3/src/ORB_SLAM3 \
  -DCMAKE_PREFIX_PATH='/private/tmp/slam-remote-viewer-orbslam3/install;/private/tmp/slam-remote-viewer-orbslam3/env' \
  -DOpenCV_DIR=/private/tmp/slam-remote-viewer-orbslam3/env/lib/cmake/opencv4 \
  -DPangolin_DIR=/private/tmp/slam-remote-viewer-orbslam3/install/lib/cmake/Pangolin
cmake --build /private/tmp/slam-pose-adapter
```

Replay the TUM sequence prepared in `docs/slam-host.md` through the adapter:

```bash
/private/tmp/slam-pose-adapter/orbslam3_dataset_pose_dump \
  /private/tmp/slam-remote-viewer-orbslam3/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /private/tmp/slam-remote-viewer-orbslam3/src/ORB_SLAM3/Examples/Monocular/TUM1.yaml \
  /private/tmp/slam-remote-viewer-orbslam3/datasets/rgbd_dataset_freiburg1_xyz
```

The verified M1 run processed 798 frames, emitted 796 valid poses, preserved
every frame ID and timestamp, and shut down without writing trajectory or map
files. The viewer defaults to disabled because the telemetry process is
headless. Saving trajectories remains an explicit responsibility of a future
diagnostic, not a tracker shutdown side effect.

## Live streamer boundary publisher

`boundary::Publisher` connects to the Unix stream listener owned by the Rust
Sender and writes boundary v1 frames synchronously. It has no application
queue. A missing listener, send timeout, or disconnect closes the connection;
the SLAM process never accumulates telemetry in memory. Strings, JSON-safe IDs,
session-relative timestamps, finite translations, and unit quaternions are
validated before writing. Protocol v1 and ZeroMQ remain entirely on the Rust
side.

Start the validating Rust diagnostic first:

```bash
cargo run --manifest-path sender/streamer/Cargo.toml \
  --bin slam_boundary_dump -- --socket /private/tmp/slam-live.sock --allow-pointcloud
```

Then append socket/session/camera/FPS arguments to the dataset replay command:

```bash
/private/tmp/slam-pose-adapter/orbslam3_dataset_pose_dump \
  /private/tmp/slam-remote-viewer-orbslam3/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /private/tmp/slam-remote-viewer-orbslam3/src/ORB_SLAM3/Examples/Monocular/TUM1.yaml \
  /private/tmp/slam-remote-viewer-orbslam3/datasets/rgbd_dataset_freiburg1_xyz \
  /private/tmp/slam-live.sock session-id camera-id 30 30
```

The final argument is the positive map-point snapshot period in frames; it
defaults to 30 when omitted. The producer copies active ORB-SLAM3 map-point IDs
and world coordinates into backend-independent values, reduces snapshots to
bounded deltas, and publishes only non-empty deltas. The diagnostic reports
tracking and point-cloud counts after an orderly session end. No image bytes,
ORB-SLAM3 types, or Protocol v1 JSON cross this boundary.

The 2026-09-03 M1 dataset check delivered all 798 tracking frames plus 26
non-empty deltas (77,175 total add/update/remove operations) to the Rust adapter
and completed with an orderly `session_end`.

## Point-cloud delta reducer

`PointCloudDeltaReducer` converts backend-independent map-point snapshots into
deterministic add, update, and remove operations. It retains one baseline but
has no work queue. Snapshot points and each delta category are ordered by stable
numeric ID, independent of backend iteration order.

The configurable snapshot and delta-operation limits bound retained and emitted
data. IDs must fit JSON's safe integer range and coordinates must be finite.
Duplicate IDs, invalid values, and limit violations fail transactionally without
changing the last valid baseline. `Reset()` clears the baseline so the next
snapshot is emitted as adds. ORB-SLAM3 extraction, boundary IPC, Protocol v1,
and Unity remain outside this reducer.

## Live camera to the streamer boundary

`orbslam3_macos_camera_sender` connects the macOS camera source, ORB-SLAM3 pose
and map-point adapter, reducer, and boundary publisher. For an MVP transport/display check, it accepts
the device ID and capture mode directly. The ORB settings file still controls
SLAM intrinsics. A camera-specific calibration is recommended before evaluating
pose accuracy, but is not required to exercise the live telemetry path.

Start `packet_dump --pose-only`, then the Rust Sender, before starting the C++
producer:

```bash
cargo run --manifest-path sender/streamer/Cargo.toml \
  --example packet_dump -- 'tcp://127.0.0.1:5555' --pose-only

cargo run --manifest-path sender/streamer/Cargo.toml \
  --bin slam-mock-sender -- --source live \
  --slam-socket /private/tmp/slam-live.sock \
  --endpoint 'tcp://127.0.0.1:5555'

/private/tmp/slam-pose-adapter/orbslam3_macos_camera_sender \
  /private/tmp/slam-remote-viewer-orbslam3/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  orb-camera.yaml DEVICE_ID 1280 720 30 /private/tmp/slam-live.sock \
  live-session mac-camera 900 30
```

The optional final value is the positive map-point snapshot period in frames
(default 30). The finite frame limit makes the session repeatable; Ctrl-C also
requests a clean stop. The final log reports captured frames, valid poses,
point-cloud deltas, camera drops, tracking-state transitions, observed input
FPS, processed FPS, and mean ORB-SLAM3 tracking time. No image is sent or saved.

The Rust Sender currently publishes the live poses as Protocol v1 but does not
yet convert boundary point-cloud deltas to `slam/v1/pointcloud`; keep using
`packet_dump --pose-only` until that follow-up is implemented.

Ctrl-C is cooperative while `TrackMonocular` is returning normally. If an
upstream ORB-SLAM3 call does not return, a five-second watchdog terminates the
process with exit code 130 so the camera and process cannot remain stuck. That
forced path cannot send `session_end`; the Rust Sender will report a producer
disconnect, which distinguishes it from a clean shutdown. `Ctrl-D` is not a
process-stop operation.

For the MVP, verify settings precede any pose, the Receiver accepts the session,
and no camera/SLAM process remains after shutdown. Coordinate accuracy,
calibration RMS, controlled-axis motion, and quantitative performance thresholds
are follow-up validation rather than blockers for the first live display.

The 2026-09-03 Apple Silicon MVP run at `640x480@30` delivered 643 live poses
from 900 camera frames with zero camera drops. Rust published those poses as
Protocol v1 and ended cleanly. See
[`../../docs/live-slam-conformance.md`](../../docs/live-slam-conformance.md) for
the measurements and the remaining Unity visual check.

### Two-computer Unity MVP check

On the Receiver/Unity computer, open the Unity project and enter Play Mode. Its
default subscriber connects to `tcp://127.0.0.1:5556`. Then start the Receiver,
replacing `MAC_IP` with the sender Mac's LAN address:

```bash
cargo run --manifest-path receiver/Cargo.toml -- \
  --endpoint 'tcp://MAC_IP:5555' \
  --output-endpoint 'tcp://127.0.0.1:5556' \
  --control-endpoint 'tcp://127.0.0.1:5557'
```

On the sender Mac, start the Rust Sender bound to all interfaces, then run the
C++ command above with the same socket path:

```bash
cargo run --manifest-path sender/streamer/Cargo.toml \
  --bin slam-mock-sender -- --source live \
  --slam-socket /private/tmp/slam-live.sock \
  --endpoint 'tcp://*:5555'
```

Allow inbound TCP port 5555 on the sender Mac; ports 5556 and 5557 stay local to
the Receiver/Unity computer. Move the monocular camera slowly sideways while
keeping a textured scene visible so ORB-SLAM3 can initialize. The MVP passes
when the Receiver accepts settings and at least one pose, Unity shows the live
camera/frustum moving, and all processes exit after the finite frame limit or
Ctrl-C.
