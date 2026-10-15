# Streamed camera input

This is the host input stage for smartphone capture (#106, #110). It accepts
real image frames and runs ORB-SLAM3, without AVFoundation or a host webcam.
The [phone capture client and HTTPS ingress](../sender/phone/README.md) feed this
format from the browser. Physical phone-to-Unity validation is a separate check;
a host dataset replay does not establish it.

## Local pipe format (SLF1)

Each frame is a 28-byte header followed immediately by tightly packed Gray8
pixels in row-major order. All integers are unsigned, big-endian.

| Offset | Bytes | Field |
| --- | --- | --- |
| 0 | 4 | ASCII `SLF1` (magic and version) |
| 4 | 4 | Width |
| 8 | 4 | Height |
| 12 | 8 | Capture frame ID |
| 20 | 8 | Capture timestamp in nanoseconds |
| 28 | width × height | Gray8 pixels |

Dimensions must match `Camera.width` and `Camera.height` in the supplied
ORB-SLAM3 calibration. Each dimension is at most 1920, and the image budget is
2,073,600 pixels (1080p in landscape or portrait). Resizing settings are rejected;
calibrate the actual transmitted resolution, lens and orientation. Do not rotate,
crop, mirror, zoom or switch lenses during a session. Gray8 conversion must retain
the capture geometry.

Frame IDs (at most 2^53−1) and timestamps (at most 2^63−1 ns) must strictly
increase, but may skip values when capture frames are dropped. Zero is a valid
first ID or timestamp. The producer subtracts the first capture timestamp for
boundary publication; receive time never substitutes for capture time.

EOF between complete frames ends the session cleanly. EOF within a frame,
unsupported magic/version, invalid dimensions or unordered metadata terminates
the producer with an error. Malformed headers are rejected before allocating a
pixel buffer. No input frame queue is added: pipe backpressure bounds buffering.
The ingress process must bound its own queue and close the pipe on disconnect or
idle timeout. The reader itself blocks on stdin and has no idle timer. A producer
failure closes the boundary abnormally rather than emitting a successful end.

## Run

Build the optional adapter following `sender/slam/README.md`; the executable is
`orbslam3_stream_camera_sender`. Start the Rust streamer in live mode first,
using a unique Unix socket, session and camera ID per camera.

```bash
camera_capture_command | /path/to/orbslam3_stream_camera_sender \
  /path/to/ORBvoc.txt /path/to/phone-camera.yaml \
  /private/tmp/phone-a.sock phone-a-session-001 phone-a-rear
```

`camera_capture_command` denotes a producer of the binary format above, not an
executable included in this stage. Start a new process and new session ID after
disconnect; never append a new camera session to an existing stream. The command
publishes tracking on every input frame and point-cloud differences once per
`Camera.fps` processed frames. The existing Rust Sender remains responsible for
Protocol v1 and network publication to Receiver/Unity.

For reproducible testing without a phone, use the TUM dataset documented in
`docs/slam-host.md` and Python OpenCV from the bootstrap environment:

```bash
set -o pipefail
/private/tmp/slam-remote-viewer-orbslam3/env/bin/python \
  tools/tum-to-camera-stream.py \
  /private/tmp/slam-remote-viewer-orbslam3/datasets/rgbd_dataset_freiburg1_xyz | \
  /private/tmp/slam-goal-orb-adapter/orbslam3_stream_camera_sender \
  /private/tmp/slam-remote-viewer-orbslam3/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /private/tmp/slam-remote-viewer-orbslam3/src/ORB_SLAM3/Examples/Monocular/TUM1.yaml \
  /private/tmp/phone-a.sock phone-a-test tum-camera
```

Use `slam_boundary_dump --socket /private/tmp/phone-a.sock --allow-pointcloud`
instead of the streamer to validate the local boundary. Logs and summaries go
to stdout/stderr; only the capture command's stdout contains binary frames.

Monocular ORB-SLAM3 has an unknown scale without an external reference. Although
the existing boundary labels coordinates as metres, this input does not recover
metric scale; do not interpret uncalibrated monocular distances as measurements.
No common coordinate alignment between separate phone maps is implied.
