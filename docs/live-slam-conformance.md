# Live SLAM backend conformance

This checklist evaluates a live monocular backend without changing Protocol v1
or Unity. It separates the MVP transport/display gate from later calibrated
accuracy and performance validation.

## Required path

```text
camera -> backend adapter -> boundary v1 -> Rust Sender
       -> Protocol v1 ZeroMQ -> Receiver / Unity
```

The MVP session may use provisional ORB settings. It only needs enough textured
motion to initialize tracking and display at least one live pose.

## Hard correctness gates

- settings arrive before pose and advertise metres, `slam_world`, `Twc`, and
  `xyzw`;
- session ID, frame ID, timestamp, pose, and tracking state survive the C++ and
  Rust boundaries without rewriting;
- timestamps and frame IDs are monotonic and every numeric value is finite;
- Ctrl-C and finite completion leave no camera, SLAM, Sender, or socket process;
- no image bytes are transmitted or saved by the telemetry path.

Failure of any hard gate rejects the backend.

## MVP pass criteria

- the camera starts at the requested mode;
- Protocol v1 settings reach the Receiver before pose;
- at least one valid live pose reaches the Receiver and is visible in Unity;
- invalid input does not reach ZeroMQ;
- finite completion or Ctrl-C shuts down cleanly.

If ORB-SLAM3 is stuck inside a tracking call, Ctrl-C must still terminate the
process within five seconds through the documented watchdog. This is recorded
as a forced upstream shutdown rather than a clean `session_end`.

Calibration quality, coordinate-direction exercises, loss/recovery behavior,
and performance thresholds are recorded as follow-up work and do not block the
first live demonstration.

## Recommended post-MVP performance gates

Measure after the first valid pose and report both the value and hardware:

- observed camera input FPS is at least 90% of the requested FPS;
- camera-reported dropped frames are at most 10% of captured plus dropped
  frames;
- processed FPS is at least 10 and mean tracking time is at most 100 ms;
- at least 50% of post-initialization frames contain a valid pose during the
  textured motion portion;
- memory does not grow continuously during the 30-second run.

These are operational baseline thresholds, not SLAM accuracy claims. A miss
requires a documented follow-up and blocks promotion unless the threshold is
explicitly revised with measurements from the primary backend.

## Evidence to retain

Retain text logs only: tool versions, git commit, device name (omit persistent
machine-specific IDs from the repository), capture mode, calibration RMS when available,
frame/pose/drop/state-transition counts, FPS and tracking time, Receiver
accepted/rejected counts, shutdown result, and the Unity coordinate checklist.
Calibration inputs and camera images remain local and must not be committed.

## 2026-09-03 MVP result

The Apple Silicon MacBook Air live path was verified at `640x480@30` with
provisional ORB settings. The camera produced 900 frames, ORB-SLAM3 produced
643 valid poses, and the camera reported zero drops. Observed input and
processing rates were 30.012 FPS and 30.046 FPS; mean tracking time was
10.499 ms. The Rust Sender received all 900 tracking events, published 643
poses, skipped 257 pose-less initialization frames, and ended the session
cleanly. `packet_dump --pose-only` observed live Protocol v1 pose sequence
numbers through 899.

This passed the camera-to-Protocol-v1 portion of the MVP. Because provisional
intrinsics were used, this run makes no pose-accuracy or coordinate-direction
claim.

A second computer connected to the sender Mac and accepted settings plus live
pose sequence numbers 120 through 185, confirming the network and Receiver
portion of the path. The initially attempted LAN address received no messages;
using the sender Mac's actual address resolved the issue without a code change.
That run established the network path before the final Unity visual check.

The Ctrl-C path was also exercised during a 9,000-frame live run. The normal
cooperative path printed `Shutdown`, sent `session_end` with reason
`interrupted`, stopped the Rust Sender, and returned in under 0.04 seconds. The
five-second watchdog remains the bounded fallback for the observed class of
upstream `TrackMonocular` stalls.

## ORB-SLAM3 worker teardown regression

A later TUM adapter replay delivered all tracking and point-cloud boundary
events and printed its success summary, then aborted while a background worker
attempted to lock an already-destroyed mutex. Inspection showed that the first
worker-wait patch had landed inside a commented upstream shutdown block and was
not executable. The corrected patch waits for worker completion and joins the
Local Mapping, Loop Closing, global bundle adjustment, and optional Viewer
thread handles. Use `tools/repeat-test-orbslam3-shutdown.sh` for ten consecutive
exit-code-zero replays before accepting the fix.

The corrected patch passed ten consecutive Apple Silicon TUM replays on
2026-09-05. Every run processed 798 frames, produced 794 to 796 valid poses,
printed the success summary, and returned exit code zero. A separate live-camera
run received Ctrl-C after 722 frames, immediately printed `Shutdown`, sent an
orderly `session_end` with reason `interrupted`, and left no producer or boundary
diagnostic process running.

## 2026-09-05 complete two-computer MVP result

The complete live path was verified from the Apple Silicon sender through a
second computer:

```text
macOS camera -> ORB-SLAM3 -> boundary v1 -> Rust Sender
             -> Protocol v1 ZeroMQ -> Receiver -> Unity
```

At `640x480@30`, the finite 900-frame session produced 72 valid poses, two
non-empty point-cloud deltas, 12 camera drops, and one tracking-state change.
The Rust Sender reported all 900 tracking events, published 72 poses and two
point-cloud messages, skipped 828 pose-less frames, received an orderly
`session_end`, and stopped normally. The Receiver on the second computer
accepted the live session, and Unity displayed the resulting point cloud. This
completes the MVP camera-to-Unity gate for settings, pose, point-cloud delta,
Protocol v1 conversion, and cross-computer delivery.

The run used the TUM1 example intrinsics with a different physical camera. It
therefore proves transport and visualization, not pose accuracy, scale accuracy,
or calibrated point-cloud quality. The relatively low pose count is retained as
operational evidence rather than hidden by an MVP threshold.

Start the Receiver and Unity subscriber first, then the Rust Sender, any local
`packet_dump`, and finally the C++ producer. Only TCP port 5555 must be reachable
on the sender Mac; Receiver output ports 5556 and 5557 remain local to the Unity
computer. A `packet_dump` timeout means that it did not observe all expected
topics during its five-minute window. In particular, starting it without a live
producer and waiting past that window is expected to time out and does not by
itself indicate a broken Sender or network path.

For operational sessions, pass `FRAME_LIMIT=0` to keep the camera producer
running until Ctrl-C. Positive values retain finite, repeatable runs. The
remaining work is post-MVP operational quality: sender-side diagnostics and
controls, optional camera calibration, and tracking quality improvements. None
changes the completed Protocol v1 E2E result.
