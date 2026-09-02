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

This passes the camera-to-Protocol-v1 portion of the MVP. Receiver/Unity visual
confirmation remains a separate manual check. Because provisional intrinsics
were used, this run makes no pose-accuracy or coordinate-direction claim.

A second computer connected to the sender Mac and accepted settings plus live
pose sequence numbers 120 through 185, confirming the network and Receiver
portion of the path. The initially attempted LAN address received no messages;
using the sender Mac's actual address resolved the issue without a code change.
Unity visual confirmation remains outstanding.

The Ctrl-C path was also exercised during a 9,000-frame live run. The normal
cooperative path printed `Shutdown`, sent `session_end` with reason
`interrupted`, stopped the Rust Sender, and returned in under 0.04 seconds. The
five-second watchdog remains the bounded fallback for the observed class of
upstream `TrackMonocular` stalls.
