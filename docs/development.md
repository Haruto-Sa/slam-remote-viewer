# Development guide

## Working agreement

This repository uses a small team workflow even when developed by one person.
Each change starts from an Issue and ends with a reviewed pull request.

```text
Issue -> branch -> implementation -> tests -> self-review -> PR -> merge
```

One Issue should produce one independently verifiable behavior. Avoid combining
transport, visualization, and recording in one pull request.

## Roles to simulate

- Product owner: defines the user-visible outcome and priority.
- Tech lead: owns architecture and interface decisions.
- Developer: implements an Issue and records assumptions.
- Reviewer: checks correctness, scope, tests, and documentation.
- QA: verifies the Issue's acceptance criteria without relying on internals.

One person can switch roles, but should perform developer and reviewer passes at
different times and explicitly record the review result on the PR.

## Git workflow

Branch names use the Issue number:

```text
feature/<issue-number>-<short-name>
fix/<issue-number>-<short-name>
docs/<issue-number>-<short-name>
```

Commit messages describe one completed change, for example:

```text
docs: define telemetry protocol v1
feat(sender): publish mock pose telemetry
test(receiver): cover coordinate conversion
```

PR descriptions include:

- `Closes #<issue-number>`
- what changed and why;
- how it was tested;
- known limitations or follow-up Issues.

## Definition of Done

An Issue is done when:

- its acceptance criteria are satisfied;
- relevant automated tests pass;
- new public behavior is documented;
- logs contain enough context to diagnose invalid input;
- no unrelated refactoring is included;
- the diff has received a reviewer pass;
- the branch is mergeable.

## Issues and backlog

- #1 Define telemetry Protocol v1, including the coordinate-system contract.
- #4 Implement a Rust Mock Sender for deterministic telemetry.
- #7 Implement the Rust ZeroMQ telemetry subscriber.
- #9 Parse and validate Protocol v1 telemetry.
- #11 Normalize pose quaternions and preserve sign continuity.
- #13 Convert `slam_world` coordinates to `unity_world`.
- #16 Republish converted telemetry to Unity over ZeroMQ.
- #17 Define a backend-independent SLAM pose source interface.
- #20 Implement the Unity background subscriber and main-thread queue.
- #21 Audit the macOS SLAM host and pin the native toolchain.
- #22 Define camera frame-source and calibration contracts.
- #23 Implement a deterministic recorded monocular frame source.
- #24 Implement a macOS live monocular camera source.
- #25 Add a monocular camera calibration workflow and validation.
- #32 Render the camera pose and frustum in Unity.
- #34 Retain and render trajectory history in Unity.
- #36 Apply and render point-cloud deltas in Unity.
- #39 Record telemetry sessions and export the final point cloud as PLY.
- #42 Replay recorded telemetry sessions into Unity.
- #45 Add a Unity telemetry diagnostics overlay.

Future work receives its number when the GitHub Issue is created:

- Connect the real SLAM adapter.

Each Issue must leave the system runnable. Until the real SLAM adapter exists,
the Mock Sender is the reference producer used by Receiver and Unity tests.

## Issue 1: Protocol v1

Goal: define a stable contract shared by Sender, Receiver, and Unity.

Acceptance criteria:

- ZeroMQ framing and topics are documented.
- Settings, pose, and point-cloud schemas have valid examples.
- `Twc`, units, axes, handedness, and quaternion order are explicit.
- validation and sequence handling behavior is explicit.
- examples parse as JSON.

## Issue 4: Mock Sender

Goal: publish deterministic telemetry without a camera or SLAM installation.

Proposed behavior:

- bind a configurable PUB endpoint, defaulting to `tcp://*:5555`;
- publish settings repeatedly;
- publish a camera moving in a horizontal circle at a configurable rate;
- periodically add point-cloud fixtures;
- use no random input and support an optional finite duration;
- exit cleanly on Ctrl-C.

Acceptance criteria:

- a packet-dump subscriber sees all three Protocol v1 topics;
- emitted payloads match the examples and validation rules;
- two runs with identical arguments produce identical telemetry;
- unit tests cover sequence numbers and serialization.

Suggested branch:

```text
feature/4-mock-sender
```

## Issue 7: Receiver subscriber

Goal: receive Protocol v1 telemetry from the Mock Sender over ZeroMQ.

Implemented behavior:

- connect a configurable SUB socket, defaulting to
  `tcp://127.0.0.1:5555`;
- subscribe to the `slam/v1/` topic prefix;
- receive topic and JSON payload as two multipart frames;
- reject invalid frame counts, UTF-8, and JSON syntax without
  panicking;
- report received and rejected message counts;
- exit cleanly on Ctrl-C.

Acceptance criteria:

- the Receiver observes settings, pose, and point-cloud topics from the Mock
  Sender;
- malformed multipart messages do not terminate the process;
- the endpoint can be configured from the command line;
- unit tests cover framing and encoding failures.

Suggested branch:

```text
feature/7-receiver-subscriber
```

## Issue 9: Protocol v1 parsing and validation

Goal: deserialize received payloads into typed Protocol v1 messages and reject
invalid telemetry before state handling or coordinate conversion.

Implemented behavior:

- deserialize settings, pose, and point-cloud payloads based on the exact
  topic;
- ignore unknown JSON fields for forward compatibility;
- validate the protocol version, session, fixed settings values, timestamps,
  numeric values, pose state, and point IDs;
- reject payloads larger than 16 MiB before JSON parsing;
- log accepted messages with topic, session, and sequence metadata instead of
  the complete payload;
- count and log invalid messages without terminating the Receiver.

Acceptance criteria:

- all three examples under `protocol/` deserialize and validate;
- missing fields, wrong types, unsupported versions, and invalid values are
  rejected;
- unknown fields remain accepted;
- the Mock Sender produces valid typed telemetry for all three topics;
- automated tests and Clippy complete without warnings.

Suggested branch:

```text
feature/9-protocol-validation
```

## Issue 11: Quaternion normalization and sign continuity

Goal: produce stable unit pose quaternions before coordinate conversion and
Unity visualization.

Implemented behavior:

- normalize Protocol v1 quaternions while preserving `[x, y, z, w]` order;
- avoid overflow while normalizing large finite components;
- reject non-finite, zero-length, and near-zero-length quaternions;
- compare consecutive normalized quaternions and negate the current value when
  their dot product is negative;
- reset the previous-quaternion reference when the session changes;
- reject quaternion-processing failures without terminating the Receiver.

Acceptance criteria:

- every accepted pose quaternion has unit length within the tested tolerance;
- equivalent `q` and `-q` inputs produce sign-continuous output;
- consecutive output quaternions have a non-negative dot product;
- a session change clears the continuity reference;
- invalid input does not replace the previous valid quaternion;
- the Mock Sender remains compatible with the integrated Receiver;
- automated tests and Clippy complete without warnings.

Suggested branch:

```text
feature/11-quaternion-continuity
```

## Issue 13: SLAM-to-Unity coordinate conversion

Goal: convert validated telemetry from the canonical `slam_world` frame into
the `unity_world` frame before local publication and visualization.

Implemented behavior:

- convert pose positions from `[x, y, z]` to `[x, -y, z]`;
- convert normalized pose quaternions from `[x, y, z, w]` to
  `[-x, y, -z, w]`;
- apply the position conversion to point-cloud `add` and `update` entries;
- preserve point IDs, point-cloud removals, and telemetry metadata;
- rewrite validated settings from `frame: "slam_world"` to
  `frame: "unity_world"`;
- apply conversion in the Receiver after validation and normalization, then
  preserve quaternion sign continuity in Unity coordinates.

Acceptance criteria:

- the documented position, quaternion, and point fixtures convert exactly;
- known positive-axis movements and 90-degree rotations convert correctly;
- settings advertise `unity_world` only after their `slam_world` input has
  passed validation;
- pose and point-cloud metadata remain unchanged;
- automated tests and Clippy complete without warnings.

Suggested branch:

```text
feature/13-coordinate-conversion
```

## Issue 16: Unity telemetry republisher

Goal: publish validated and converted telemetry from the Receiver on a local
ZeroMQ endpoint for the Unity Viewer.

Implemented behavior:

- bind a PUB socket to configurable `--output-endpoint`, defaulting to
  `tcp://127.0.0.1:5556`;
- serialize converted typed messages as Protocol v1 JSON;
- publish the original topic and JSON payload as two multipart frames;
- retain and repeat the latest converted Settings message every five seconds;
- continue receiving after publication failures and report failure counts;
- exclude complete payloads from publication-error logs.

Acceptance criteria:

- settings, pose, and point-cloud messages retain their Protocol v1 topics;
- republished settings advertise `frame: "unity_world"`;
- invalid telemetry is never passed to the publisher;
- a late subscriber can recover the latest Settings message;
- the output endpoint is configurable;
- automated tests and Clippy complete without warnings.

Suggested branch:

```text
feature/16-unity-republisher
```

## Issue 17: SLAM pose source interface

Goal: decouple Protocol v1 publishing from concrete SLAM implementations by
introducing a reusable pose source contract.

Implemented behavior:

- represent a SLAM pose with frame ID, timestamp, translation, quaternion, and
  tracking state;
- expose poses through a backend-independent `PoseSource` trait;
- generate the deterministic circular fixture through `MockPoseSource`;
- map frame ID, timestamp, pose values, and tracking state into Protocol v1;
- keep backend-specific and ORB-SLAM3 types out of the network layer.

Acceptance criteria:

- the Mock Sender uses the same interface intended for a real backend;
- SLAM pose fields serialize to the expected Protocol v1 pose payload;
- source coordinate assumptions match canonical `Twc` in `slam_world`;
- finite sessions preserve the existing deterministic pose count;
- automated tests and Clippy complete without warnings.

Suggested branch:

```text
feature/17-slam-pose-source
```

## Issue 20: Unity background subscriber and main-thread queue

Goal: receive converted Protocol v1 telemetry without blocking or calling Unity
APIs outside the main thread.

Implemented behavior:

- initialize the Unity `6000.2.4f1` project and restore NetMQ through
  NuGetForUnity;
- connect a configurable SUB socket, defaulting to
  `tcp://127.0.0.1:5556`, on a background thread;
- subscribe to the `slam/v1/` prefix and require exactly two UTF-8 frames;
- parse and validate settings, pose, and point-cloud JSON into immutable DTOs;
- reject telemetry until matching `unity_world` settings establish the active
  session;
- transfer accepted messages through a bounded, thread-safe queue with a
  deterministic drop-oldest overflow policy;
- drain the queue and invoke consumers from Unity `Update` on the main thread;
- cancel, join, and dispose the subscriber when its component is disabled.

Acceptance criteria:

- all three Protocol v1 topics reach the main-thread queue;
- invalid topics, payloads, coordinate settings, and sessions are rejected;
- a full queue remains bounded and reports dropped messages;
- no Unity API is called from the subscriber thread;
- startup and shutdown complete cleanly;
- Unity EditMode tests cover parsing, session gating, overflow, live NetMQ
  receipt, and shutdown.

Suggested branch:

```text
feature/20-unity-subscriber
```

## Issue 32: Unity camera pose and frustum

Goal: visualize the latest Unity-coordinate camera pose without adding scene
work to the network thread.

Implemented behavior:

- consume immutable Settings and Pose messages from the subscriber's
  main-thread event;
- apply Protocol v1 `Twc` position and `[x, y, z, w]` orientation to a camera
  marker;
- generate a wireframe frustum with a configurable vertical field of view,
  depth, and line width;
- derive the frustum aspect ratio from the Settings camera dimensions;
- distinguish tracking states with configurable colors;
- clear the previous pose when Settings establish a new session;
- ignore poses received before Settings or for another session;
- create default marker and frustum objects when scene references are empty.

Acceptance criteria:

- known position and rotation fixtures update the marker exactly;
- the frustum matches the Settings aspect ratio;
- a session change hides and resets the previous pose;
- all scene changes occur on Unity's main thread;
- EditMode tests cover geometry, pose application, and session handling.

Suggested branch:

```text
feature/32-unity-camera-frustum
```

## Issue 34: Unity trajectory history

Goal: retain a bounded history of tracking positions and render the traveled
path without drawing across tracking gaps.

Implemented behavior:

- consume Settings and Pose messages from the subscriber's main-thread event;
- retain positions only while the pose state is `tracking`;
- split the rendered path after `initializing`, `lost`, or `relocalizing`;
- discard positions closer than a configurable minimum distance;
- enforce a configurable point capacity with deterministic drop-oldest
  behavior;
- clear all points and rendered segments when the session changes;
- ignore poses received before Settings or for another session;
- create a default trajectory root and LineRenderers when scene references are
  empty;
- expose line material, color, width, sampling distance, and capacity in the
  Inspector.

Acceptance criteria:

- tracking positions render in arrival order;
- history remains bounded and removes its oldest points first;
- tracking gaps produce separate LineRenderer segments;
- a session change removes the previous trajectory;
- all scene changes occur on Unity's main thread;
- EditMode tests cover retention, sampling, overflow, gaps, and reset.

Suggested branch:

```text
feature/34-unity-trajectory
```

## Issue 36: Unity point-cloud deltas

Goal: maintain persistent Protocol v1 point state and render it with a bounded
number of Unity scene objects.

Implemented behavior:

- consume Settings and PointCloud messages from the subscriber's main-thread
  event;
- retain Unity-coordinate positions by point ID;
- apply each delta in remove, update, then add order;
- treat unknown updates as adds, duplicate adds as updates, and unknown removes
  as no-ops;
- clear point state and rendering when the session changes;
- ignore deltas received before Settings or for another session;
- sort positions by point ID for deterministic batched rendering;
- render all points through one ParticleSystem instead of per-point
  GameObjects;
- skip render-data rebuilds when a delta does not change state;
- expose material, color, size, and visibility in the Inspector.

Acceptance criteria:

- delta operations follow the Protocol v1 order and idempotent semantics;
- a session change removes all previous points;
- mismatched sessions do not mutate state;
- rendering uses one ParticleSystem regardless of point count;
- all scene changes occur on Unity's main thread;
- EditMode tests cover delta semantics, reset, batching, and render updates.

Suggested branch:

```text
feature/36-unity-point-cloud
```

## Issue 39: Session recording and PLY export

Goal: record accepted Unity-coordinate Protocol v1 telemetry and export the
final retained point cloud for offline inspection.

Implemented behavior:

- enable recording explicitly with the Receiver's `--record-dir` option;
- require Settings before Pose or PointCloud telemetry and reject messages for
  a session other than the active one;
- move serialization and file writes to a dedicated recording thread;
- create a separate, non-overwriting directory for every observed session;
- record accepted Settings, Pose, and PointCloud messages in arrival order as
  newline-delimited JSON;
- apply point-cloud changes in remove, update, then add order;
- retain points by ID and write final positions in ascending ID order;
- finalize ASCII PLY and JSON metadata files on session change or clean
  Receiver shutdown;
- report file-operation failures with the affected path and operation.

Acceptance criteria:

- recorded telemetry can be inspected without Unity;
- the PLY vertex count equals the retained point count;
- removed points are absent and updated points contain their latest positions;
- session changes produce separate output directories and point states;
- failed output paths produce actionable errors;
- automated tests cover session gating, delta semantics, deterministic export,
  message ordering, session reset, and write failures.

Suggested branch:

```text
feature/39-session-recording-ply
```

## Issue 42: Recorded telemetry session player

Goal: replay Receiver recordings into Unity for deterministic offline
inspection without a camera or SLAM backend.

Implemented behavior:

- provide a separate `slam-session-player` binary in the Receiver package;
- load `metadata.json` and `telemetry.ndjson` from one recorded session;
- require Protocol v1, `unity_world`, metres, a matching session, supported
  topics, and Settings as the first message;
- verify recorded message counts and reconstruct point-cloud deltas to verify
  the final point count;
- retain each payload's original JSON bytes and multipart topic;
- preserve file order and derive playback timing from Pose and PointCloud
  timestamps;
- scale timing with a positive finite playback-speed multiplier;
- clamp decreasing timestamps to the preceding message's playback time rather
  than reordering messages;
- wait after binding so Unity can subscribe before Settings is published;
- support Ctrl-C while waiting and during playback;
- include the input filename and line number in malformed-telemetry errors.

Acceptance criteria:

- a valid recording reproduces its trajectory and final point cloud in Unity;
- playback preserves topic, payload, and recorded message order;
- speed changes timing without modifying payloads;
- truncated recordings and inconsistent final point counts are rejected;
- tests require neither Unity nor camera/SLAM hardware;
- no files below `sender/slam` are changed.

Suggested branch:

```text
feature/42-telemetry-session-player
```

## Issue 45: Unity telemetry diagnostics overlay

Goal: expose live Viewer health in Unity so camera and SLAM integration can be
diagnosed without relying only on terminal logs.

Implemented behavior:

- maintain a testable diagnostics state model with stopped, waiting,
  receiving, and stale states;
- track the active session, latest pose state, and accepted-message age on
  Unity's main thread;
- expose the subscriber endpoint, running state, queue depth, counts, and
  latest rejection/fault information through read-only properties;
- read retained trajectory and point-cloud counts from their visualizers;
- tolerate missing optional visualizer references and report their counts as
  unavailable;
- draw an optional immediate-mode overlay without adding UI package or asset
  dependencies;
- keep telemetry processing and state updates active while drawing is hidden;
- add the overlay to the default Viewer scene with a configurable stale timeout,
  panel rectangle, font size, and visibility.

Acceptance criteria:

- state changes from waiting to receiving after accepted telemetry and becomes
  stale after the configured quiet period;
- stopped subscriber state, session reset, pose tracking state, counters, and
  optional visualizer counts are represented correctly;
- disabling overlay drawing does not disable its component or subscriber;
- EditMode tests cover status transitions, session reset, mismatches, counters,
  missing counts, and invalid time configuration;
- no files below `sender/slam` are changed.

Suggested branch:

```text
feature/45-unity-telemetry-diagnostics
```

## Issue 57: Reduce SLAM point-cloud telemetry on the Intel Mac sender

Goal: minimize live point-cloud traffic before Protocol v1 serialization while
preserving stable map-point identity and a recognizable visualization.

Implemented behavior:

- accept backend-independent snapshots containing stable point IDs and metre
  coordinates;
- deterministically retain the lowest point ID in each configurable voxel;
- cap retained points and report voxel and capacity filtering statistics;
- emit only Protocol v1-compatible add, update, and remove operations;
- suppress small updates relative to the last transmitted position, while
  accumulating movement until it crosses the configured threshold;
- clear retained state on SLAM session reset so the next snapshot is complete;
- reject duplicate IDs, non-finite coordinates, and IDs outside the Protocol v1
  JSON-safe integer range.

Acceptance criteria:

- unchanged and sub-threshold snapshots produce no telemetry operations;
- output ordering and voxel selection are deterministic regardless of input order;
- removal, representative changes, capacity limits, and reset are tested;
- ORB-SLAM3 types remain outside this reusable reduction layer;
- the existing camera and calibration tests remain successful.

Suggested branch:

```text
feature/57-intel-mac-pointcloud-delta
```

## Intel Mac task IM-SLAM-02: ORB-SLAM3 tracked-point adapter

Goal: copy ORB-SLAM3 output into backend-independent telemetry values without
exposing backend pointers to the reducer or network layers.

Implemented behavior:

- compile only when `SLAM_ENABLE_ORB_SLAM3=ON` and an explicit patched source
  root is supplied;
- use the public `System::GetTrackedMapPoints` API;
- omit null and bad MapPoints, copy stable IDs and world positions, and sort by
  ID before returning;
- keep the default camera build independent of ORB-SLAM3 and its GPL library.

Suggested branch:

```text
feature/intel-mac-orbslam3-adapter
```

## Intel Mac task IM-SLAM-03: ORB-SLAM3 monocular tracker

Goal: feed the shared live or recorded camera frame contract into ORB-SLAM3 and
return one backend-independent result for telemetry and diagnostics.

Implemented behavior:

- accept Gray8, BGR8, and RGB8 `ImageFrame` buffers without an extra image copy;
- preserve frame IDs and convert monotonic capture timestamps to seconds;
- convert ORB-SLAM3's `Tcw` result to the canonical `Twc` camera pose and xyzw
  quaternion expected by Protocol v1;
- map backend tracking states and retain the last valid pose while tracking is
  temporarily lost;
- attach the public tracked-point snapshot for downstream voxel reduction;
- reset the active map and cached pose together, and shut down backend threads
  exactly once.

Suggested branch:

```text
feature/intel-mac-orbslam3-monocular-tracker
```

## Intel Mac task IM-SLAM-04: Live ORB-SLAM3 diagnostics

Goal: verify the complete local camera and SLAM path with Pangolin before adding
network publishing.

Implemented behavior:

- load the validated camera calibration and start its matching macOS device;
- feed every captured frame to the monocular ORB-SLAM3 tracker;
- enable ORB-SLAM3's Pangolin viewer for local visual confirmation;
- reduce tracked points at 5 Hz instead of building an unbounded output queue;
- report tracking state, pose availability, point operations, filtering, and
  camera frame drops without logging image data;
- stop camera capture and ORB-SLAM3 worker threads cleanly on Ctrl-C.

Suggested branch:

```text
feature/intel-mac-live-orbslam3-diagnostics
```

## Local development order

Run and verify components from left to right:

1. Mock Sender plus packet dump;
2. Mock Sender plus Receiver plus packet dump;
3. Mock Sender plus Receiver plus Unity;
4. real SLAM adapter plus the established pipeline.

This order keeps failures attributable to one boundary at a time.

## Documentation authority

The Markdown files directly under `docs/` and examples directly under
`protocol/` are the current source of truth. The extracted
`docs/slam_remote_viewer_development_docs/` directory and its ZIP are retained
as planning input only; where they differ, the top-level documents take
precedence.
