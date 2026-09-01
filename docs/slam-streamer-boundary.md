# Live SLAM-to-streamer boundary

This document defines boundary version 1 between a local C++ SLAM producer and
the Rust Sender. It is an internal process contract, not telemetry Protocol v1.
The C++ process publishes backend-independent tracking results; Rust alone owns
Protocol v1 topics, per-topic sequence numbers, and telemetry JSON.

## Transport and ownership

The producer and consumer communicate through a Unix domain `SOCK_STREAM`
socket on the same host. The socket path is runtime configuration and must not
be committed as an absolute machine-specific path. Rust owns the listening
socket and accepts one C++ producer connection at a time.

Each message has this framing:

```text
4-byte unsigned big-endian payload length | UTF-8 JSON payload
```

The maximum JSON payload is 1 MiB. A consumer rejects a larger declared length
before allocating its payload buffer. EOF before any length byte is a clean
disconnect; EOF within a length or payload is a truncated frame. A partial
frame is discarded on disconnect.

There is no application queue in this contract. The producer writes directly
to the bounded operating-system socket buffer. Later production adapters must
drop or coalesce replaceable point-cloud updates rather than create an
unbounded queue when the consumer is absent or slow.

## Coordinate and numeric contract

- timestamps are finite, non-negative seconds since the current SLAM session
  started;
- frame IDs and point IDs are unsigned integers no greater than
  `9007199254740991`, so they round-trip through JSON safely;
- a pose is camera `Twc` in the canonical right-handed `slam_world` frame;
- translation is `[x, y, z]` in metres;
- orientation is a finite unit quaternion `[x, y, z, w]`;
- `tracking` requires a pose; initializing, lost, and relocalizing may carry a
  pose when the backend has a meaningful last estimate;
- point positions use the same `slam_world` frame and metre unit;
- image bytes, image paths, and ORB-SLAM3 types are forbidden.

## Message lifecycle

`hello` must be the first message on every connection. It selects the exact
boundary major version and establishes the session and camera metadata. The
consumer rejects unknown message types, an unsupported version, empty identity
fields, or non-positive camera dimensions and FPS.

After `hello`, the producer may interleave `tracking_frame` and
`pointcloud_delta` messages. Frame ID and timestamp ordering are monotonic
within each message type; the point-cloud stream may run more slowly than the
tracking stream. A point ID may occur at most once across add, update, and
remove in one delta.

`session_end` ends the connection's session. No later message is accepted.
Disconnect without `session_end` is an abnormal but recoverable end: the
consumer discards partial input and reports the disconnect. Every reconnect
starts with a new `hello`. A different session ID resets ordering and retained
point-cloud state; reconnecting the same session is rejected by the production
adapter unless a future boundary version defines resume semantics.

## Schemas

### Hello

```json
{
  "type": "hello",
  "boundary_version": 1,
  "session_id": "slam-session-001",
  "producer": "orbslam3-monocular",
  "camera": {
    "camera_type": "monocular",
    "id": "camera-id",
    "width": 1280,
    "height": 720,
    "fps": 30
  }
}
```

### Tracking frame

```json
{
  "type": "tracking_frame",
  "boundary_version": 1,
  "session_id": "slam-session-001",
  "frame_id": 42,
  "timestamp_seconds": 1.4,
  "tracking_state": "tracking",
  "pose": {
    "translation": [1.0, 2.0, 3.0],
    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]
  }
}
```

Tracking state is one of `initializing`, `tracking`, `lost`, or `relocalizing`.
`pose` may be `null` under the state rules above.

### Point-cloud delta

```json
{
  "type": "pointcloud_delta",
  "boundary_version": 1,
  "session_id": "slam-session-001",
  "frame_id": 42,
  "timestamp_seconds": 1.4,
  "add": [{ "id": 1001, "position": [0.1, 0.2, 1.4] }],
  "update": [{ "id": 1002, "position": [0.2, 0.3, 1.5] }],
  "remove": [1003]
}
```

### Session end

```json
{
  "type": "session_end",
  "boundary_version": 1,
  "session_id": "slam-session-001",
  "reason": "shutdown"
}
```

## Validation and compatibility

Boundary version 1 fails closed. A consumer rejects incompatible versions,
unknown message types or fields, malformed JSON, invalid UTF-8, non-finite
numbers, invalid quaternions, unsafe IDs, duplicate point operations, session
mismatch, ordering regression, and messages outside the lifecycle above. It
must log the category and connection context without logging image data.

Adding optional semantics still requires a new boundary version because the
version 1 Rust schema rejects unknown fields. Producers and consumers must be
upgraded together; there is no legacy or best-effort fallback.

## Hardware-independent verification

Contract fixtures live under
`sender/streamer/tests/fixtures/slam_boundary/`. Rust tests decode the framing,
validate a complete session, and reject version, lifecycle, ordering, numeric,
and point-operation violations without starting a camera, ORB-SLAM3, a Unix
socket, or ZeroMQ.
