# Telemetry Protocol v1

## Purpose

This protocol carries SLAM settings, camera poses, and point-cloud deltas through
the following pipeline.

```text
SLAM -> Sender -> ZeroMQ -> Receiver -> ZeroMQ -> Unity
```

The wire representation is deliberately independent of the SLAM implementation
and Unity types. All text is UTF-8 and all payloads are JSON.

## Transport

Telemetry uses ZeroMQ `PUB/SUB` with two multipart frames per message.

| Frame | Content |
|---|---|
| 0 | Topic encoded as UTF-8 |
| 1 | JSON payload encoded as UTF-8 |

Protocol v1 topics are:

- `slam/v1/settings`
- `slam/v1/pose`
- `slam/v1/pointcloud`

Default endpoints:

| Publisher | Bind endpoint | Subscriber |
|---|---|---|
| Sender | `tcp://*:5555` | Receiver |
| Receiver | `tcp://127.0.0.1:5556` | Unity |

Endpoints must be configurable. The Sender publishes coordinates in the SLAM
frame. The Receiver validates messages, converts them into Unity coordinates,
rewrites `settings.frame` to `unity_world`, and republishes the same topics
locally. This prevents a subscriber from interpreting converted coordinates as
raw SLAM coordinates.

Because PUB/SUB can lose messages while a subscriber is connecting, the Sender
must publish `settings` at startup and repeat it every 5 seconds. The Receiver
must do the same on its local publisher. Pose and point-cloud messages are not
retransmitted in v1.

## Common rules

- `v` is the protocol major version and is currently `1`.
- `session` is a non-empty identifier that changes whenever SLAM restarts or
  resets its map.
- `seq` is a per-topic unsigned, monotonically increasing integer. It starts at
  zero and may contain gaps because telemetry is lossy.
- `t` is the number of seconds since the start of the session. It must be finite
  and non-negative.
- Positions and point coordinates are measured in metres.
- Quaternions use `[x, y, z, w]` order and must be normalized by the Receiver.
- JSON numbers must be finite. `NaN` and infinity are invalid.
- Unknown JSON fields must be ignored for forward compatibility.
- A missing required field, wrong type, unsupported version, or invalid numeric
  value makes the complete message invalid.
- Receivers should reject payloads larger than 16 MiB.

## Settings

Topic: `slam/v1/settings`

Settings establish the interpretation of messages for one session.

```json
{
  "v": 1,
  "session": "001",
  "unit": "m",
  "frame": "slam_world",
  "pose_convention": "Twc",
  "quaternion": "xyzw",
  "camera": {
    "type": "pc",
    "id": "builtin_0",
    "width": 1280,
    "height": 720,
    "fps": 30
  },
  "pointcloud_mode": "delta"
}
```

Required fixed values for v1 are `unit: "m"`, `pose_convention: "Twc"`,
`quaternion: "xyzw"`, and `pointcloud_mode: "delta"`. The Sender emits
`frame: "slam_world"`; after coordinate conversion, the Receiver emits
`frame: "unity_world"`. `Twc` means the camera pose in the named world frame,
not the world pose in camera coordinates.

When the session changes, the Receiver and Unity must clear all pose,
trajectory, and point-cloud state before accepting telemetry for the new
session.

## Pose

Topic: `slam/v1/pose`

```json
{
  "v": 1,
  "session": "001",
  "seq": 1523,
  "t": 123.456789,
  "p": [1.2, 0.4, 2.1],
  "q": [0.0, 0.7071068, 0.0, 0.7071068],
  "state": "tracking"
}
```

- `p` is the camera position `[x, y, z]`.
- `q` is the camera orientation `[x, y, z, w]`.
- `state` is one of `initializing`, `tracking`, `lost`, or `relocalizing`.
- The Receiver forwards every valid state for status display, but Unity must
  extend the trajectory only while `state` is `tracking`.

## Point-cloud delta

Topic: `slam/v1/pointcloud`

```json
{
  "v": 1,
  "session": "001",
  "seq": 82,
  "t": 123.5,
  "add": [
    [1001, 0.1, 0.2, 1.4],
    [1002, 0.2, 0.3, 1.5]
  ],
  "update": [],
  "remove": []
}
```

`add` and `update` entries are `[id, x, y, z]`. `remove` contains point IDs.
IDs are non-negative integers no larger than `9007199254740991` (`2^53 - 1`).

Operations are applied in this order:

1. remove
2. update
3. add

For idempotent recovery from duplicated messages, updating an unknown ID acts
as add, adding an existing ID acts as update, and removing an unknown ID is a
no-op.

## Subscriber behavior

The Receiver and Unity keep the last sequence number for each telemetry topic
and session (`pose` and `pointcloud`; `settings` has no sequence number). A
message whose `seq` is less than or equal to the last accepted value is ignored.
A gap is recorded as a warning but is not fatal.

Telemetry received before valid settings for its session is ignored. Invalid
messages must not terminate the process; they are counted and logged with the
topic and reason, without logging an entire large payload.

Version 1 does not guarantee delivery or provide a point-cloud snapshot. If
delta loss proves significant in testing, a snapshot/control socket will be
introduced as a later protocol extension rather than changing v1 semantics.
