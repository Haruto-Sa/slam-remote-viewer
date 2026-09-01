# SLAM Receiver

`slam-receiver` connects to a ZeroMQ PUB endpoint and receives Protocol v1
telemetry as a SUB client. It validates and converts accepted telemetry, then
republishes it from a local PUB endpoint for Unity. It also binds a local REP
endpoint through which Unity marks standalone clip boundaries.

Each message must contain exactly two multipart frames:

1. a UTF-8 topic;
2. a UTF-8 JSON payload.

The Receiver validates transport framing, payload size, UTF-8 encoding, JSON
syntax, and Protocol v1 fields. Valid payloads are deserialized into typed
settings, pose, and point-cloud messages. Pose quaternions are normalized,
converted from the canonical SLAM frame into Unity coordinates, and then made
sign-continuous within each session. Accepted telemetry is republished for
Unity and can optionally be recorded for offline inspection.

See [`../docs/protocol.md`](../docs/protocol.md) for the Protocol v1 contract.

## Subscribed topics

The Subscriber registers the `slam/v1/` prefix and receives:

- `slam/v1/settings`
- `slam/v1/pose`
- `slam/v1/pointcloud`

## Requirements

- Rust stable toolchain
- A C/C++ compiler for the bundled libzmq source build

The first Cargo build can take longer because `zmq-sys` may build libzmq from
source.

## Test

Run from the repository root:

```bash
cargo fmt --manifest-path receiver/Cargo.toml --check
cargo check --manifest-path receiver/Cargo.toml
cargo test --manifest-path receiver/Cargo.toml
cargo clippy --manifest-path receiver/Cargo.toml -- -D warnings
```

## Run

```bash
cargo run \
  --manifest-path receiver/Cargo.toml \
  -- \
  --endpoint 'tcp://127.0.0.1:5555' \
  --output-endpoint 'tcp://127.0.0.1:5556' \
  --control-endpoint 'tcp://127.0.0.1:5557' \
  --record-dir recordings
```

All shown values are defaults and can be omitted. The input endpoint connects
to the Sender, the output endpoint publishes telemetry to Unity, and the
control endpoint accepts Unity clip commands. Full-session recording is always
enabled and defaults to `recordings`.

Press Ctrl-C to stop cleanly.

## Verify with the Mock Sender

Start the Receiver first. In another terminal, run:

```bash
cargo run \
  --manifest-path sender/streamer/Cargo.toml \
  -- \
  --endpoint 'tcp://127.0.0.1:5555' \
  --session receiver-test \
  --pose-rate-hz 2 \
  --duration-sec 7
```

The Receiver should report all three Protocol v1 topics. Settings and
point-cloud messages are repeated by the Mock Sender so a late Subscriber can
recover after the initial PUB/SUB connection delay.

Valid messages are logged without printing the complete payload:

```text
received topic=slam/v1/settings session=receiver-test seq=-
received topic=slam/v1/pose session=receiver-test seq=0
received topic=slam/v1/pointcloud session=receiver-test seq=0
```

Accepted messages are serialized and republished with the same topics as two
multipart frames. The latest converted Settings message is repeated every five
seconds so a late Unity subscriber can establish the session contract. Send
failures are logged using topic, session, sequence metadata, and a reason; the
payload is never logged. Publication counts appear in the shutdown summary.

## Session recording and PLY export

When `--record-dir` is specified, the Receiver sends accepted, converted
telemetry to a dedicated recording thread. Network reception and Unity
publication do not wait for file writes. Pose and PointCloud messages are
rejected until Settings establishes the active session, and telemetry for a
different session is rejected.

Each session creates a directory beneath the configured root. Unsafe filename
characters are replaced, and an incrementing suffix prevents an existing
recording from being overwritten:

```text
recordings/
└── receiver-test/
    ├── telemetry.ndjson
    ├── pointcloud.ply
    └── metadata.json
```

- `telemetry.ndjson` contains one `{ "topic", "payload" }` object per accepted
  input message in arrival order.
- `pointcloud.ply` is an ASCII PLY containing final retained positions in
  `unity_world` coordinates. Positions are emitted in ascending point-ID order;
  IDs are used for state management but are not exported as a PLY property.
- `metadata.json` records the protocol version, session, coordinate frame and
  unit, message counts, final point count, and output filenames.

Point-cloud operations are applied in Protocol v1 order: remove, update, then
add. Unknown updates add a point, existing adds replace its position, and
unknown removals are no-ops. A new Settings session finalizes the previous
recording and starts with empty point state. Ctrl-C drains the recording queue
and finalizes the active session before the Receiver exits.

PLY and metadata files are written through temporary files and renamed only
after a successful flush. File errors include the failed operation and path;
the Receiver exits unsuccessfully when requested recording cannot be
finalized.

## Unity-controlled telemetry clips

The Receiver continuously records the complete accepted session independently
of clip capture. Unity sends `start_clip`, `stop_clip`, and `status` JSON
commands to the local control endpoint. Commands are queued to a dedicated clip
worker so clip finalization does not block telemetry reception or publication.

`Start Clip` snapshots the active Settings, latest camera Pose, and complete
retained point cloud. Subsequent Pose and PointCloud messages are retained until
`Stop Clip`. The resulting standalone directory is written beneath
`recordings/clips/`:

```text
recordings/
├── receiver-test/
│   └── ... full-session files ...
└── clips/
    └── receiver-test-clip-001/
        ├── telemetry.ndjson
        ├── pointcloud.ply
        └── metadata.json
```

Clip metadata includes the source session, inclusive start and exclusive end
message indices, source telemetry start/end times, interval message count, and
the standard recording statistics. Additional metadata fields are ignored by
the existing player, so the clip directory can be passed directly to
`slam-session-player`. Starting before Settings and a Pose are available fails
without stopping the Receiver. Overlapping clips are rejected; a completed or
failed clip can be followed by another clip in the same session.

## Replay a recorded session into Unity

Start the Unity Viewer and enter Play Mode so its subscriber is connected.
Do not run `slam-receiver` at the same time: the Receiver and session player
both bind the default Unity endpoint.

In another terminal, run:

```bash
cargo run \
  --manifest-path receiver/Cargo.toml \
  --bin slam-session-player \
  -- \
  --session-dir recordings/receiver-test \
  --speed 1.0
```

The player binds `tcp://127.0.0.1:5556`, waits 500 milliseconds for Unity's
subscription handshake, publishes Settings first, and then reproduces Pose and
PointCloud timing. `--speed 2.0` plays twice as fast; `--speed 0.5` plays at
half speed. Use `--endpoint` and `--startup-delay-ms` to override the endpoint
or initial wait.

Before opening the socket, the player validates:

- the metadata protocol version, session, unit, frame, and safe filenames;
- every NDJSON object, supported topic, payload shape, session, and fixed
  Protocol v1 values;
- Settings appears first;
- message, Pose, and PointCloud counts match metadata;
- replaying all point-cloud deltas produces the metadata final point count.

Payload JSON is published exactly as stored, without coordinate conversion or
reserialization. Recorded order is never changed. If timestamps decrease, the
later message is sent immediately after the preceding scheduled message.
Malformed telemetry errors identify `telemetry.ndjson` and its line number.
Ctrl-C interrupts both the startup wait and timed playback cleanly.

## Validation behavior

A received message is rejected without terminating the process when:

- the multipart message does not contain exactly two frames;
- the topic is not valid UTF-8;
- the payload is not valid UTF-8;
- the payload exceeds 16 MiB;
- the topic is not a supported Protocol v1 topic;
- the payload is not syntactically valid JSON;
- a required field is missing or has the wrong type;
- the protocol version, session, or fixed settings values are invalid;
- a timestamp, position, quaternion, or point coordinate is invalid;
- a quaternion has zero or near-zero norm;
- a point ID exceeds the JSON safe-integer limit.

Unknown JSON fields are ignored for forward compatibility.

Rejected messages are written to standard error and counted in the shutdown
summary. The complete payload is not included in validation-error logs.

## Quaternion processing

Pose quaternions retain Protocol v1 `[x, y, z, w]` component order. The
Receiver normalizes each quaternion with an overflow-resistant calculation and
rejects norms below `1e-12`.

Because `q` and `-q` represent the same rotation, after coordinate conversion
the Receiver compares each normalized quaternion with the previous accepted
pose in the same session. If their dot product is negative, every component of
the current quaternion is negated. The previous-quaternion reference is reset
when the session changes.

## Coordinate conversion

After validation and quaternion normalization, the Receiver applies the
coordinate contract from
[`../docs/coordinate-system.md`](../docs/coordinate-system.md):

- pose positions `[x, y, z]` become `[x, -y, z]`;
- pose quaternions `[x, y, z, w]` become `[-x, y, -z, w]`;
- point-cloud `add` and `update` coordinates use the same position conversion;
- point IDs, `remove` entries, and telemetry metadata remain unchanged;
- valid settings change from `frame: "slam_world"` to
  `frame: "unity_world"` after input validation.

## Known limitations

- Sequence gaps and duplicates are not yet tracked.
- Session lifecycle is enforced for recording and Unity republishing, but no
  explicit end-of-session Protocol v1 message exists.
