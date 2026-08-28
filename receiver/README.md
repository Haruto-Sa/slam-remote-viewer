# SLAM Receiver

`slam-receiver` connects to a ZeroMQ PUB endpoint and receives Protocol v1
telemetry as a SUB client.

Each message must contain exactly two multipart frames:

1. a UTF-8 topic;
2. a UTF-8 JSON payload.

The Receiver validates transport framing, payload size, UTF-8 encoding, JSON
syntax, and Protocol v1 fields. Valid payloads are deserialized into typed
settings, pose, and point-cloud messages. Pose quaternions are normalized,
converted from the canonical SLAM frame into Unity coordinates, and then made
sign-continuous within each session. Unity republishing is handled by a later
Issue.

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
  --endpoint 'tcp://127.0.0.1:5555'
```

The default endpoint is `tcp://127.0.0.1:5555`, so `--endpoint` can be omitted.

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
- Complete session lifecycle and state clearing are not yet implemented.
- Point-cloud deltas are not yet applied to persistent state.
- Validated telemetry is not yet republished to Unity.
