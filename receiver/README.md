# SLAM Receiver

`slam-receiver` connects to a ZeroMQ PUB endpoint and receives Protocol v1
telemetry as a SUB client.

Each message must contain exactly two multipart frames:

1. a UTF-8 topic;
2. a UTF-8 JSON payload.

The Receiver currently validates the transport framing, UTF-8 encoding, and
JSON syntax. Protocol DTO parsing, coordinate conversion, and Unity
republishing are handled by later Issues.

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

## Validation behavior

A received message is rejected without terminating the process when:

- the multipart message does not contain exactly two frames;
- the topic is not valid UTF-8;
- the payload is not valid UTF-8;
- the payload is not syntactically valid JSON.

Rejected messages are written to standard error and counted in the shutdown
summary.

## Known limitations

- Payloads are not yet deserialized into Protocol v1 message types.
- Protocol fields and values are not yet semantically validated.
- Sequence gaps and session changes are not yet tracked.
- SLAM-to-Unity coordinate conversion is not yet implemented.
- Valid payloads are logged in full, which is temporary diagnostic behavior.
