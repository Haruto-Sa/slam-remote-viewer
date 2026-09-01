# SLAM Mock Sender

`slam-mock-sender` publishes deterministic Protocol v1 telemetry over a
ZeroMQ PUB socket. It replaces the camera and SLAM backend while developing
the Receiver and Unity viewer.

## Published telemetry

Each message consists of two ZeroMQ multipart frames: a UTF-8 topic followed
by a UTF-8 JSON payload.

| Topic | Behavior |
|---|---|
| `slam/v1/settings` | Published at startup and every 5 seconds |
| `slam/v1/pose` | Circular camera trajectory at the configured rate |
| `slam/v1/pointcloud` | Fixed point-cloud delta at startup and every 5 seconds |

The Sender publishes `Twc` in `slam_world`, metres, with quaternions in
`[x, y, z, w]` order. It does not perform the Unity coordinate conversion.
See [`../../docs/protocol.md`](../../docs/protocol.md) and
[`../../docs/coordinate-system.md`](../../docs/coordinate-system.md).

## SLAM pose source interface

The network layer consumes the backend-independent `PoseSource` interface.
Each `SlamPose` contains a frame ID, session-relative timestamp, translation,
`[x, y, z, w]` orientation, and tracking state. The Protocol v1 adapter maps:

- source frame ID to pose `seq`;
- timestamp in seconds to `t`;
- translation to `p`;
- orientation to `q`;
- the backend-independent tracking state to the wire `state` value.

A source must provide camera `Twc` in the canonical right-handed `slam_world`
frame: +X right, +Y down, +Z forward, in metres. A backend exposing another
frame or `Tcw` must adapt it before constructing `SlamPose`. The Mock Sender
uses `MockPoseSource` through the same interface intended for a future real
SLAM backend.

The versioned local process contract for a C++ producer is defined in
[`../../docs/slam-streamer-boundary.md`](../../docs/slam-streamer-boundary.md).
It deliberately stops before Protocol v1 serialization and network publishing.

## Requirements

- Rust stable toolchain
- A C/C++ compiler for the bundled libzmq source build

The pinned `zmq` 0.10.0 dependency uses `zmq-sys` 0.12.0, which builds its
pinned libzmq source through `zeromq-src`. The Sender therefore does not need a
repository-specific `PKG_CONFIG_PATH`, a Homebrew ZeroMQ dylib, or a patched
`rust-zmq` checkout. The first Cargo build can take longer while that native
dependency is compiled.

### Apple Silicon verification

Use an arm64 shell and Rust toolchain throughout the build. Homebrew tools, when
needed for other SLAM dependencies, must come from `/opt/homebrew`; do not add a
repository-local absolute-path override to compensate for an Intel/Rosetta
shell.

Run a clean locked build from the repository root:

```bash
test "$(uname -m)" = arm64
test "$(rustc -vV | sed -n 's/^host: //p')" = aarch64-apple-darwin
cargo clean --manifest-path sender/streamer/Cargo.toml
cargo build --locked --manifest-path sender/streamer/Cargo.toml
cargo test --locked --manifest-path sender/streamer/Cargo.toml
file sender/streamer/target/debug/slam-mock-sender
otool -L sender/streamer/target/debug/slam-mock-sender
```

`file` must report a Mach-O `arm64` executable. With the currently pinned Rust
dependencies, `otool -L` must not report a Homebrew `libzmq` dylib; libzmq is
linked from the native source build instead. An `/usr/local` or x86_64 library
in the output is an architecture error, not a condition to hide with a fallback
path.

## Test

Run from the repository root:

```bash
cargo fmt --manifest-path sender/streamer/Cargo.toml --check
cargo check --manifest-path sender/streamer/Cargo.toml --examples
cargo test --locked --manifest-path sender/streamer/Cargo.toml
```

## Run continuously

```bash
cargo run \
  --manifest-path sender/streamer/Cargo.toml \
  -- \
  --endpoint 'tcp://127.0.0.1:5555'
```

Press Ctrl-C to stop cleanly.

## Run a deterministic finite session

```bash
cargo run \
  --manifest-path sender/streamer/Cargo.toml \
  -- \
  --endpoint 'tcp://127.0.0.1:5555' \
  --session integration-test \
  --pose-rate-hz 10 \
  --radius-m 3 \
  --angular-speed-rad-per-sec 0.25 \
  --duration-sec 2
```

This example emits 20 poses with sequence numbers `0..19`. Pose timestamps
are derived from `seq / pose-rate-hz`, rather than scheduler timing. The
generator uses no random values, so a seed is unnecessary and identical
arguments produce identical payloads.

## CLI options

| Option | Default | Constraint |
|---|---:|---|
| `--endpoint` | `tcp://*:5555` | Valid ZeroMQ bind endpoint |
| `--session` | `mock-session-001` | Non-empty string |
| `--pose-rate-hz` | `30` | Finite and greater than zero |
| `--radius-m` | `2` | Finite and greater than zero |
| `--angular-speed-rad-per-sec` | `0.5` | Finite and non-negative |
| `--duration-sec` | unlimited | Finite and greater than zero |

Run `cargo run --manifest-path sender/streamer/Cargo.toml -- --help` for the
generated command reference.

## Verify all topics

Start the diagnostic Subscriber first:

```bash
cargo run \
  --manifest-path sender/streamer/Cargo.toml \
  --example packet_dump \
  -- 'tcp://127.0.0.1:5555'
```

Start the Mock Sender in another terminal. `packet_dump` exits successfully
after receiving settings, pose, and point-cloud messages. It reports a timeout
with a hint if no telemetry arrives within 30 seconds.

## Known limitations

- PUB/SUB is lossy and v1 has no replay or point-cloud snapshot.
- The point cloud is a fixed two-point fixture.
- Settings and point-cloud messages are repeated so late subscribers can
  recover the current mock state.
