# Architecture

## Runtime pipeline

```text
Camera -> SLAM -> Rust Sender -> ZeroMQ/LAN -> Rust Receiver
                                                |
                                                v
                                      ZeroMQ/localhost -> Unity
```

The SLAM implementation, transport, and viewer are separated by the Protocol
v1 contract. During viewer development, a Mock Sender replaces Camera and SLAM.

## Responsibilities

### SLAM adapter

- obtains frames from the selected camera source;
- converts library-specific output to canonical `Twc`;
- supplies poses and map-point changes to the Sender.

### Rust Sender

- assigns session and per-topic sequence numbers;
- serializes Protocol v1 JSON;
- publishes settings, poses, and point-cloud deltas;
- contains no Unity-specific coordinate conversion.

### Rust Receiver

- subscribes to the Sender;
- parses and validates messages;
- rejects stale or incompatible telemetry;
- converts SLAM coordinates to Unity coordinates;
- changes the local settings frame from `slam_world` to `unity_world`;
- normalizes quaternions and preserves sign continuity;
- republishes validated telemetry on localhost.

### Unity Viewer

- receives telemetry on a background thread;
- passes immutable messages to the main thread through a bounded queue;
- maintains camera, trajectory, and point-cloud state;
- renders state and records supported outputs;
- never calls the Unity API from the network thread.

## Repository layout

```text
docs/                 architecture and development decisions
protocol/             canonical example messages
sender/slam/           SLAM adapter
sender/streamer/       Rust Sender and Mock Sender
receiver/              Rust Receiver
unity/Assets/Network/  ZeroMQ transport and message DTOs
unity/Assets/Model/    runtime state independent of rendering
unity/Assets/Visualization/ Unity rendering components
unity/Assets/Scripts/  composition and application lifecycle
tools/                 diagnostics and packet inspection
```

## Dependency direction

Visualization may depend on Model, and application Scripts may compose all
Unity layers. Model must not depend on Visualization or Unity networking.
Network code parses data but does not mutate scene objects directly.

## Initial technology choices

- Rust: `serde`, `serde_json`, and ZeroMQ bindings
- Unity/C#: NetMQ for a managed ZeroMQ client
- Serialization: JSON for inspectability during the MVP
- Testing: Rust unit/integration tests plus Unity EditMode tests

Binary serialization, reliable snapshots, and remote control are deliberately
deferred until measurements show they are required.
