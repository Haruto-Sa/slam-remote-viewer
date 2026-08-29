# Unity Viewer

The Unity project receives validated Protocol v1 telemetry from the Receiver.
It expects converted `unity_world` messages on `tcp://127.0.0.1:5556` by
default.

## Requirements

- Unity `6000.2.4f1`
- the .NET Framework compatibility profile configured by this project

Open the `unity/` directory in Unity Hub. Unity Package Manager installs
NuGetForUnity and Newtonsoft.Json from `Packages/manifest.json`.
NuGetForUnity then restores NetMQ and its dependencies from
`Assets/packages.config` into the ignored `Assets/Packages/` directory.

## Subscriber component

Add `TelemetrySubscriberBehaviour` to a GameObject that remains enabled while
telemetry should be received. The component:

- connects a SUB socket to the configurable endpoint;
- subscribes to the `slam/v1/` topic prefix on a background thread;
- accepts exactly two UTF-8 frames containing a topic and JSON payload;
- validates immutable settings, pose, and point-cloud message objects;
- rejects telemetry until matching `unity_world` settings have arrived;
- places accepted messages in a bounded queue, dropping the oldest entry when
  the queue is full;
- invokes `MessageReceived` from `Update`, and therefore on Unity's main
  thread;
- cancels and joins the worker when the component is disabled.

The worker does not call Unity APIs. Consumers should update scenes only from
the `MessageReceived` callback. `AcceptedCount`, `RejectedCount`, and
`DroppedCount` expose ingress health for diagnostics.

## Tests

Run EditMode tests in the Unity Test Runner, or in batch mode on macOS:

```text
/Applications/Unity/Hub/Editor/6000.2.4f1/Unity.app/Contents/MacOS/Unity \
  -batchmode -nographics \
  -projectPath /absolute/path/to/slam-remote-viewer/unity \
  -runTests -testPlatform EditMode \
  -testResults /tmp/slam-remote-viewer-editmode-results.xml
```

Do not add `-quit`; the Unity Test Runner exits after writing its results.

## Camera pose and frustum scene setup

1. Open `Assets/Scenes/Viewer.unity`.
2. Create an empty `Telemetry` GameObject.
3. Add `TelemetrySubscriberBehaviour` and leave its endpoint at
   `tcp://127.0.0.1:5556`.
4. Add `CameraPoseVisualizer` to the same GameObject.
5. Enter Play Mode after starting the Receiver and Mock Sender.

`CameraPoseVisualizer` subscribes to the main-thread `MessageReceived` event.
When its scene-object fields are empty, it creates a cube marker and a
wireframe frustum automatically. Assign `Camera Pose Root` and `Frustum Line`
in the Inspector only when replacing those defaults with custom objects.

The visualization applies converted Protocol v1 `Twc` directly: `p` becomes
the marker's world position and `[x, y, z, w]` becomes its world rotation.
Settings establish the active session and camera aspect ratio. A new session
resets and hides the previous pose; poses received before matching Settings are
ignored. Marker and frustum colors indicate `tracking`, `initializing`, `lost`,
and `relocalizing` states.

## Trajectory history

Add `CameraTrajectoryVisualizer` to the same `Telemetry` GameObject as the
subscriber and camera-pose visualizer. Its scene references may remain empty;
the component creates a `Camera Trajectory` root and one LineRenderer per
continuous tracking interval.

Only `tracking` poses contribute points. `initializing`, `lost`, and
`relocalizing` break the current line so a later tracking pose starts a new
segment. A new Settings session clears all retained points and segments.
`Minimum Point Distance` controls spatial downsampling, while `Maximum Point
Count` bounds memory and removes the oldest positions first. Line material,
color, and width are configurable in the Inspector.

## Point-cloud deltas

Add `PointCloudVisualizer` to the `Telemetry` GameObject. Its Particle System
field may remain empty; the component creates one `Point Cloud` child and
renders every retained point through that single ParticleSystem.

Point IDs persist across delta messages. Operations are applied in Protocol v1
order: remove, update, then add. An unknown update adds the point, an existing
add updates it, and an unknown remove is ignored. A new Settings session clears
all point state and particles. Point material, color, size, and visibility are
configurable in the Inspector.
