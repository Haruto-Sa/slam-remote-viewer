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
