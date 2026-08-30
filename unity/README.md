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
the `MessageReceived` callback. The component exposes its endpoint, running
state, queue depth, accepted/rejected/dropped counts, and latest rejection or
subscriber fault for diagnostics.

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

## Telemetry diagnostics overlay

The default `Viewer` scene includes `TelemetryDiagnosticsOverlay` on the
`Telemetry` GameObject. When its source fields are empty, it discovers the
subscriber and visualizers on the same GameObject. Missing trajectory or
point-cloud visualizers are displayed as `N/A` and do not stop the overlay.

The panel shows:

- subscriber endpoint and current session;
- latest pose tracking state;
- age of the latest accepted main-thread message;
- accepted, rejected, and dropped message counts;
- current main-thread queue depth;
- retained trajectory and point-cloud point counts;
- subscriber fault count, latest fault, and latest rejection reason.

Health statuses mean:

- `Waiting`: the subscriber is running but no accepted Settings or telemetry
  has reached the main thread;
- `Receiving`: an accepted message arrived within `Stale Timeout Seconds`;
- `Stale`: the subscriber remains running but no accepted message arrived
  within the timeout;
- `Stopped`: the subscriber thread is not running or the component is disabled.

`Show Overlay` controls only drawing; telemetry subscription, visualization,
and diagnostic state updates continue when it is off. Increase `Panel Rect`
or change `Font Size` if longer endpoint or error text is clipped. The Viewer
scene defaults to a `520 x 360` panel with an `18`-pixel font for readable
display on Retina-class development screens.

For a quick check, enter Play Mode without a Receiver and observe `Waiting`.
Then start the Receiver plus Mock Sender, or the recorded-session player, and
observe `Receiving`, session/tracking values, and increasing counters. Stop the
producer and wait for the configured timeout to verify `Stale`.

## Viewer camera controls

The default `Viewer` scene includes `OrbitCameraController` on `Main Camera`.
In Play Mode:

- hold the right mouse button and drag to orbit around the focus point;
- hold the middle mouse button and drag to pan the camera and focus point;
- use the mouse wheel to zoom;
- press `1` for the front view from negative Z;
- press `2` for the right view from positive X;
- press `3` for the back view from positive Z;
- press `4` for the left view from negative X;
- press `F` to center and fit all visible retained telemetry;
- press `R` to restore the initial camera pose and focus point.

The focus point, distance and pitch limits, orbit and pan speeds, zoom amount,
reset key, and four preset keys are configurable in the Inspector. A side-view
preset preserves the current focus and distance while setting pitch to zero.
Orbit, pan, and zoom remain available immediately afterward. Motion uses
unscaled time, so camera navigation continues when `Time.timeScale` is zero.
Pointer input over the visible telemetry diagnostics panel is ignored; keyboard
presets and reset remain available over the panel.

Frame-all uses the retained camera pose, trajectory, and point-cloud bounds for
layers that are currently visible. It excludes hidden layers and the world grid.
The current viewing direction is preserved while focus and distance change.
`Frame All Key` and `Framing Padding` are configurable on
`OrbitCameraController`; the result remains within its minimum and maximum
distance limits. Pressing `F` before any visible telemetry exists has no effect.

## World grid and axes

The default `Viewer` scene includes `WorldReferenceVisualizer` on the
world-origin `World Reference` GameObject. It draws an XZ ground grid plus the
positive Unity axes: X is red and points right, Y is green and points up, and Z
is blue and points forward.

`Show Reference` hides or shows the complete reference without affecting
telemetry. `Grid Spacing`, `Grid Extent`, `Line Width`, `Grid Color`, `Axis
Length`, and the three axis colors are configurable in the Inspector. Grid
geometry uses complete spacing intervals within the configured extent and is
batched into one mesh; together with the three axis lines, the renderer count
remains fixed at four. The reference does not subscribe to telemetry, so
session changes do not rebuild or clear it.

## Visualization visibility shortcuts

The default `Viewer` scene includes `VisualizationVisibilityController` on the
`Telemetry` GameObject. In Play Mode:

- press `P` to toggle the tracked camera marker and frustum;
- press `T` to toggle trajectory lines;
- press `C` to toggle the point cloud;
- press `G` to toggle the world grid and axes;
- press `D` to toggle the diagnostics overlay;
- press `V` to restore the configured default visibility values.

Shortcut keys and default visibility values are configurable on the controller
in the Inspector. Hiding a layer changes only its renderers: telemetry remains
subscribed, retained state keeps updating, and showing the layer again displays
the latest state without recreating its geometry. Missing optional visualizers
are ignored safely.
