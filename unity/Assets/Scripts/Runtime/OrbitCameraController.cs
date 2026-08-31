using System;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class OrbitCameraController : MonoBehaviour
    {
        [Header("Focus and limits")]
        [SerializeField]
        private Vector3 focusPoint = new Vector3(0f, 1f, 0f);

        [SerializeField, Min(0.01f)]
        private float minimumDistance = 0.5f;

        [SerializeField, Min(0.02f)]
        private float maximumDistance = 100f;

        [SerializeField, Range(-89f, 0f)]
        private float minimumPitchDegrees = -85f;

        [SerializeField, Range(0f, 89f)]
        private float maximumPitchDegrees = 85f;

        [Header("Input")]
        [SerializeField, Min(0f)]
        private float orbitSpeedDegreesPerSecond = 180f;

        [SerializeField, Min(0f)]
        private float panSpeedUnitsPerSecond = 5f;

        [SerializeField, Min(0f)]
        private float zoomUnitsPerScroll = 1.5f;

        [SerializeField]
        private KeyCode resetKey = KeyCode.R;

        [Header("View presets")]
        [SerializeField]
        private KeyCode frontViewKey = KeyCode.Alpha1;

        [SerializeField]
        private KeyCode rightViewKey = KeyCode.Alpha2;

        [SerializeField]
        private KeyCode backViewKey = KeyCode.Alpha3;

        [SerializeField]
        private KeyCode leftViewKey = KeyCode.Alpha4;

        [Header("Frame visible telemetry")]
        [SerializeField]
        private Camera viewerCamera;

        [SerializeField]
        private CameraPoseVisualizer cameraPoseVisualizer;

        [SerializeField]
        private CameraTrajectoryVisualizer trajectoryVisualizer;

        [SerializeField]
        private PointCloudVisualizer pointCloudVisualizer;

        [SerializeField]
        private KeyCode frameAllKey = KeyCode.F;

        [SerializeField, Min(1f)]
        private float framingPadding = 1.1f;

        [Header("Input blocking")]
        [SerializeField]
        private TelemetryDiagnosticsOverlay diagnosticsOverlay;

        [SerializeField]
        private ViewerControlsOverlay controlsOverlay;

        private OrbitCameraState state;

        public OrbitCameraState State => state;
        public KeyCode ResetKey => resetKey;
        public KeyCode FrontViewKey => frontViewKey;
        public KeyCode RightViewKey => rightViewKey;
        public KeyCode BackViewKey => backViewKey;
        public KeyCode LeftViewKey => leftViewKey;
        public KeyCode FrameAllKey => frameAllKey;

        private void Awake()
        {
            Initialize();
        }

        private void Update()
        {
            OrbitCameraCommand command = ReadCommand();
            ApplyCommand(command, Time.unscaledDeltaTime);
        }

        private void OnValidate()
        {
            minimumDistance = Mathf.Max(0.01f, minimumDistance);
            maximumDistance = Mathf.Max(minimumDistance + 0.01f, maximumDistance);
            minimumPitchDegrees = Mathf.Clamp(minimumPitchDegrees, -89f, -0.01f);
            maximumPitchDegrees = Mathf.Clamp(maximumPitchDegrees, 0.01f, 89f);
            orbitSpeedDegreesPerSecond = Mathf.Max(0f, orbitSpeedDegreesPerSecond);
            panSpeedUnitsPerSecond = Mathf.Max(0f, panSpeedUnitsPerSecond);
            zoomUnitsPerScroll = Mathf.Max(0f, zoomUnitsPerScroll);
            framingPadding = float.IsNaN(framingPadding) || float.IsInfinity(framingPadding)
                ? 1.1f
                : Mathf.Max(1f, framingPadding);
        }

        public bool ApplyCommand(OrbitCameraCommand command, float unscaledDeltaTime)
        {
            if (!enabled)
            {
                return false;
            }
            if (float.IsNaN(unscaledDeltaTime) || float.IsInfinity(unscaledDeltaTime) ||
                unscaledDeltaTime < 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(unscaledDeltaTime),
                    "unscaled delta time must be finite and non-negative");
            }
            if (state == null)
            {
                Initialize();
            }

            bool changed = state.ApplyCommand(
                command,
                orbitSpeedDegreesPerSecond * unscaledDeltaTime,
                panSpeedUnitsPerSecond * unscaledDeltaTime,
                zoomUnitsPerScroll);
            if (changed)
            {
                ApplyPose();
            }
            return changed;
        }

        public bool FrameVisibleTelemetry()
        {
            if (!TryBuildFrameRequest(out OrbitCameraFrameRequest request))
            {
                return false;
            }

            return ApplyCommand(
                new OrbitCameraCommand(
                    Vector2.zero,
                    Vector2.zero,
                    0f,
                    frameRequest: request),
                0f);
        }

        private void Initialize()
        {
            OnValidate();
            ResolveFrameSources();
            state = new OrbitCameraState(
                transform.position,
                focusPoint,
                minimumDistance,
                maximumDistance,
                minimumPitchDegrees,
                maximumPitchDegrees);
            ApplyPose();
        }

        private OrbitCameraCommand ReadCommand()
        {
            bool blocked =
                diagnosticsOverlay != null && diagnosticsOverlay.ContainsScreenPoint(
                    Input.mousePosition,
                    Screen.height) ||
                controlsOverlay != null && controlsOverlay.ContainsScreenPoint(
                    Input.mousePosition,
                    Screen.height);
            Vector2 orbit = Input.GetMouseButton(1)
                ? new Vector2(Input.GetAxisRaw("Mouse X"), Input.GetAxisRaw("Mouse Y"))
                : Vector2.zero;
            Vector2 pan = Input.GetMouseButton(2)
                ? new Vector2(Input.GetAxisRaw("Mouse X"), Input.GetAxisRaw("Mouse Y"))
                : Vector2.zero;
            OrbitCameraViewPreset? viewPreset = ReadViewPreset();
            OrbitCameraFrameRequest? frameRequest = null;
            if (Input.GetKeyDown(frameAllKey) &&
                TryBuildFrameRequest(out OrbitCameraFrameRequest request))
            {
                frameRequest = request;
            }

            return new OrbitCameraCommand(
                orbit,
                pan,
                Input.mouseScrollDelta.y,
                Input.GetKeyDown(resetKey),
                blocked,
                viewPreset,
                frameRequest);
        }

        private OrbitCameraViewPreset? ReadViewPreset()
        {
            if (Input.GetKeyDown(frontViewKey))
            {
                return OrbitCameraViewPreset.Front;
            }
            if (Input.GetKeyDown(rightViewKey))
            {
                return OrbitCameraViewPreset.Right;
            }
            if (Input.GetKeyDown(backViewKey))
            {
                return OrbitCameraViewPreset.Back;
            }
            if (Input.GetKeyDown(leftViewKey))
            {
                return OrbitCameraViewPreset.Left;
            }
            return null;
        }

        private void ResolveFrameSources()
        {
            if (viewerCamera == null)
            {
                viewerCamera = GetComponent<Camera>();
            }
            if (cameraPoseVisualizer == null)
            {
                cameraPoseVisualizer = GetComponent<CameraPoseVisualizer>();
            }
            if (trajectoryVisualizer == null)
            {
                trajectoryVisualizer = GetComponent<CameraTrajectoryVisualizer>();
            }
            if (pointCloudVisualizer == null)
            {
                pointCloudVisualizer = GetComponent<PointCloudVisualizer>();
            }
        }

        private bool TryBuildFrameRequest(out OrbitCameraFrameRequest request)
        {
            ResolveFrameSources();
            var accumulator = new TelemetryBoundsAccumulator();
            AddVisibleBounds(cameraPoseVisualizer, accumulator);
            AddVisibleBounds(trajectoryVisualizer, accumulator);
            AddVisibleBounds(pointCloudVisualizer, accumulator);

            if (viewerCamera == null || !accumulator.TryGetBounds(out Bounds bounds))
            {
                request = default;
                return false;
            }

            request = new OrbitCameraFrameRequest(
                bounds,
                viewerCamera.fieldOfView,
                viewerCamera.aspect,
                framingPadding,
                viewerCamera.nearClipPlane);
            return true;
        }

        private static void AddVisibleBounds(
            CameraPoseVisualizer visualizer,
            TelemetryBoundsAccumulator accumulator)
        {
            if (visualizer != null && visualizer.IsVisible &&
                visualizer.TryGetWorldBounds(out Bounds bounds))
            {
                accumulator.Add(bounds);
            }
        }

        private static void AddVisibleBounds(
            CameraTrajectoryVisualizer visualizer,
            TelemetryBoundsAccumulator accumulator)
        {
            if (visualizer != null && visualizer.IsVisible &&
                visualizer.TryGetWorldBounds(out Bounds bounds))
            {
                accumulator.Add(bounds);
            }
        }

        private static void AddVisibleBounds(
            PointCloudVisualizer visualizer,
            TelemetryBoundsAccumulator accumulator)
        {
            if (visualizer != null && visualizer.IsVisible &&
                visualizer.TryGetWorldBounds(out Bounds bounds))
            {
                accumulator.Add(bounds);
            }
        }

        private void ApplyPose()
        {
            transform.SetPositionAndRotation(state.Position, state.Rotation);
        }
    }
}
