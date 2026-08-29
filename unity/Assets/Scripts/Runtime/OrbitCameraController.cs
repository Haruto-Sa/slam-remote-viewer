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

        [Header("Input blocking")]
        [SerializeField]
        private TelemetryDiagnosticsOverlay diagnosticsOverlay;

        private OrbitCameraState state;

        public OrbitCameraState State => state;

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

        private void Initialize()
        {
            OnValidate();
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
            bool blocked = diagnosticsOverlay != null && diagnosticsOverlay.ContainsScreenPoint(
                Input.mousePosition,
                Screen.height);
            Vector2 orbit = Input.GetMouseButton(1)
                ? new Vector2(Input.GetAxisRaw("Mouse X"), Input.GetAxisRaw("Mouse Y"))
                : Vector2.zero;
            Vector2 pan = Input.GetMouseButton(2)
                ? new Vector2(Input.GetAxisRaw("Mouse X"), Input.GetAxisRaw("Mouse Y"))
                : Vector2.zero;

            return new OrbitCameraCommand(
                orbit,
                pan,
                Input.mouseScrollDelta.y,
                Input.GetKeyDown(resetKey),
                blocked);
        }

        private void ApplyPose()
        {
            transform.SetPositionAndRotation(state.Position, state.Rotation);
        }
    }
}
