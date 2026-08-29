using System;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class TelemetryDiagnosticsOverlay : MonoBehaviour
    {
        [Header("Sources")]
        [SerializeField]
        private TelemetrySubscriberBehaviour subscriber;

        [SerializeField]
        private CameraPoseVisualizer cameraPoseVisualizer;

        [SerializeField]
        private CameraTrajectoryVisualizer trajectoryVisualizer;

        [SerializeField]
        private PointCloudVisualizer pointCloudVisualizer;

        [Header("Health")]
        [SerializeField, Min(0.1f)]
        private float staleTimeoutSeconds = 2f;

        [Header("Overlay")]
        [SerializeField]
        private bool showOverlay = true;

        [SerializeField]
        private Rect panelRect = new Rect(16f, 16f, 520f, 360f);

        [SerializeField, Min(8)]
        private int fontSize = 18;

        private GUIStyle textStyle;
        private TelemetryDiagnosticsState state;

        public TelemetryDiagnosticsState State => state;
        public Rect PanelRect => panelRect;
        public bool ShowOverlay
        {
            get => showOverlay;
            set => showOverlay = value;
        }

        public bool ContainsScreenPoint(Vector2 screenPoint, float screenHeight)
        {
            if (!showOverlay || !IsFinite(screenPoint.x) || !IsFinite(screenPoint.y) ||
                !IsFinite(screenHeight) || screenHeight < 0f)
            {
                return false;
            }

            var guiPoint = new Vector2(screenPoint.x, screenHeight - screenPoint.y);
            return panelRect.Contains(guiPoint);
        }

        private void Awake()
        {
            Initialize();
        }

        private void OnEnable()
        {
            ResolveSources();
            EnsureState();
            state.Start(subscriber != null ? subscriber.Endpoint : null);
            if (subscriber != null)
            {
                subscriber.MessageReceived += HandleMessage;
            }
        }

        private void Update()
        {
            RefreshMetrics();
        }

        private void OnDisable()
        {
            if (subscriber != null)
            {
                subscriber.MessageReceived -= HandleMessage;
            }
            state?.SetRunning(false);
        }

        private void OnValidate()
        {
            staleTimeoutSeconds = Mathf.Max(0.1f, staleTimeoutSeconds);
            fontSize = Mathf.Max(8, fontSize);
            if (state != null)
            {
                state.StaleTimeoutSeconds = staleTimeoutSeconds;
            }
            textStyle = null;
        }

        private void OnGUI()
        {
            if (!showOverlay || state == null)
            {
                return;
            }

            EnsureStyle();
            double now = Time.realtimeSinceStartupAsDouble;
            TelemetryHealthStatus status = state.GetStatus(now);
            GUI.Box(panelRect, GUIContent.none);

            var contentRect = new Rect(
                panelRect.x + 12f,
                panelRect.y + 10f,
                panelRect.width - 24f,
                panelRect.height - 20f);

            Color previousColor = GUI.color;
            GUI.color = ColorFor(status);
            GUI.Label(
                new Rect(contentRect.x, contentRect.y, contentRect.width, 24f),
                "Telemetry: " + status,
                textStyle);
            GUI.color = previousColor;

            GUI.Label(
                new Rect(contentRect.x, contentRect.y + 26f, contentRect.width, contentRect.height - 26f),
                BuildDetails(now),
                textStyle);
        }

        public void HandleMessage(ITelemetryMessage message)
        {
            EnsureState();
            state.Observe(message, Time.realtimeSinceStartupAsDouble);
        }

        private void Initialize()
        {
            staleTimeoutSeconds = Mathf.Max(0.1f, staleTimeoutSeconds);
            fontSize = Mathf.Max(8, fontSize);
            state = new TelemetryDiagnosticsState(staleTimeoutSeconds);
            ResolveSources();
        }

        private void EnsureState()
        {
            if (state == null)
            {
                Initialize();
            }
        }

        private void ResolveSources()
        {
            if (subscriber == null)
            {
                subscriber = GetComponent<TelemetrySubscriberBehaviour>();
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

        private void RefreshMetrics()
        {
            EnsureState();
            state.SetRunning(subscriber != null && subscriber.IsRunning);
            state.UpdateMetrics(
                subscriber?.AcceptedCount ?? 0L,
                subscriber?.RejectedCount ?? 0L,
                subscriber?.DroppedCount ?? 0L,
                subscriber?.QueueCount ?? 0,
                trajectoryVisualizer != null ? trajectoryVisualizer.PointCount : (int?)null,
                pointCloudVisualizer != null ? pointCloudVisualizer.PointCount : (int?)null,
                subscriber?.FaultCount ?? 0L,
                subscriber?.LastFault,
                subscriber?.LastRejectionReason);
        }

        private string BuildDetails(double now)
        {
            string age = state.GetLatestMessageAgeSeconds(now) is double ageSeconds
                ? ageSeconds.ToString("F2") + " s"
                : "-";
            string trajectoryPoints = state.TrajectoryPointCount?.ToString() ?? "N/A";
            string pointCloudPoints = state.PointCloudPointCount?.ToString() ?? "N/A";
            string tracking = state.TrackingState?.ToString() ?? "-";
            string session = string.IsNullOrEmpty(state.ActiveSession) ? "-" : state.ActiveSession;
            string endpoint = string.IsNullOrEmpty(state.Endpoint) ? "-" : state.Endpoint;
            string fault = string.IsNullOrEmpty(state.LastFault) ? "-" : state.LastFault;
            string rejection = string.IsNullOrEmpty(state.LastRejectionReason)
                ? "-"
                : state.LastRejectionReason;

            return
                "Endpoint: " + endpoint + "\n" +
                "Session: " + session + "\n" +
                "Tracking: " + tracking + "\n" +
                "Last message age: " + age + "\n" +
                "Messages: accepted " + state.AcceptedCount +
                " / rejected " + state.RejectedCount +
                " / dropped " + state.DroppedCount + "\n" +
                "Queue: " + state.QueueCount + "\n" +
                "Trajectory points: " + trajectoryPoints + "\n" +
                "Point-cloud points: " + pointCloudPoints + "\n" +
                "Subscriber faults: " + state.FaultCount + " (" + fault + ")\n" +
                "Last rejection: " + rejection;
        }

        private void EnsureStyle()
        {
            if (textStyle != null)
            {
                return;
            }

            textStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = fontSize,
                wordWrap = false,
                clipping = TextClipping.Clip
            };
        }

        private static Color ColorFor(TelemetryHealthStatus status)
        {
            switch (status)
            {
                case TelemetryHealthStatus.Receiving:
                    return new Color(0.25f, 1f, 0.35f, 1f);
                case TelemetryHealthStatus.Stale:
                    return new Color(1f, 0.75f, 0.15f, 1f);
                case TelemetryHealthStatus.Stopped:
                    return new Color(1f, 0.35f, 0.3f, 1f);
                default:
                    return Color.white;
            }
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
