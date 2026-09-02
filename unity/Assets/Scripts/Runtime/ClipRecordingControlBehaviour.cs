using System;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class ClipRecordingControlBehaviour : MonoBehaviour
    {
        [Header("Receiver control")]
        [SerializeField]
        private string endpoint = ClipControlClient.DefaultEndpoint;

        [Header("Panel")]
        [SerializeField]
        private bool showPanel = true;

        [SerializeField, Min(200f)]
        private float panelWidth = 600f;

        [SerializeField, Min(120f)]
        private float panelHeight = 220f;

        [SerializeField, Min(0f)]
        private float panelMargin = 16f;

        [SerializeField, Min(8)]
        private int fontSize = 18;

        private ClipControlClient client;
        private GUIStyle textStyle;
        private readonly ClipRecordingViewState state = new ClipRecordingViewState();

        public ClipRecordingViewState State => state;
        public bool IsVisible => showPanel;

        private void OnEnable()
        {
            client = new ClipControlClient(endpoint);
            client.Start();
        }

        private void Update()
        {
            if (client == null)
            {
                return;
            }

            while (client.TryDequeueResponse(out ClipControlResponse response))
            {
                state.Apply(response);
            }
        }

        private void OnDisable()
        {
            if (client == null)
            {
                return;
            }

            try
            {
                client.Dispose();
            }
            catch (Exception exception)
            {
                Debug.LogError("Failed to stop clip control client: " + exception.Message, this);
            }
            finally
            {
                client = null;
            }
        }

        private void OnValidate()
        {
            panelWidth = Mathf.Max(200f, panelWidth);
            panelHeight = Mathf.Max(120f, panelHeight);
            panelMargin = Mathf.Max(0f, panelMargin);
            fontSize = Mathf.Max(8, fontSize);
            textStyle = null;
        }

        private void OnGUI()
        {
            if (!showPanel)
            {
                return;
            }

            EnsureStyle();
            Rect panel = ResolvePanelRect(Screen.height);
            GUI.Box(panel, GUIContent.none);
            GUI.Label(
                new Rect(panel.x + 12f, panel.y + 10f, panel.width - 24f, 126f),
                BuildStatusText(),
                textStyle);

            bool previousEnabled = GUI.enabled;
            GUI.enabled = state.CanStart;
            if (GUI.Button(new Rect(panel.x + 12f, panel.y + 150f, 180f, 48f), "Start Clip"))
            {
                StartClip();
            }
            GUI.enabled = state.CanStop;
            if (GUI.Button(new Rect(panel.x + 204f, panel.y + 150f, 180f, 48f), "Stop Clip"))
            {
                StopClip();
            }
            GUI.enabled = previousEnabled;
        }

        public void StartClip()
        {
            client?.StartClip();
        }

        public void StopClip()
        {
            client?.StopClip();
        }

        public void ApplyResponse(ClipControlResponse response)
        {
            state.Apply(response);
        }

        public bool ContainsScreenPoint(Vector2 screenPoint, float screenHeight)
        {
            if (!showPanel || !IsFinite(screenPoint.x) || !IsFinite(screenPoint.y) ||
                !IsFinite(screenHeight) || screenHeight < 0f)
            {
                return false;
            }

            var guiPoint = new Vector2(screenPoint.x, screenHeight - screenPoint.y);
            return ResolvePanelRect(screenHeight).Contains(guiPoint);
        }

        public Rect ResolvePanelRect(float screenHeight)
        {
            return new Rect(
                panelMargin,
                Mathf.Max(panelMargin, screenHeight - panelMargin - panelHeight),
                panelWidth,
                panelHeight);
        }

        private string BuildStatusText()
        {
            string output = string.IsNullOrEmpty(state.OutputPath) ? "-" : state.OutputPath;
            string error = string.IsNullOrEmpty(state.Error) ? "-" : state.Error;
            return
                "Telemetry Clip: " + state.State + "\n" +
                "Session: " + (string.IsNullOrEmpty(state.Session) ? "-" : state.Session) +
                "   Elapsed: " + state.ElapsedSeconds.ToString("F2") + " s" +
                "   Messages: " + state.MessageCount + "\n" +
                "Output: " + output + "\n" +
                "Error: " + error;
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
                wordWrap = true,
                clipping = TextClipping.Clip
            };
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
