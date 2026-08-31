using System;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class ViewerControlsOverlay : MonoBehaviour
    {
        [Header("Shortcut sources")]
        [SerializeField]
        private OrbitCameraController orbitCameraController;

        [SerializeField]
        private VisualizationVisibilityController visibilityController;

        [Header("Overlay")]
        [SerializeField]
        private bool showOverlay = true;

        [SerializeField]
        private KeyCode toggleKey = KeyCode.H;

        [SerializeField]
        private Rect panelRect = new Rect(544f, 16f, 400f, 330f);

        [SerializeField, Min(8)]
        private int fontSize = 18;

        private GUIStyle textStyle;

        public bool IsVisible => showOverlay;
        public bool ShowOverlay
        {
            get => showOverlay;
            set => showOverlay = value;
        }
        public KeyCode ToggleKey => toggleKey;
        public Rect PanelRect => panelRect;

        private void Awake()
        {
            ResolveSources();
            OnValidate();
        }

        private void Update()
        {
            if (Input.GetKeyDown(toggleKey))
            {
                ToggleVisible();
            }
        }

        private void OnValidate()
        {
            fontSize = Mathf.Max(8, fontSize);
            panelRect.width = Mathf.Max(200f, FiniteOr(panelRect.width, 400f));
            panelRect.height = Mathf.Max(100f, FiniteOr(panelRect.height, 330f));
            panelRect.x = FiniteOr(panelRect.x, 544f);
            panelRect.y = FiniteOr(panelRect.y, 16f);
            textStyle = null;
        }

        private void OnGUI()
        {
            if (!showOverlay)
            {
                return;
            }

            EnsureStyle();
            GUI.Box(panelRect, GUIContent.none);
            GUI.Label(
                new Rect(
                    panelRect.x + 12f,
                    panelRect.y + 10f,
                    panelRect.width - 24f,
                    panelRect.height - 20f),
                BuildContent(),
                textStyle);
        }

        public void SetVisible(bool visible)
        {
            showOverlay = visible;
        }

        public void ToggleVisible()
        {
            showOverlay = !showOverlay;
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

        public string BuildContent()
        {
            ResolveSources();

            KeyCode front = orbitCameraController != null
                ? orbitCameraController.FrontViewKey
                : KeyCode.Alpha1;
            KeyCode right = orbitCameraController != null
                ? orbitCameraController.RightViewKey
                : KeyCode.Alpha2;
            KeyCode back = orbitCameraController != null
                ? orbitCameraController.BackViewKey
                : KeyCode.Alpha3;
            KeyCode left = orbitCameraController != null
                ? orbitCameraController.LeftViewKey
                : KeyCode.Alpha4;
            KeyCode frame = orbitCameraController != null
                ? orbitCameraController.FrameAllKey
                : KeyCode.F;
            KeyCode reset = orbitCameraController != null
                ? orbitCameraController.ResetKey
                : KeyCode.R;
            KeyCode pose = visibilityController != null
                ? visibilityController.CameraPoseKey
                : KeyCode.P;
            KeyCode trajectory = visibilityController != null
                ? visibilityController.TrajectoryKey
                : KeyCode.T;
            KeyCode points = visibilityController != null
                ? visibilityController.PointCloudKey
                : KeyCode.C;
            KeyCode world = visibilityController != null
                ? visibilityController.WorldReferenceKey
                : KeyCode.G;
            KeyCode diagnostics = visibilityController != null
                ? visibilityController.DiagnosticsKey
                : KeyCode.D;
            KeyCode restore = visibilityController != null
                ? visibilityController.RestoreDefaultsKey
                : KeyCode.V;

            return
                "Viewer Controls\n" +
                "Mouse\n" +
                "  RMB drag       Orbit\n" +
                "  MMB drag       Pan\n" +
                "  Wheel          Zoom\n" +
                "Views\n" +
                "  " + FormatKey(front) + " Front      " + FormatKey(right) + " Right\n" +
                "  " + FormatKey(back) + " Back       " + FormatKey(left) + " Left\n" +
                "  " + FormatKey(frame) + "              Frame visible telemetry\n" +
                "  " + FormatKey(reset) + "              Reset camera\n" +
                "Visibility\n" +
                "  " + FormatKey(pose) + " Pose       " + FormatKey(trajectory) +
                " Trajectory\n" +
                "  " + FormatKey(points) + " Point cloud  " + FormatKey(world) + " Grid\n" +
                "  " + FormatKey(diagnostics) + " Diagnostics " + FormatKey(restore) +
                " Restore\n" +
                "  " + FormatKey(toggleKey) + "              Hide this help";
        }

        public static string FormatKey(KeyCode key)
        {
            string name = key.ToString();
            return name.StartsWith("Alpha", StringComparison.Ordinal) && name.Length > 5
                ? name.Substring(5)
                : name;
        }

        private void ResolveSources()
        {
            if (orbitCameraController == null)
            {
                orbitCameraController = FindFirstObjectByType<OrbitCameraController>();
            }
            if (visibilityController == null)
            {
                visibilityController = GetComponent<VisualizationVisibilityController>();
            }
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

        private static float FiniteOr(float value, float fallback)
        {
            return IsFinite(value) ? value : fallback;
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
