using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class CameraPoseVisualizer : MonoBehaviour
    {
        private const string ColorProperty = "_Color";
        private const string BaseColorProperty = "_BaseColor";

        [Header("Telemetry")]
        [SerializeField]
        private TelemetrySubscriberBehaviour subscriber;

        [Header("Scene objects (created automatically when empty)")]
        [SerializeField]
        private Transform cameraPoseRoot;

        [SerializeField]
        private LineRenderer frustumLine;

        [Header("Visibility")]
        [SerializeField]
        private bool showPose = true;

        [Header("Frustum")]
        [SerializeField, Range(1f, 179f)]
        private float verticalFieldOfViewDegrees = 60f;

        [SerializeField, Min(0.01f)]
        private float frustumDepth = 0.5f;

        [SerializeField, Min(0.001f)]
        private float lineWidth = 0.01f;

        [SerializeField, Min(0.001f)]
        private float markerSize = 0.1f;

        [Header("Tracking-state colors")]
        [SerializeField]
        private Color trackingColor = new Color(0.2f, 1f, 0.3f, 1f);

        [SerializeField]
        private Color initializingColor = new Color(1f, 0.8f, 0.1f, 1f);

        [SerializeField]
        private Color lostColor = new Color(1f, 0.2f, 0.2f, 1f);

        [SerializeField]
        private Color relocalizingColor = new Color(0.3f, 0.6f, 1f, 1f);

        private MaterialPropertyBlock colorProperties;
        private Renderer[] markerRenderers;
        private Material generatedLineMaterial;
        private string activeSession;
        private float aspectRatio = 1f;

        public string ActiveSession => activeSession;
        public bool HasPose { get; private set; }
        public float AspectRatio => aspectRatio;
        public PoseTrackingState? TrackingState { get; private set; }
        public Transform CameraPoseTransform => cameraPoseRoot;
        public LineRenderer FrustumLine => frustumLine;
        public bool IsVisible => showPose;

        private void Awake()
        {
            colorProperties = new MaterialPropertyBlock();
            if (!IsFinite(aspectRatio) || aspectRatio <= 0f)
            {
                aspectRatio = 1f;
            }

            EnsureVisuals();
            ClearPose();
        }

        private void OnEnable()
        {
            if (subscriber == null)
            {
                subscriber = GetComponent<TelemetrySubscriberBehaviour>();
            }

            if (subscriber != null)
            {
                subscriber.MessageReceived += HandleMessage;
            }
        }

        private void OnDisable()
        {
            if (subscriber != null)
            {
                subscriber.MessageReceived -= HandleMessage;
            }
        }

        private void OnDestroy()
        {
            if (generatedLineMaterial == null)
            {
                return;
            }

            if (Application.isPlaying)
            {
                Destroy(generatedLineMaterial);
            }
            else
            {
                DestroyImmediate(generatedLineMaterial);
            }
        }

        private void OnValidate()
        {
            verticalFieldOfViewDegrees = Mathf.Clamp(verticalFieldOfViewDegrees, 1f, 179f);
            frustumDepth = Mathf.Max(0.01f, frustumDepth);
            lineWidth = Mathf.Max(0.001f, lineWidth);
            markerSize = Mathf.Max(0.001f, markerSize);

            if (frustumLine != null)
            {
                ConfigureLineRenderer();
                RebuildFrustum();
            }
        }

        public void HandleMessage(ITelemetryMessage message)
        {
            if (cameraPoseRoot == null || frustumLine == null)
            {
                EnsureVisuals();
                ClearPose();
            }

            if (message is SettingsMessage settings)
            {
                ApplySettings(settings);
                return;
            }

            if (message is PoseMessage pose)
            {
                ApplyPose(pose);
            }
        }

        public void SetVisible(bool visible)
        {
            showPose = visible;
            SetVisualsEnabled(showPose && HasPose);
        }

        public bool TryGetWorldBounds(out Bounds bounds)
        {
            if (!HasPose || cameraPoseRoot == null)
            {
                bounds = default;
                return false;
            }

            bounds = new Bounds(cameraPoseRoot.position, Vector3.zero);
            return true;
        }

        private void ApplySettings(SettingsMessage settings)
        {
            if (settings == null || settings.Camera == null ||
                settings.Camera.Width == 0 || settings.Camera.Height == 0)
            {
                return;
            }

            if (activeSession != settings.Session)
            {
                ClearPose();
                activeSession = settings.Session;
            }

            aspectRatio = (float)settings.Camera.Width / settings.Camera.Height;
            RebuildFrustum();
        }

        private void ApplyPose(PoseMessage pose)
        {
            if (pose == null || activeSession == null || pose.Session != activeSession ||
                pose.Position.Count != 3 || pose.OrientationXyzw.Count != 4)
            {
                return;
            }

            var position = new Vector3(
                (float)pose.Position[0],
                (float)pose.Position[1],
                (float)pose.Position[2]);
            var rotation = new Quaternion(
                (float)pose.OrientationXyzw[0],
                (float)pose.OrientationXyzw[1],
                (float)pose.OrientationXyzw[2],
                (float)pose.OrientationXyzw[3]);

            float magnitude = Mathf.Sqrt(
                rotation.x * rotation.x + rotation.y * rotation.y +
                rotation.z * rotation.z + rotation.w * rotation.w);
            if (!IsFinite(magnitude) || magnitude <= Mathf.Epsilon)
            {
                return;
            }

            rotation.x /= magnitude;
            rotation.y /= magnitude;
            rotation.z /= magnitude;
            rotation.w /= magnitude;

            cameraPoseRoot.SetPositionAndRotation(position, rotation);
            TrackingState = pose.State;
            HasPose = true;
            SetVisualsEnabled(showPose);
            ApplyStateColor(ColorFor(pose.State));
        }

        private void EnsureVisuals()
        {
            if (cameraPoseRoot == null)
            {
                var poseObject = new GameObject("Tracked Camera Pose");
                cameraPoseRoot = poseObject.transform;
                cameraPoseRoot.SetParent(transform, false);

                GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Cube);
                marker.name = "Camera Marker";
                marker.transform.SetParent(cameraPoseRoot, false);
                marker.transform.localScale = Vector3.one * markerSize;
            }

            if (frustumLine == null)
            {
                var lineObject = new GameObject("Camera Frustum");
                lineObject.transform.SetParent(cameraPoseRoot, false);
                frustumLine = lineObject.AddComponent<LineRenderer>();
            }
            else if (frustumLine.transform.parent != cameraPoseRoot)
            {
                frustumLine.transform.SetParent(cameraPoseRoot, false);
            }

            markerRenderers = cameraPoseRoot.GetComponentsInChildren<Renderer>(true);
            ConfigureLineRenderer();
            RebuildFrustum();
        }

        private void ConfigureLineRenderer()
        {
            frustumLine.useWorldSpace = false;
            frustumLine.loop = false;
            frustumLine.startWidth = lineWidth;
            frustumLine.endWidth = lineWidth;
            frustumLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            frustumLine.receiveShadows = false;

            if (frustumLine.sharedMaterial == null)
            {
                Shader shader = Shader.Find("Sprites/Default");
                if (shader != null)
                {
                    generatedLineMaterial = new Material(shader)
                    {
                        name = "Generated Camera Frustum Material"
                    };
                    frustumLine.sharedMaterial = generatedLineMaterial;
                }
            }
        }

        private void RebuildFrustum()
        {
            if (frustumLine == null)
            {
                return;
            }

            float safeAspectRatio = IsFinite(aspectRatio) && aspectRatio > 0f
                ? aspectRatio
                : 1f;
            aspectRatio = safeAspectRatio;
            Vector3[] positions = CameraFrustumGeometry.Build(
                safeAspectRatio,
                verticalFieldOfViewDegrees,
                frustumDepth);
            frustumLine.positionCount = positions.Length;
            frustumLine.SetPositions(positions);
        }

        private void ClearPose()
        {
            HasPose = false;
            TrackingState = null;
            if (cameraPoseRoot != null)
            {
                cameraPoseRoot.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            }

            SetVisualsEnabled(false);
        }

        private void SetVisualsEnabled(bool enabled)
        {
            if (markerRenderers != null)
            {
                foreach (Renderer markerRenderer in markerRenderers)
                {
                    if (markerRenderer != null && markerRenderer != frustumLine)
                    {
                        markerRenderer.enabled = enabled;
                    }
                }
            }

            if (frustumLine != null)
            {
                frustumLine.enabled = enabled;
            }
        }

        private void ApplyStateColor(Color color)
        {
            if (colorProperties == null)
            {
                colorProperties = new MaterialPropertyBlock();
            }

            if (markerRenderers != null)
            {
                foreach (Renderer markerRenderer in markerRenderers)
                {
                    if (markerRenderer == null || markerRenderer == frustumLine)
                    {
                        continue;
                    }

                    markerRenderer.GetPropertyBlock(colorProperties);
                    colorProperties.SetColor(ColorProperty, color);
                    colorProperties.SetColor(BaseColorProperty, color);
                    markerRenderer.SetPropertyBlock(colorProperties);
                }
            }

            frustumLine.startColor = color;
            frustumLine.endColor = color;
        }

        private Color ColorFor(PoseTrackingState state)
        {
            switch (state)
            {
                case PoseTrackingState.Tracking:
                    return trackingColor;
                case PoseTrackingState.Initializing:
                    return initializingColor;
                case PoseTrackingState.Lost:
                    return lostColor;
                case PoseTrackingState.Relocalizing:
                    return relocalizingColor;
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
