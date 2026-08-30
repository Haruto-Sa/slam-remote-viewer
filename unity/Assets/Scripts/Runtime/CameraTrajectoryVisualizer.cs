using System.Collections.Generic;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class CameraTrajectoryVisualizer : MonoBehaviour
    {
        [Header("Telemetry")]
        [SerializeField]
        private TelemetrySubscriberBehaviour subscriber;

        [Header("Trajectory")]
        [SerializeField]
        private Transform trajectoryRoot;

        [Header("Visibility")]
        [SerializeField]
        private bool showTrajectory = true;

        [SerializeField]
        private Material lineMaterial;

        [SerializeField]
        private Color lineColor = new Color(0.1f, 0.8f, 1f, 1f);

        [SerializeField, Min(0.001f)]
        private float lineWidth = 0.02f;

        [SerializeField, Min(0f)]
        private float minimumPointDistance = 0.02f;

        [SerializeField, Min(1)]
        private int maximumPointCount = 10000;

        private readonly List<LineRenderer> segmentRenderers = new List<LineRenderer>();
        private TrajectoryHistory history;
        private Material generatedLineMaterial;

        public string ActiveSession => history?.ActiveSession;
        public int PointCount => history?.PointCount ?? 0;
        public int SegmentCount => history?.SegmentCount ?? 0;
        public Transform TrajectoryRoot => trajectoryRoot;
        public IReadOnlyList<LineRenderer> SegmentRenderers => segmentRenderers;
        public bool IsVisible => showTrajectory;

        private void Awake()
        {
            Initialize();
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
            lineWidth = Mathf.Max(0.001f, lineWidth);
            minimumPointDistance = Mathf.Max(0f, minimumPointDistance);
            maximumPointCount = Mathf.Max(1, maximumPointCount);

            foreach (LineRenderer segmentRenderer in segmentRenderers)
            {
                ConfigureRenderer(segmentRenderer);
            }
        }

        public void HandleMessage(ITelemetryMessage message)
        {
            if (history == null || trajectoryRoot == null)
            {
                Initialize();
            }

            if (history.HandleMessage(message))
            {
                RefreshRenderers();
            }
        }

        public IReadOnlyList<Vector3> GetSegment(int index)
        {
            return history.GetSegment(index);
        }

        public void SetVisible(bool visible)
        {
            showTrajectory = visible;
            foreach (LineRenderer renderer in segmentRenderers)
            {
                if (renderer != null)
                {
                    renderer.enabled = showTrajectory;
                }
            }
        }

        private void Initialize()
        {
            if (trajectoryRoot == null)
            {
                var rootObject = new GameObject("Camera Trajectory");
                trajectoryRoot = rootObject.transform;
                trajectoryRoot.SetParent(transform, false);
            }

            history = new TrajectoryHistory(
                Mathf.Max(1, maximumPointCount),
                Mathf.Max(0f, minimumPointDistance));
            ClearRenderers();
        }

        private void RefreshRenderers()
        {
            while (segmentRenderers.Count < history.SegmentCount)
            {
                segmentRenderers.Add(CreateRenderer(segmentRenderers.Count));
            }

            while (segmentRenderers.Count > history.SegmentCount)
            {
                int lastIndex = segmentRenderers.Count - 1;
                LineRenderer renderer = segmentRenderers[lastIndex];
                segmentRenderers.RemoveAt(lastIndex);
                DestroyRenderer(renderer);
            }

            for (var segmentIndex = 0; segmentIndex < history.SegmentCount; segmentIndex++)
            {
                IReadOnlyList<Vector3> segment = history.GetSegment(segmentIndex);
                var positions = new Vector3[segment.Count];
                for (var pointIndex = 0; pointIndex < segment.Count; pointIndex++)
                {
                    positions[pointIndex] = segment[pointIndex];
                }

                LineRenderer renderer = segmentRenderers[segmentIndex];
                renderer.positionCount = positions.Length;
                renderer.SetPositions(positions);
            }
        }

        private LineRenderer CreateRenderer(int segmentIndex)
        {
            var segmentObject = new GameObject("Trajectory Segment " + segmentIndex);
            segmentObject.transform.SetParent(trajectoryRoot, false);
            LineRenderer renderer = segmentObject.AddComponent<LineRenderer>();
            ConfigureRenderer(renderer);
            return renderer;
        }

        private void ConfigureRenderer(LineRenderer renderer)
        {
            if (renderer == null)
            {
                return;
            }

            renderer.useWorldSpace = true;
            renderer.loop = false;
            renderer.startWidth = lineWidth;
            renderer.endWidth = lineWidth;
            renderer.startColor = lineColor;
            renderer.endColor = lineColor;
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            renderer.receiveShadows = false;
            renderer.enabled = showTrajectory;

            if (lineMaterial != null)
            {
                renderer.sharedMaterial = lineMaterial;
            }
            else if (renderer.sharedMaterial == null)
            {
                renderer.sharedMaterial = GetOrCreateDefaultMaterial();
            }
        }

        private Material GetOrCreateDefaultMaterial()
        {
            if (generatedLineMaterial != null)
            {
                return generatedLineMaterial;
            }

            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null)
            {
                return null;
            }

            generatedLineMaterial = new Material(shader)
            {
                name = "Generated Camera Trajectory Material"
            };
            return generatedLineMaterial;
        }

        private void ClearRenderers()
        {
            foreach (LineRenderer renderer in segmentRenderers)
            {
                DestroyRenderer(renderer);
            }

            segmentRenderers.Clear();
        }

        private static void DestroyRenderer(LineRenderer renderer)
        {
            if (renderer == null)
            {
                return;
            }

            if (Application.isPlaying)
            {
                Destroy(renderer.gameObject);
            }
            else
            {
                DestroyImmediate(renderer.gameObject);
            }
        }
    }
}
