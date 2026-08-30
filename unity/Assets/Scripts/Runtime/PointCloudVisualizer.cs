using System.Collections.Generic;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class PointCloudVisualizer : MonoBehaviour
    {
        [Header("Telemetry")]
        [SerializeField]
        private TelemetrySubscriberBehaviour subscriber;

        [Header("Point cloud")]
        [SerializeField]
        private ParticleSystem particleSystemTarget;

        [SerializeField]
        private Material pointMaterial;

        [SerializeField, Min(0.001f)]
        private float pointSize = 0.04f;

        [SerializeField]
        private Color pointColor = new Color(0.9f, 0.95f, 1f, 1f);

        [SerializeField]
        private bool showPointCloud = true;

        private readonly List<Vector3> orderedPositions = new List<Vector3>();
        private PointCloudState state;
        private Material generatedPointMaterial;

        public string ActiveSession => state?.ActiveSession;
        public int PointCount => state?.PointCount ?? 0;
        public long StateRevision => state?.Revision ?? 0;
        public long RenderRevision { get; private set; }
        public int RenderedPointCount { get; private set; }
        public ParticleSystem ParticleSystemTarget => particleSystemTarget;
        public bool IsVisible => showPointCloud;

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
            if (generatedPointMaterial == null)
            {
                return;
            }

            if (Application.isPlaying)
            {
                Destroy(generatedPointMaterial);
            }
            else
            {
                DestroyImmediate(generatedPointMaterial);
            }
        }

        private void OnValidate()
        {
            pointSize = Mathf.Max(0.001f, pointSize);
            if (particleSystemTarget != null)
            {
                ConfigureParticleSystem();
                RebuildParticles();
            }
        }

        public void HandleMessage(ITelemetryMessage message)
        {
            if (state == null || particleSystemTarget == null)
            {
                Initialize();
            }

            if (state.HandleMessage(message))
            {
                RebuildParticles();
            }
        }

        public bool TryGetPoint(ulong id, out Vector3 position)
        {
            if (state == null)
            {
                position = default;
                return false;
            }

            return state.TryGetPoint(id, out position);
        }

        public void SetVisible(bool visible)
        {
            showPointCloud = visible;
            if (particleSystemTarget != null)
            {
                particleSystemTarget.GetComponent<ParticleSystemRenderer>().enabled = visible;
            }
        }

        private void Initialize()
        {
            state = new PointCloudState();
            if (particleSystemTarget == null)
            {
                var pointCloudObject = new GameObject("Point Cloud");
                pointCloudObject.transform.SetParent(transform, false);
                particleSystemTarget = pointCloudObject.AddComponent<ParticleSystem>();
                particleSystemTarget.Stop(true, ParticleSystemStopBehavior.StopEmittingAndClear);
            }

            ConfigureParticleSystem();
            RebuildParticles();
        }

        private void ConfigureParticleSystem()
        {
            ParticleSystem.MainModule main = particleSystemTarget.main;
            main.loop = false;
            main.playOnAwake = false;
            main.simulationSpace = ParticleSystemSimulationSpace.World;
            main.maxParticles = Mathf.Max(1, state?.PointCount ?? 1);

            ParticleSystem.EmissionModule emission = particleSystemTarget.emission;
            emission.enabled = false;
            ParticleSystem.ShapeModule shape = particleSystemTarget.shape;
            shape.enabled = false;

            var renderer = particleSystemTarget.GetComponent<ParticleSystemRenderer>();
            renderer.renderMode = ParticleSystemRenderMode.Billboard;
            renderer.enabled = showPointCloud;
            if (pointMaterial != null)
            {
                renderer.sharedMaterial = pointMaterial;
            }
            else if (renderer.sharedMaterial == null)
            {
                renderer.sharedMaterial = GetOrCreateDefaultMaterial();
            }
        }

        private void RebuildParticles()
        {
            if (particleSystemTarget == null || state == null)
            {
                return;
            }

            ConfigureParticleSystem();
            state.CopyOrderedPositions(orderedPositions);
            if (orderedPositions.Count == 0)
            {
                particleSystemTarget.Clear(true);
                SetRendererBounds(new Bounds(Vector3.zero, Vector3.zero));
                RenderedPointCount = 0;
                RenderRevision++;
                return;
            }

            var particles = new ParticleSystem.Particle[orderedPositions.Count];
            for (var index = 0; index < orderedPositions.Count; index++)
            {
                particles[index] = new ParticleSystem.Particle
                {
                    position = orderedPositions[index],
                    startColor = pointColor,
                    startSize = pointSize,
                    startLifetime = 1000000f,
                    remainingLifetime = 1000000f
                };
            }

            particleSystemTarget.SetParticles(particles, particles.Length);
            UpdateRendererBounds();
            RenderedPointCount = particles.Length;
            if (!particleSystemTarget.isPlaying)
            {
                // Emission remains disabled. Playing keeps the manually supplied
                // particles alive and visible in both EditMode tests and Play Mode.
                particleSystemTarget.Play(false);
            }

            RenderRevision++;
        }

        private void UpdateRendererBounds()
        {
            Transform particleTransform = particleSystemTarget.transform;
            Vector3 firstPosition = particleTransform.InverseTransformPoint(orderedPositions[0]);
            var bounds = new Bounds(firstPosition, Vector3.one * pointSize);
            for (var index = 1; index < orderedPositions.Count; index++)
            {
                Vector3 localPosition = particleTransform.InverseTransformPoint(
                    orderedPositions[index]);
                bounds.Encapsulate(new Bounds(localPosition, Vector3.one * pointSize));
            }

            SetRendererBounds(bounds);
        }

        private void SetRendererBounds(Bounds bounds)
        {
            var renderer = particleSystemTarget.GetComponent<ParticleSystemRenderer>();
            renderer.localBounds = bounds;
        }

        private Material GetOrCreateDefaultMaterial()
        {
            if (generatedPointMaterial != null)
            {
                return generatedPointMaterial;
            }

            Shader shader = Shader.Find("Particles/Standard Unlit");
            if (shader == null)
            {
                shader = Shader.Find("Sprites/Default");
            }

            if (shader == null)
            {
                return null;
            }

            generatedPointMaterial = new Material(shader)
            {
                name = "Generated Point Cloud Material"
            };
            return generatedPointMaterial;
        }
    }
}
