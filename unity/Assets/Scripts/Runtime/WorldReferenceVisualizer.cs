using UnityEngine;
using UnityEngine.Rendering;

namespace Slam.RemoteViewer
{
    public sealed class WorldReferenceVisualizer : MonoBehaviour
    {
        [Header("Visibility")]
        [SerializeField]
        private bool showReference = true;

        [Header("Grid")]
        [SerializeField, Min(0.01f)]
        private float gridSpacing = 1f;

        [SerializeField, Min(0.01f)]
        private float gridExtent = 10f;

        [SerializeField, Min(0.001f)]
        private float lineWidth = 0.025f;

        [SerializeField]
        private Color gridColor = new Color(0.55f, 0.6f, 0.65f, 0.55f);

        [Header("Positive axes")]
        [SerializeField, Min(0.01f)]
        private float axisLength = 2f;

        [SerializeField]
        private Color xAxisColor = new Color(1f, 0.2f, 0.2f, 1f);

        [SerializeField]
        private Color yAxisColor = new Color(0.2f, 1f, 0.2f, 1f);

        [SerializeField]
        private Color zAxisColor = new Color(0.2f, 0.45f, 1f, 1f);

        private MeshFilter gridMeshFilter;
        private MeshRenderer gridRenderer;
        private LineRenderer xAxisLine;
        private LineRenderer yAxisLine;
        private LineRenderer zAxisLine;
        private Mesh generatedGridMesh;
        private Material generatedLineMaterial;

        public bool ShowReference
        {
            get => showReference;
            set
            {
                showReference = value;
                SetRendererVisibility(showReference && isActiveAndEnabled);
            }
        }

        public bool IsVisible => showReference;

        public float GridSpacing => gridSpacing;
        public float GridExtent => gridExtent;
        public float LineWidth => lineWidth;
        public float AxisLength => axisLength;
        public MeshRenderer GridRenderer => gridRenderer;
        public LineRenderer XAxisLine => xAxisLine;
        public LineRenderer YAxisLine => yAxisLine;
        public LineRenderer ZAxisLine => zAxisLine;
        public int RendererCount => GetComponentsInChildren<Renderer>(true).Length;

        public void SetVisible(bool visible)
        {
            ShowReference = visible;
        }

        private void Awake()
        {
            Initialize();
        }

        public void Rebuild()
        {
            Initialize();
        }

        private void OnEnable()
        {
            EnsureVisuals();
            SetRendererVisibility(showReference);
        }

        private void OnDisable()
        {
            SetRendererVisibility(false);
        }

        private void OnDestroy()
        {
            DestroyGeneratedObject(generatedGridMesh);
            DestroyGeneratedObject(generatedLineMaterial);
        }

        private void OnValidate()
        {
            gridSpacing = PositiveFiniteOrDefault(gridSpacing, 1f, 0.01f);
            gridExtent = PositiveFiniteOrDefault(gridExtent, 10f, gridSpacing);
            gridExtent = Mathf.Clamp(gridExtent, gridSpacing, gridSpacing * 1000f);
            lineWidth = PositiveFiniteOrDefault(lineWidth, 0.025f, 0.001f);
            lineWidth = Mathf.Min(lineWidth, gridSpacing);
            axisLength = PositiveFiniteOrDefault(axisLength, 2f, 0.01f);
            if (gridMeshFilter != null)
            {
                RebuildVisuals();
            }
        }

        private void Initialize()
        {
            OnValidate();
            EnsureVisuals();
            RebuildVisuals();
            SetRendererVisibility(showReference && isActiveAndEnabled);
        }

        private void EnsureVisuals()
        {
            if (gridMeshFilter == null)
            {
                var gridObject = new GameObject("World Grid");
                gridObject.transform.SetParent(transform, false);
                gridMeshFilter = gridObject.AddComponent<MeshFilter>();
                gridRenderer = gridObject.AddComponent<MeshRenderer>();
            }

            if (xAxisLine == null)
            {
                xAxisLine = CreateAxisLine("Positive X Axis");
            }
            if (yAxisLine == null)
            {
                yAxisLine = CreateAxisLine("Positive Y Axis");
            }
            if (zAxisLine == null)
            {
                zAxisLine = CreateAxisLine("Positive Z Axis");
            }

            EnsureMaterial();
            gridRenderer.sharedMaterial = generatedLineMaterial;
            ConfigureAxis(xAxisLine, Vector3.right * axisLength, xAxisColor);
            ConfigureAxis(yAxisLine, Vector3.up * axisLength, yAxisColor);
            ConfigureAxis(zAxisLine, Vector3.forward * axisLength, zAxisColor);
        }

        private LineRenderer CreateAxisLine(string objectName)
        {
            var axisObject = new GameObject(objectName);
            axisObject.transform.SetParent(transform, false);
            return axisObject.AddComponent<LineRenderer>();
        }

        private void EnsureMaterial()
        {
            if (generatedLineMaterial != null)
            {
                return;
            }

            Shader shader = Shader.Find("Sprites/Default");
            if (shader != null)
            {
                generatedLineMaterial = new Material(shader)
                {
                    name = "Generated World Reference Material"
                };
            }
        }

        private void RebuildVisuals()
        {
            EnsureVisuals();
            WorldGridMeshData geometry = WorldReferenceGeometry.BuildGrid(
                gridSpacing,
                gridExtent,
                lineWidth,
                gridColor);

            if (generatedGridMesh == null)
            {
                generatedGridMesh = new Mesh
                {
                    name = "Generated World Grid Mesh"
                };
            }
            else
            {
                generatedGridMesh.Clear();
            }

            generatedGridMesh.vertices = geometry.Vertices;
            generatedGridMesh.colors = geometry.Colors;
            generatedGridMesh.triangles = geometry.Triangles;
            generatedGridMesh.RecalculateBounds();
            gridMeshFilter.sharedMesh = generatedGridMesh;

            ConfigureAxis(xAxisLine, Vector3.right * axisLength, xAxisColor);
            ConfigureAxis(yAxisLine, Vector3.up * axisLength, yAxisColor);
            ConfigureAxis(zAxisLine, Vector3.forward * axisLength, zAxisColor);
            SetRendererVisibility(showReference && isActiveAndEnabled);
        }

        private void ConfigureAxis(LineRenderer line, Vector3 end, Color color)
        {
            line.useWorldSpace = false;
            line.loop = false;
            line.positionCount = 2;
            line.SetPosition(0, Vector3.zero);
            line.SetPosition(1, end);
            line.startWidth = lineWidth * 2f;
            line.endWidth = lineWidth * 2f;
            line.startColor = color;
            line.endColor = color;
            line.sharedMaterial = generatedLineMaterial;
            line.shadowCastingMode = ShadowCastingMode.Off;
            line.receiveShadows = false;
            line.sortingOrder = 1;
        }

        private void SetRendererVisibility(bool visible)
        {
            if (gridRenderer != null)
            {
                gridRenderer.enabled = visible;
            }
            if (xAxisLine != null)
            {
                xAxisLine.enabled = visible;
            }
            if (yAxisLine != null)
            {
                yAxisLine.enabled = visible;
            }
            if (zAxisLine != null)
            {
                zAxisLine.enabled = visible;
            }
        }

        private static void DestroyGeneratedObject(Object generatedObject)
        {
            if (generatedObject == null)
            {
                return;
            }

            if (Application.isPlaying)
            {
                Destroy(generatedObject);
            }
            else
            {
                DestroyImmediate(generatedObject);
            }
        }

        private static float PositiveFiniteOrDefault(
            float value,
            float defaultValue,
            float minimum)
        {
            if (float.IsNaN(value) || float.IsInfinity(value))
            {
                return defaultValue;
            }
            return Mathf.Max(minimum, value);
        }
    }
}
