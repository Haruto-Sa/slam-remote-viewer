using System;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class WorldGridMeshData
    {
        public WorldGridMeshData(
            Vector3[] vertices,
            Color[] colors,
            int[] triangles,
            float actualExtent)
        {
            Vertices = vertices;
            Colors = colors;
            Triangles = triangles;
            ActualExtent = actualExtent;
        }

        public Vector3[] Vertices { get; }
        public Color[] Colors { get; }
        public int[] Triangles { get; }
        public float ActualExtent { get; }
    }

    public static class WorldReferenceGeometry
    {
        private const int MaximumStepsEachSide = 1000;

        public static WorldGridMeshData BuildGrid(
            float spacing,
            float extent,
            float lineWidth,
            Color color)
        {
            ValidatePositiveFinite(spacing, nameof(spacing));
            ValidatePositiveFinite(extent, nameof(extent));
            ValidatePositiveFinite(lineWidth, nameof(lineWidth));
            if (extent < spacing)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(extent),
                    "grid extent must be at least one grid spacing");
            }
            if (!IsFinite(color.r) || !IsFinite(color.g) ||
                !IsFinite(color.b) || !IsFinite(color.a))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(color),
                    "grid color must contain finite values");
            }

            int stepsEachSide = Mathf.FloorToInt(extent / spacing + 0.0001f);
            if (stepsEachSide > MaximumStepsEachSide)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(extent),
                    $"grid may contain at most {MaximumStepsEachSide} steps on each side");
            }

            float actualExtent = stepsEachSide * spacing;
            int positionsPerDirection = stepsEachSide * 2 + 1;
            int lineCount = positionsPerDirection * 2;
            var vertices = new Vector3[lineCount * 4];
            var colors = new Color[vertices.Length];
            var triangles = new int[lineCount * 6];
            int lineIndex = 0;

            for (int step = -stepsEachSide; step <= stepsEachSide; step++)
            {
                float coordinate = step * spacing;
                AddQuad(
                    vertices,
                    colors,
                    triangles,
                    lineIndex++,
                    new Vector3(coordinate, 0f, -actualExtent),
                    new Vector3(coordinate, 0f, actualExtent),
                    Vector3.right,
                    lineWidth,
                    color);
                AddQuad(
                    vertices,
                    colors,
                    triangles,
                    lineIndex++,
                    new Vector3(-actualExtent, 0f, coordinate),
                    new Vector3(actualExtent, 0f, coordinate),
                    -Vector3.forward,
                    lineWidth,
                    color);
            }

            return new WorldGridMeshData(vertices, colors, triangles, actualExtent);
        }

        private static void AddQuad(
            Vector3[] vertices,
            Color[] colors,
            int[] triangles,
            int lineIndex,
            Vector3 start,
            Vector3 end,
            Vector3 widthDirection,
            float lineWidth,
            Color color)
        {
            int vertexIndex = lineIndex * 4;
            Vector3 halfWidth = widthDirection * (lineWidth * 0.5f);
            vertices[vertexIndex] = start - halfWidth;
            vertices[vertexIndex + 1] = start + halfWidth;
            vertices[vertexIndex + 2] = end + halfWidth;
            vertices[vertexIndex + 3] = end - halfWidth;
            colors[vertexIndex] = color;
            colors[vertexIndex + 1] = color;
            colors[vertexIndex + 2] = color;
            colors[vertexIndex + 3] = color;

            int triangleIndex = lineIndex * 6;
            triangles[triangleIndex] = vertexIndex;
            triangles[triangleIndex + 1] = vertexIndex + 2;
            triangles[triangleIndex + 2] = vertexIndex + 1;
            triangles[triangleIndex + 3] = vertexIndex;
            triangles[triangleIndex + 4] = vertexIndex + 3;
            triangles[triangleIndex + 5] = vertexIndex + 2;
        }

        private static void ValidatePositiveFinite(float value, string name)
        {
            if (!IsFinite(value) || value <= 0f)
            {
                throw new ArgumentOutOfRangeException(name, "value must be positive and finite");
            }
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
