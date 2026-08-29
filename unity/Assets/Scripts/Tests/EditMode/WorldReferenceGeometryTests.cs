using System;
using NUnit.Framework;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class WorldReferenceGeometryTests
    {
        [Test]
        public void BuildsCenteredDeterministicGrid()
        {
            Color color = new Color(0.2f, 0.3f, 0.4f, 0.5f);

            WorldGridMeshData geometry = WorldReferenceGeometry.BuildGrid(
                1f,
                2f,
                0.1f,
                color);

            Assert.That(geometry.ActualExtent, Is.EqualTo(2f));
            Assert.That(geometry.Vertices, Has.Length.EqualTo(40));
            Assert.That(geometry.Colors, Has.Length.EqualTo(40));
            Assert.That(geometry.Triangles, Has.Length.EqualTo(60));
            Assert.That(geometry.Colors, Has.All.EqualTo(color));
            Assert.That(MinimumX(geometry.Vertices), Is.EqualTo(-2.05f).Within(0.0001f));
            Assert.That(MaximumX(geometry.Vertices), Is.EqualTo(2.05f).Within(0.0001f));
            Assert.That(MinimumZ(geometry.Vertices), Is.EqualTo(-2.05f).Within(0.0001f));
            Assert.That(MaximumZ(geometry.Vertices), Is.EqualTo(2.05f).Within(0.0001f));
        }

        [Test]
        public void ExtentUsesOnlyCompleteGridSteps()
        {
            WorldGridMeshData geometry = WorldReferenceGeometry.BuildGrid(
                1f,
                2.4f,
                0.05f,
                Color.white);

            Assert.That(geometry.ActualExtent, Is.EqualTo(2f));
            Assert.That(geometry.Vertices, Has.Length.EqualTo(40));
        }

        [TestCase(0f, 1f, 0.1f)]
        [TestCase(1f, 0.5f, 0.1f)]
        [TestCase(1f, 1f, 0f)]
        public void RejectsInvalidConfiguration(float spacing, float extent, float lineWidth)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                WorldReferenceGeometry.BuildGrid(
                    spacing,
                    extent,
                    lineWidth,
                    Color.white));
        }

        [Test]
        public void RejectsNonFiniteConfigurationAndColor()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                WorldReferenceGeometry.BuildGrid(
                    float.NaN,
                    1f,
                    0.1f,
                    Color.white));
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                WorldReferenceGeometry.BuildGrid(
                    1f,
                    1f,
                    0.1f,
                    new Color(float.PositiveInfinity, 0f, 0f, 1f)));
        }

        private static float MinimumX(Vector3[] positions)
        {
            float minimum = float.PositiveInfinity;
            foreach (Vector3 position in positions)
            {
                minimum = Mathf.Min(minimum, position.x);
            }
            return minimum;
        }

        private static float MaximumX(Vector3[] positions)
        {
            float maximum = float.NegativeInfinity;
            foreach (Vector3 position in positions)
            {
                maximum = Mathf.Max(maximum, position.x);
            }
            return maximum;
        }

        private static float MinimumZ(Vector3[] positions)
        {
            float minimum = float.PositiveInfinity;
            foreach (Vector3 position in positions)
            {
                minimum = Mathf.Min(minimum, position.z);
            }
            return minimum;
        }

        private static float MaximumZ(Vector3[] positions)
        {
            float maximum = float.NegativeInfinity;
            foreach (Vector3 position in positions)
            {
                maximum = Mathf.Max(maximum, position.z);
            }
            return maximum;
        }
    }
}
