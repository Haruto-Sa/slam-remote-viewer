using System;
using NUnit.Framework;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class CameraFrustumGeometryTests
    {
        [Test]
        public void BuildsExpectedCornersFromAspectFieldOfViewAndDepth()
        {
            Vector3[] positions = CameraFrustumGeometry.Build(2f, 90f, 2f);

            Assert.That(positions, Has.Length.EqualTo(12));
            AssertVector(positions[0], Vector3.zero);
            AssertVector(positions[1], new Vector3(-4f, 2f, 2f));
            AssertVector(positions[2], new Vector3(4f, 2f, 2f));
            AssertVector(positions[3], new Vector3(4f, -2f, 2f));
            AssertVector(positions[4], new Vector3(-4f, -2f, 2f));
        }

        [TestCase(0f, 60f, 1f)]
        [TestCase(1f, 0f, 1f)]
        [TestCase(1f, 180f, 1f)]
        [TestCase(1f, 60f, 0f)]
        public void RejectsInvalidProjectionValues(float aspect, float fieldOfView, float depth)
        {
            Assert.Throws<ArgumentOutOfRangeException>(
                () => CameraFrustumGeometry.Build(aspect, fieldOfView, depth));
        }

        private static void AssertVector(Vector3 actual, Vector3 expected)
        {
            Assert.That(Vector3.Distance(actual, expected), Is.LessThan(0.0001f));
        }
    }
}
