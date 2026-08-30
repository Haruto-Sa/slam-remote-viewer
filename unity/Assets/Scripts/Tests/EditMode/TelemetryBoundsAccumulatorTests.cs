using System;
using NUnit.Framework;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class TelemetryBoundsAccumulatorTests
    {
        [Test]
        public void EmptyAccumulatorHasNoBounds()
        {
            var accumulator = new TelemetryBoundsAccumulator();

            Assert.That(accumulator.TryGetBounds(out _), Is.False);
        }

        [Test]
        public void CombinesBoundsDeterministically()
        {
            var accumulator = new TelemetryBoundsAccumulator();
            accumulator.Add(new Bounds(new Vector3(-2f, 1f, 0f), new Vector3(2f, 2f, 2f)));
            accumulator.Add(new Bounds(new Vector3(4f, 3f, 2f), Vector3.zero));

            Assert.That(accumulator.TryGetBounds(out Bounds bounds), Is.True);
            AssertVector(bounds.min, new Vector3(-3f, 0f, -1f));
            AssertVector(bounds.max, new Vector3(4f, 3f, 2f));
        }

        [Test]
        public void RejectsNonFiniteBounds()
        {
            var accumulator = new TelemetryBoundsAccumulator();
            var bounds = new Bounds(Vector3.zero, Vector3.one);
            bounds.center = new Vector3(float.NaN, 0f, 0f);

            Assert.Throws<ArgumentOutOfRangeException>(() => accumulator.Add(bounds));
        }

        private static void AssertVector(Vector3 actual, Vector3 expected)
        {
            Assert.That(Vector3.Distance(actual, expected), Is.LessThan(0.0001f));
        }
    }
}
