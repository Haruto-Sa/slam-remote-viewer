using System.Linq;
using NUnit.Framework;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class WorldReferenceVisualizerTests
    {
        private GameObject host;
        private WorldReferenceVisualizer visualizer;

        [SetUp]
        public void SetUp()
        {
            host = new GameObject("World Reference Test");
            visualizer = host.AddComponent<WorldReferenceVisualizer>();
            visualizer.Rebuild();
        }

        [TearDown]
        public void TearDown()
        {
            Object.DestroyImmediate(host);
        }

        [Test]
        public void CreatesOneGridAndThreeAxisRenderers()
        {
            Assert.That(visualizer.RendererCount, Is.EqualTo(4));
            Assert.That(visualizer.GridRenderer, Is.Not.Null);
            Assert.That(visualizer.XAxisLine, Is.Not.Null);
            Assert.That(visualizer.YAxisLine, Is.Not.Null);
            Assert.That(visualizer.ZAxisLine, Is.Not.Null);
        }

        [Test]
        public void AxesPointAlongPositiveUnityDirections()
        {
            AssertVector(visualizer.XAxisLine.GetPosition(1), Vector3.right * 2f);
            AssertVector(visualizer.YAxisLine.GetPosition(1), Vector3.up * 2f);
            AssertVector(visualizer.ZAxisLine.GetPosition(1), Vector3.forward * 2f);
            AssertColor(
                visualizer.XAxisLine.startColor,
                new Color(1f, 0.2f, 0.2f, 1f));
            AssertColor(
                visualizer.YAxisLine.startColor,
                new Color(0.2f, 1f, 0.2f, 1f));
            AssertColor(
                visualizer.ZAxisLine.startColor,
                new Color(0.2f, 0.45f, 1f, 1f));
        }

        [Test]
        public void VisibilityToggleAffectsEveryRendererWithoutRecreatingGeometry()
        {
            int[] rendererIds = host.GetComponentsInChildren<Renderer>(true)
                .Select(renderer => renderer.GetInstanceID())
                .OrderBy(id => id)
                .ToArray();

            visualizer.ShowReference = false;

            Assert.That(
                host.GetComponentsInChildren<Renderer>(true).All(renderer => !renderer.enabled),
                Is.True);
            Assert.That(
                host.GetComponentsInChildren<Renderer>(true)
                    .Select(renderer => renderer.GetInstanceID())
                    .OrderBy(id => id),
                Is.EqualTo(rendererIds));

            visualizer.ShowReference = true;
            Assert.That(
                host.GetComponentsInChildren<Renderer>(true).All(renderer => renderer.enabled),
                Is.True);
        }

        private static void AssertVector(Vector3 actual, Vector3 expected)
        {
            Assert.That(Vector3.Distance(actual, expected), Is.LessThan(0.0001f));
        }

        private static void AssertColor(Color actual, Color expected)
        {
            Assert.That(actual.r, Is.EqualTo(expected.r).Within(1f / 255f));
            Assert.That(actual.g, Is.EqualTo(expected.g).Within(1f / 255f));
            Assert.That(actual.b, Is.EqualTo(expected.b).Within(1f / 255f));
            Assert.That(actual.a, Is.EqualTo(expected.a).Within(1f / 255f));
        }
    }
}
