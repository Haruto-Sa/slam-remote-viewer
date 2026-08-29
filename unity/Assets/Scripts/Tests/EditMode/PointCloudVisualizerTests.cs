using NUnit.Framework;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class PointCloudVisualizerTests
    {
        private GameObject host;
        private PointCloudVisualizer visualizer;

        [SetUp]
        public void SetUp()
        {
            host = new GameObject("Point cloud visualizer test");
            visualizer = host.AddComponent<PointCloudVisualizer>();
        }

        [TearDown]
        public void TearDown()
        {
            Object.DestroyImmediate(host);
        }

        [Test]
        public void CreatesOneBatchedParticleSystemForAllPoints()
        {
            visualizer.HandleMessage(PointCloudStateTests.Settings("session-a"));
            visualizer.HandleMessage(PointCloudStateTests.Delta(
                "session-a",
                add: new System.Collections.Generic.IList<double>[]
                {
                    new double[] { 1, 1, 2, 3 },
                    new double[] { 2, 4, 5, 6 }
                }));

            Assert.That(visualizer.ParticleSystemTarget, Is.Not.Null);
            Assert.That(host.GetComponentsInChildren<ParticleSystem>(true), Has.Length.EqualTo(1));
            Assert.That(visualizer.PointCount, Is.EqualTo(2));
            Assert.That(visualizer.RenderedPointCount, Is.EqualTo(2));
            Assert.That(visualizer.ParticleSystemTarget.main.maxParticles, Is.EqualTo(2));
        }

        [Test]
        public void NoOpDeltaDoesNotRebuildParticles()
        {
            visualizer.HandleMessage(PointCloudStateTests.Settings("session-a"));
            visualizer.HandleMessage(PointCloudStateTests.Delta(
                "session-a",
                remove: new ulong[] { 999 }));
            long revision = visualizer.RenderRevision;

            visualizer.HandleMessage(PointCloudStateTests.Delta(
                "session-a",
                remove: new ulong[] { 999 }));

            Assert.That(visualizer.RenderRevision, Is.EqualTo(revision));
        }

        [Test]
        public void SessionChangeClearsRenderedParticles()
        {
            visualizer.HandleMessage(PointCloudStateTests.Settings("session-a"));
            visualizer.HandleMessage(PointCloudStateTests.Delta(
                "session-a",
                add: new System.Collections.Generic.IList<double>[]
                {
                    new double[] { 1, 1, 2, 3 }
                }));

            visualizer.HandleMessage(PointCloudStateTests.Settings("session-b"));

            Assert.That(visualizer.PointCount, Is.Zero);
            Assert.That(visualizer.RenderedPointCount, Is.Zero);
            Assert.That(visualizer.ActiveSession, Is.EqualTo("session-b"));
        }
    }
}
