using NUnit.Framework;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class CameraTrajectoryVisualizerTests
    {
        private GameObject host;
        private CameraTrajectoryVisualizer visualizer;

        [SetUp]
        public void SetUp()
        {
            host = new GameObject("Camera trajectory visualizer test");
            visualizer = host.AddComponent<CameraTrajectoryVisualizer>();
        }

        [TearDown]
        public void TearDown()
        {
            Object.DestroyImmediate(host);
        }

        [Test]
        public void CreatesDefaultRootAndRendersTrackingSegment()
        {
            visualizer.HandleMessage(TrajectoryHistoryTests.Settings("session-a"));
            visualizer.HandleMessage(TrajectoryHistoryTests.Pose("session-a", 0, 0, 0));
            visualizer.HandleMessage(TrajectoryHistoryTests.Pose("session-a", 1, 0, 0));

            Assert.That(visualizer.TrajectoryRoot, Is.Not.Null);
            Assert.That(visualizer.PointCount, Is.EqualTo(2));
            Assert.That(visualizer.SegmentCount, Is.EqualTo(1));
            Assert.That(visualizer.SegmentRenderers, Has.Count.EqualTo(1));
            Assert.That(visualizer.SegmentRenderers[0].positionCount, Is.EqualTo(2));
            Assert.That(
                Vector3.Distance(
                    visualizer.SegmentRenderers[0].GetPosition(1),
                    new Vector3(1, 0, 0)),
                Is.LessThan(0.0001f));
        }

        [Test]
        public void SessionChangeRemovesRenderedSegments()
        {
            visualizer.HandleMessage(TrajectoryHistoryTests.Settings("session-a"));
            visualizer.HandleMessage(TrajectoryHistoryTests.Pose("session-a", 0, 0, 0));

            visualizer.HandleMessage(TrajectoryHistoryTests.Settings("session-b"));

            Assert.That(visualizer.PointCount, Is.Zero);
            Assert.That(visualizer.SegmentCount, Is.Zero);
            Assert.That(visualizer.SegmentRenderers, Is.Empty);
            Assert.That(visualizer.ActiveSession, Is.EqualTo("session-b"));
        }
    }
}
