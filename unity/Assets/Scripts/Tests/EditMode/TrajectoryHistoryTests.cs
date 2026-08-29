using System;
using NUnit.Framework;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class TrajectoryHistoryTests
    {
        [Test]
        public void RetainsTrackingPositionsInArrivalOrder()
        {
            var history = new TrajectoryHistory(10, 0f);
            history.HandleMessage(Settings("session-a"));

            Assert.That(history.HandleMessage(Pose("session-a", 1, 0, 0)), Is.True);
            Assert.That(history.HandleMessage(Pose("session-a", 2, 0, 0)), Is.True);

            Assert.That(history.PointCount, Is.EqualTo(2));
            Assert.That(history.SegmentCount, Is.EqualTo(1));
            AssertVector(history.GetSegment(0)[0], new Vector3(1, 0, 0));
            AssertVector(history.GetSegment(0)[1], new Vector3(2, 0, 0));
        }

        [Test]
        public void IgnoresPointsCloserThanMinimumDistance()
        {
            var history = new TrajectoryHistory(10, 1f);
            history.HandleMessage(Settings("session-a"));

            history.HandleMessage(Pose("session-a", 0, 0, 0));
            Assert.That(history.HandleMessage(Pose("session-a", 0.5, 0, 0)), Is.False);
            Assert.That(history.HandleMessage(Pose("session-a", 1, 0, 0)), Is.True);

            Assert.That(history.PointCount, Is.EqualTo(2));
            AssertVector(history.GetSegment(0)[1], new Vector3(1, 0, 0));
        }

        [Test]
        public void DropsOldestPointsWhenCapacityIsExceeded()
        {
            var history = new TrajectoryHistory(2, 0f);
            history.HandleMessage(Settings("session-a"));

            history.HandleMessage(Pose("session-a", 1, 0, 0));
            history.HandleMessage(Pose("session-a", 2, 0, 0));
            history.HandleMessage(Pose("session-a", 3, 0, 0));

            Assert.That(history.PointCount, Is.EqualTo(2));
            AssertVector(history.GetSegment(0)[0], new Vector3(2, 0, 0));
            AssertVector(history.GetSegment(0)[1], new Vector3(3, 0, 0));
        }

        [Test]
        public void NonTrackingPoseStartsANewSegment()
        {
            var history = new TrajectoryHistory(10, 0f);
            history.HandleMessage(Settings("session-a"));
            history.HandleMessage(Pose("session-a", 0, 0, 0));

            history.HandleMessage(Pose("session-a", 1, 0, 0, PoseTrackingState.Lost));
            history.HandleMessage(Pose("session-a", 2, 0, 0));

            Assert.That(history.PointCount, Is.EqualTo(2));
            Assert.That(history.SegmentCount, Is.EqualTo(2));
            AssertVector(history.GetSegment(0)[0], Vector3.zero);
            AssertVector(history.GetSegment(1)[0], new Vector3(2, 0, 0));
        }

        [Test]
        public void SessionChangeClearsHistoryAndRejectsOldSession()
        {
            var history = new TrajectoryHistory(10, 0f);
            history.HandleMessage(Settings("session-a"));
            history.HandleMessage(Pose("session-a", 1, 0, 0));

            Assert.That(history.HandleMessage(Settings("session-b")), Is.True);
            Assert.That(history.PointCount, Is.Zero);
            Assert.That(history.SegmentCount, Is.Zero);
            Assert.That(history.ActiveSession, Is.EqualTo("session-b"));
            Assert.That(history.HandleMessage(Pose("session-a", 2, 0, 0)), Is.False);
            Assert.That(history.PointCount, Is.Zero);
        }

        [Test]
        public void IgnoresPoseUntilSettingsArrive()
        {
            var history = new TrajectoryHistory(10, 0f);

            Assert.That(history.HandleMessage(Pose("session-a", 1, 0, 0)), Is.False);
            Assert.That(history.PointCount, Is.Zero);
        }

        [Test]
        public void RejectsInvalidConfiguration()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new TrajectoryHistory(0, 0f));
            Assert.Throws<ArgumentOutOfRangeException>(() => new TrajectoryHistory(1, -1f));
            Assert.Throws<ArgumentOutOfRangeException>(() => new TrajectoryHistory(1, float.NaN));
        }

        internal static SettingsMessage Settings(string session)
        {
            return new SettingsMessage(
                1,
                session,
                "m",
                "unity_world",
                "Twc",
                "xyzw",
                new CameraSettings("pc", "test-camera", 1280, 720, 30),
                "delta");
        }

        internal static PoseMessage Pose(
            string session,
            double x,
            double y,
            double z,
            PoseTrackingState state = PoseTrackingState.Tracking)
        {
            return new PoseMessage(
                1,
                session,
                0,
                0,
                new[] { x, y, z },
                new[] { 0.0, 0.0, 0.0, 1.0 },
                state);
        }

        private static void AssertVector(Vector3 actual, Vector3 expected)
        {
            Assert.That(Vector3.Distance(actual, expected), Is.LessThan(0.0001f));
        }
    }
}
