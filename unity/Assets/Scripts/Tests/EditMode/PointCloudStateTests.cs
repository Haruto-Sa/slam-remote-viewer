using System.Collections.Generic;
using NUnit.Framework;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class PointCloudStateTests
    {
        [Test]
        public void AppliesAddUpdateAndRemoveOperations()
        {
            var state = new PointCloudState();
            state.HandleMessage(Settings("session-a"));

            Assert.That(state.HandleMessage(Delta(
                "session-a",
                Add(Point(1, 1, 2, 3), Point(2, 4, 5, 6)))), Is.True);
            AssertPoint(state, 1, new Vector3(1, 2, 3));
            AssertPoint(state, 2, new Vector3(4, 5, 6));

            Assert.That(state.HandleMessage(Delta(
                "session-a",
                update: Add(Point(1, 7, 8, 9)))), Is.True);
            AssertPoint(state, 1, new Vector3(7, 8, 9));

            Assert.That(state.HandleMessage(Delta(
                "session-a",
                remove: new ulong[] { 2 })), Is.True);
            Assert.That(state.TryGetPoint(2, out _), Is.False);
        }

        [Test]
        public void AppliesMixedDeltaInRemoveUpdateAddOrder()
        {
            var state = new PointCloudState();
            state.HandleMessage(Settings("session-a"));
            state.HandleMessage(Delta("session-a", Add(Point(5, 1, 0, 0))));

            state.HandleMessage(Delta(
                "session-a",
                add: Add(Point(5, 3, 0, 0)),
                update: Add(Point(5, 2, 0, 0)),
                remove: new ulong[] { 5 }));

            AssertPoint(state, 5, new Vector3(3, 0, 0));
            Assert.That(state.PointCount, Is.EqualTo(1));
        }

        [Test]
        public void UnknownUpdateAddsAndDuplicateAddUpdates()
        {
            var state = new PointCloudState();
            state.HandleMessage(Settings("session-a"));

            state.HandleMessage(Delta("session-a", update: Add(Point(9, 1, 0, 0))));
            AssertPoint(state, 9, new Vector3(1, 0, 0));

            state.HandleMessage(Delta("session-a", add: Add(Point(9, 2, 0, 0))));
            AssertPoint(state, 9, new Vector3(2, 0, 0));
            Assert.That(state.PointCount, Is.EqualTo(1));
        }

        [Test]
        public void NoOpDeltaDoesNotChangeRevision()
        {
            var state = new PointCloudState();
            state.HandleMessage(Settings("session-a"));
            state.HandleMessage(Delta("session-a", add: Add(Point(1, 1, 2, 3))));
            long revision = state.Revision;

            Assert.That(state.HandleMessage(Delta(
                "session-a",
                add: Add(Point(1, 1, 2, 3)),
                remove: new ulong[] { 999 })), Is.False);
            Assert.That(state.Revision, Is.EqualTo(revision));
        }

        [Test]
        public void SessionChangeClearsPointsAndRejectsOldSession()
        {
            var state = new PointCloudState();
            state.HandleMessage(Settings("session-a"));
            state.HandleMessage(Delta("session-a", add: Add(Point(1, 1, 2, 3))));

            Assert.That(state.HandleMessage(Settings("session-b")), Is.True);
            Assert.That(state.PointCount, Is.Zero);
            Assert.That(state.ActiveSession, Is.EqualTo("session-b"));
            Assert.That(state.HandleMessage(Delta("session-a", add: Add(Point(2, 2, 3, 4)))), Is.False);
            Assert.That(state.PointCount, Is.Zero);
        }

        [Test]
        public void IgnoresDeltaUntilSettingsArrive()
        {
            var state = new PointCloudState();

            Assert.That(state.HandleMessage(Delta("session-a", add: Add(Point(1, 1, 2, 3)))), Is.False);
            Assert.That(state.PointCount, Is.Zero);
        }

        [Test]
        public void CopiesPositionsInPointIdOrder()
        {
            var state = new PointCloudState();
            var positions = new List<Vector3>();
            state.HandleMessage(Settings("session-a"));
            state.HandleMessage(Delta(
                "session-a",
                add: Add(Point(20, 2, 0, 0), Point(10, 1, 0, 0))));

            state.CopyOrderedPositions(positions);

            Assert.That(positions, Has.Count.EqualTo(2));
            Assert.That(positions[0], Is.EqualTo(new Vector3(1, 0, 0)));
            Assert.That(positions[1], Is.EqualTo(new Vector3(2, 0, 0)));
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

        internal static PointCloudMessage Delta(
            string session,
            IList<IList<double>> add = null,
            IList<IList<double>> update = null,
            IList<ulong> remove = null)
        {
            return new PointCloudMessage(
                1,
                session,
                0,
                0,
                add ?? Add(),
                update ?? Add(),
                remove ?? new ulong[0]);
        }

        private static IList<IList<double>> Add(params IList<double>[] points)
        {
            return points;
        }

        private static IList<double> Point(ulong id, double x, double y, double z)
        {
            return new[] { (double)id, x, y, z };
        }

        private static void AssertPoint(PointCloudState state, ulong id, Vector3 expected)
        {
            Assert.That(state.TryGetPoint(id, out Vector3 actual), Is.True);
            Assert.That(Vector3.Distance(actual, expected), Is.LessThan(0.0001f));
        }
    }
}
