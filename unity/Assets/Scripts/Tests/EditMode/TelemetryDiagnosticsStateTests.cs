using System;
using NUnit.Framework;
using Slam.RemoteViewer.Network;

namespace Slam.RemoteViewer.Tests
{
    public sealed class TelemetryDiagnosticsStateTests
    {
        [Test]
        public void TransitionsFromStoppedToWaitingReceivingAndStale()
        {
            var state = new TelemetryDiagnosticsState(2.0);

            Assert.That(state.GetStatus(0.0), Is.EqualTo(TelemetryHealthStatus.Stopped));
            state.Start("tcp://127.0.0.1:5556");
            Assert.That(state.GetStatus(10.0), Is.EqualTo(TelemetryHealthStatus.Waiting));

            Assert.That(state.Observe(Settings("session-a"), 10.0), Is.True);
            Assert.That(state.GetStatus(11.999), Is.EqualTo(TelemetryHealthStatus.Receiving));
            Assert.That(state.GetStatus(12.0), Is.EqualTo(TelemetryHealthStatus.Stale));

            state.SetRunning(false);
            Assert.That(state.GetStatus(20.0), Is.EqualTo(TelemetryHealthStatus.Stopped));
        }

        [Test]
        public void NewSettingsResetSessionTrackingStateAndMessageAge()
        {
            var state = new TelemetryDiagnosticsState(2.0);
            state.Start("endpoint");
            state.Observe(Settings("session-a"), 1.0);
            state.Observe(Pose("session-a", PoseTrackingState.Tracking), 2.0);

            Assert.That(state.ActiveSession, Is.EqualTo("session-a"));
            Assert.That(state.TrackingState, Is.EqualTo(PoseTrackingState.Tracking));

            state.Observe(Settings("session-b"), 5.0);

            Assert.That(state.ActiveSession, Is.EqualTo("session-b"));
            Assert.That(state.TrackingState, Is.Null);
            Assert.That(state.GetLatestMessageAgeSeconds(5.5), Is.EqualTo(0.5).Within(1e-9));
        }

        [Test]
        public void IgnoresMessagesBeforeSettingsOrForAnotherSession()
        {
            var state = new TelemetryDiagnosticsState(2.0);
            state.Start("endpoint");

            Assert.That(state.Observe(Pose("session-a", PoseTrackingState.Tracking), 1.0), Is.False);
            Assert.That(state.HasReceivedMessage, Is.False);

            state.Observe(Settings("session-a"), 2.0);
            Assert.That(
                state.Observe(Pose("session-b", PoseTrackingState.Lost), 3.0),
                Is.False);
            Assert.That(state.TrackingState, Is.Null);
            Assert.That(state.GetLatestMessageAgeSeconds(3.0), Is.EqualTo(1.0));
        }

        [Test]
        public void StoresCountersAndOptionalVisualizerCounts()
        {
            var state = new TelemetryDiagnosticsState(2.0);

            state.UpdateMetrics(10, 2, 3, 4, 50, 500, 1, "SocketException", "bad JSON");

            Assert.That(state.AcceptedCount, Is.EqualTo(10));
            Assert.That(state.RejectedCount, Is.EqualTo(2));
            Assert.That(state.DroppedCount, Is.EqualTo(3));
            Assert.That(state.QueueCount, Is.EqualTo(4));
            Assert.That(state.TrajectoryPointCount, Is.EqualTo(50));
            Assert.That(state.PointCloudPointCount, Is.EqualTo(500));
            Assert.That(state.FaultCount, Is.EqualTo(1));
            Assert.That(state.LastFault, Is.EqualTo("SocketException"));
            Assert.That(state.LastRejectionReason, Is.EqualTo("bad JSON"));

            state.UpdateMetrics(-1, -2, -3, -4, null, null, -5, null, null);
            Assert.That(state.AcceptedCount, Is.Zero);
            Assert.That(state.QueueCount, Is.Zero);
            Assert.That(state.TrajectoryPointCount, Is.Null);
            Assert.That(state.PointCloudPointCount, Is.Null);
        }

        [Test]
        public void RejectsInvalidTimeoutsAndTimes()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new TelemetryDiagnosticsState(0.0));
            Assert.Throws<ArgumentOutOfRangeException>(() => new TelemetryDiagnosticsState(double.NaN));

            var state = new TelemetryDiagnosticsState(1.0);
            Assert.Throws<ArgumentOutOfRangeException>(() => state.GetStatus(double.PositiveInfinity));
            Assert.Throws<ArgumentOutOfRangeException>(() => state.Observe(Settings("session"), double.NaN));
        }

        private static SettingsMessage Settings(string session)
        {
            return new SettingsMessage(
                1,
                session,
                "m",
                "unity_world",
                "Twc",
                "xyzw",
                new CameraSettings("mock", "camera", 640, 480, 30),
                "delta");
        }

        private static PoseMessage Pose(string session, PoseTrackingState trackingState)
        {
            return new PoseMessage(
                1,
                session,
                0,
                0.0,
                new[] { 0.0, 0.0, 0.0 },
                new[] { 0.0, 0.0, 0.0, 1.0 },
                trackingState);
        }
    }
}
