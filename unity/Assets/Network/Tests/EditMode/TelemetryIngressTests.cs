using NUnit.Framework;

namespace Slam.RemoteViewer.Network.Tests
{
    public sealed class TelemetryIngressTests
    {
        [Test]
        public void RequiresSnapshotModeBeforeAcceptingSnapshots()
        {
            var ingress = new TelemetryIngress(new BoundedMessageQueue<ITelemetryMessage>(4));
            ingress.TryAccept(TelemetryTopics.Settings, TelemetryParserTests.UnitySettings);
            Assert.That(ingress.TryAccept(TelemetryTopics.PointCloud, TelemetryParserTests.Snapshot), Is.False);
            ingress.TryAccept(TelemetryTopics.Settings, TelemetryParserTests.UnitySettings.Replace("\"delta\"", "\"delta_snapshot\""));
            Assert.That(ingress.TryAccept(TelemetryTopics.PointCloud, TelemetryParserTests.Snapshot), Is.True);
            ingress.TryAccept(TelemetryTopics.Settings, TelemetryParserTests.UnitySettings.Replace("\"delta\"", "\"delta_snapshot\""));
            Assert.That(ingress.TryAccept(TelemetryTopics.PointCloud, TelemetryParserTests.Snapshot), Is.False);
            Assert.That(ingress.TryAccept(TelemetryTopics.PointCloud, TelemetryParserTests.Snapshot.Replace("\"seq\":10", "\"seq\":11")), Is.True);
        }
        [Test]
        public void RejectsTelemetryUntilMatchingSettingsArrive()
        {
            var queue = new BoundedMessageQueue<ITelemetryMessage>(4);
            var ingress = new TelemetryIngress(queue);

            Assert.That(ingress.TryAccept(TelemetryTopics.Pose, TelemetryParserTests.Pose), Is.False);
            Assert.That(ingress.RejectedCount, Is.EqualTo(1));

            Assert.That(ingress.TryAccept(TelemetryTopics.Settings, TelemetryParserTests.UnitySettings), Is.True);
            Assert.That(ingress.TryAccept(TelemetryTopics.Pose, TelemetryParserTests.Pose), Is.True);

            Assert.That(ingress.AcceptedCount, Is.EqualTo(2));
            Assert.That(queue.Count, Is.EqualTo(2));
        }

        [Test]
        public void RejectsMismatchedSession()
        {
            var queue = new BoundedMessageQueue<ITelemetryMessage>(4);
            var ingress = new TelemetryIngress(queue);
            string otherSessionPose = TelemetryParserTests.Pose.Replace("test-session", "other-session");

            ingress.TryAccept(TelemetryTopics.Settings, TelemetryParserTests.UnitySettings);

            Assert.That(ingress.TryAccept(TelemetryTopics.Pose, otherSessionPose), Is.False);
            Assert.That(ingress.LastRejectionReason, Does.Contain("matching"));
        }

        [Test]
        public void ExposesQueueOverflowCount()
        {
            var queue = new BoundedMessageQueue<ITelemetryMessage>(1);
            var ingress = new TelemetryIngress(queue);

            ingress.TryAccept(TelemetryTopics.Settings, TelemetryParserTests.UnitySettings);
            ingress.TryAccept(TelemetryTopics.Pose, TelemetryParserTests.Pose);

            Assert.That(ingress.DroppedCount, Is.EqualTo(1));
            Assert.That(queue.TryDequeue(out ITelemetryMessage remaining), Is.True);
            Assert.That(remaining.Topic, Is.EqualTo(TelemetryTopics.Pose));
        }
    }
}
