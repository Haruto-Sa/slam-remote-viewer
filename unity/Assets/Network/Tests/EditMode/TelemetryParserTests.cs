using NUnit.Framework;

namespace Slam.RemoteViewer.Network.Tests
{
    public sealed class TelemetryParserTests
    {
        internal const string UnitySettings =
            "{\"v\":1,\"session\":\"test-session\",\"unit\":\"m\",\"frame\":\"unity_world\"," +
            "\"pose_convention\":\"Twc\",\"quaternion\":\"xyzw\",\"camera\":{" +
            "\"type\":\"pc\",\"id\":\"builtin_0\",\"width\":1280,\"height\":720,\"fps\":30}," +
            "\"pointcloud_mode\":\"delta\"}";

        internal const string Pose =
            "{\"v\":1,\"session\":\"test-session\",\"seq\":7,\"t\":0.25," +
            "\"p\":[1.0,-2.0,3.0],\"q\":[-0.1,0.2,-0.3,0.9],\"state\":\"tracking\"}";

        internal const string PointCloud =
            "{\"v\":1,\"session\":\"test-session\",\"seq\":8,\"t\":0.5," +
            "\"add\":[[1001,1.0,-2.0,3.0]],\"update\":[[1002,4.0,-5.0,6.0]],\"remove\":[1003]}";

        [Test]
        public void ParsesUnitySettings()
        {
            bool parsed = TelemetryParser.TryParse(
                TelemetryTopics.Settings,
                UnitySettings,
                out ITelemetryMessage message,
                out string error);

            Assert.That(parsed, Is.True, error);
            var settings = message as SettingsMessage;
            Assert.That(settings, Is.Not.Null);
            Assert.That(settings.Frame, Is.EqualTo("unity_world"));
            Assert.That(settings.Camera.Id, Is.EqualTo("builtin_0"));
            Assert.That(settings.Sequence, Is.Null);
        }

        [Test]
        public void RejectsSlamWorldSettings()
        {
            string slamSettings = UnitySettings.Replace("unity_world", "slam_world");

            bool parsed = TelemetryParser.TryParse(
                TelemetryTopics.Settings,
                slamSettings,
                out _,
                out string error);

            Assert.That(parsed, Is.False);
            Assert.That(error, Does.Contain("Unity Protocol v1"));
        }

        [Test]
        public void ParsesImmutablePose()
        {
            bool parsed = TelemetryParser.TryParse(
                TelemetryTopics.Pose,
                Pose,
                out ITelemetryMessage message,
                out string error);

            Assert.That(parsed, Is.True, error);
            var pose = message as PoseMessage;
            Assert.That(pose, Is.Not.Null);
            Assert.That(pose.SequenceNumber, Is.EqualTo(7));
            Assert.That(pose.Position, Is.EqualTo(new[] { 1.0, -2.0, 3.0 }));
            Assert.That(pose.OrientationXyzw, Is.EqualTo(new[] { -0.1, 0.2, -0.3, 0.9 }));
            Assert.That(pose.State, Is.EqualTo(PoseTrackingState.Tracking));
        }

        [Test]
        public void ParsesPointCloudEntries()
        {
            bool parsed = TelemetryParser.TryParse(
                TelemetryTopics.PointCloud,
                PointCloud,
                out ITelemetryMessage message,
                out string error);

            Assert.That(parsed, Is.True, error);
            var pointCloud = message as PointCloudMessage;
            Assert.That(pointCloud, Is.Not.Null);
            Assert.That(pointCloud.Add[0].Id, Is.EqualTo(1001));
            Assert.That(pointCloud.Add[0].Y, Is.EqualTo(-2.0));
            Assert.That(pointCloud.Update[0].Id, Is.EqualTo(1002));
            Assert.That(pointCloud.Remove, Is.EqualTo(new ulong[] { 1003 }));
        }

        [TestCase("slam/v1/future", "{}")]
        [TestCase(TelemetryTopics.Pose, "{not-json}")]
        [TestCase(TelemetryTopics.Pose, "{\"v\":1}")]
        public void RejectsMalformedTelemetry(string topic, string payload)
        {
            Assert.That(TelemetryParser.TryParse(topic, payload, out _, out _), Is.False);
        }
    }
}
