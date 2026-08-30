using NUnit.Framework;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class CameraPoseVisualizerTests
    {
        private GameObject host;
        private CameraPoseVisualizer visualizer;

        [SetUp]
        public void SetUp()
        {
            host = new GameObject("Camera pose visualizer test");
            visualizer = host.AddComponent<CameraPoseVisualizer>();
        }

        [TearDown]
        public void TearDown()
        {
            Object.DestroyImmediate(host);
        }

        [Test]
        public void AppliesMatchingPoseToGeneratedCameraMarker()
        {
            visualizer.HandleMessage(Settings("session-a", 1280, 720));
            visualizer.HandleMessage(Pose("session-a", new[] { 1.0, 2.0, 3.0 }, new[] { 0.0, 0.0, 0.0, 1.0 }));

            Assert.That(visualizer.HasPose, Is.True);
            Assert.That(visualizer.ActiveSession, Is.EqualTo("session-a"));
            Assert.That(visualizer.TrackingState, Is.EqualTo(PoseTrackingState.Tracking));
            AssertVector(visualizer.CameraPoseTransform.position, new Vector3(1f, 2f, 3f));
            Assert.That(
                Quaternion.Angle(visualizer.CameraPoseTransform.rotation, Quaternion.identity),
                Is.LessThan(0.001f));
            Assert.That(visualizer.FrustumLine.enabled, Is.True);
        }

        [Test]
        public void ClearsPoseAndRejectsOldPoseWhenSessionChanges()
        {
            visualizer.HandleMessage(Settings("session-a", 1280, 720));
            visualizer.HandleMessage(Pose("session-a", new[] { 1.0, 2.0, 3.0 }, new[] { 0.0, 0.0, 0.0, 1.0 }));

            visualizer.HandleMessage(Settings("session-b", 640, 480));

            Assert.That(visualizer.HasPose, Is.False);
            Assert.That(visualizer.ActiveSession, Is.EqualTo("session-b"));
            Assert.That(visualizer.FrustumLine.enabled, Is.False);

            visualizer.HandleMessage(Pose("session-a", new[] { 9.0, 9.0, 9.0 }, new[] { 0.0, 0.0, 0.0, 1.0 }));

            Assert.That(visualizer.HasPose, Is.False);
            AssertVector(visualizer.CameraPoseTransform.position, Vector3.zero);
        }

        [Test]
        public void RebuildsFrustumFromSettingsAspectRatio()
        {
            visualizer.HandleMessage(Settings("wide-session", 200, 100));

            Assert.That(visualizer.AspectRatio, Is.EqualTo(2f).Within(0.0001f));
            Vector3 topLeft = visualizer.FrustumLine.GetPosition(1);
            Assert.That(Mathf.Abs(topLeft.x / topLeft.y), Is.EqualTo(2f).Within(0.0001f));
        }

        [Test]
        public void AppliesKnownNinetyDegreeYRotation()
        {
            double halfSqrt = System.Math.Sqrt(0.5);
            visualizer.HandleMessage(Settings("rotation-session", 1280, 720));
            visualizer.HandleMessage(Pose(
                "rotation-session",
                new[] { 0.0, 0.0, 0.0 },
                new[] { 0.0, halfSqrt, 0.0, halfSqrt }));

            Assert.That(
                Quaternion.Angle(
                    visualizer.CameraPoseTransform.rotation,
                    Quaternion.Euler(0f, 90f, 0f)),
                Is.LessThan(0.001f));
        }

        [Test]
        public void UsesDifferentColorsForTrackingAndLostStates()
        {
            visualizer.HandleMessage(Settings("state-session", 1280, 720));
            visualizer.HandleMessage(Pose(
                "state-session",
                new[] { 0.0, 0.0, 0.0 },
                new[] { 0.0, 0.0, 0.0, 1.0 },
                PoseTrackingState.Tracking));
            Color tracking = visualizer.FrustumLine.startColor;

            visualizer.HandleMessage(Pose(
                "state-session",
                new[] { 0.0, 0.0, 0.0 },
                new[] { 0.0, 0.0, 0.0, 1.0 },
                PoseTrackingState.Lost));

            Assert.That(visualizer.FrustumLine.startColor, Is.Not.EqualTo(tracking));
            Assert.That(visualizer.TrackingState, Is.EqualTo(PoseTrackingState.Lost));
        }

        [Test]
        public void IgnoresPoseUntilSettingsEstablishSession()
        {
            visualizer.HandleMessage(Pose("session-a", new[] { 1.0, 2.0, 3.0 }, new[] { 0.0, 0.0, 0.0, 1.0 }));

            Assert.That(visualizer.HasPose, Is.False);
            Assert.That(visualizer.FrustumLine.enabled, Is.False);
        }

        [Test]
        public void HiddenPoseStillRetainsLatestTelemetryAndReusesVisuals()
        {
            visualizer.HandleMessage(Settings("session-a", 1280, 720));
            visualizer.HandleMessage(Pose(
                "session-a",
                new[] { 1.0, 2.0, 3.0 },
                new[] { 0.0, 0.0, 0.0, 1.0 }));
            int frustumId = visualizer.FrustumLine.GetInstanceID();

            visualizer.SetVisible(false);
            visualizer.HandleMessage(Pose(
                "session-a",
                new[] { 4.0, 5.0, 6.0 },
                new[] { 0.0, 0.0, 0.0, 1.0 }));

            Assert.That(visualizer.IsVisible, Is.False);
            Assert.That(visualizer.HasPose, Is.True);
            AssertVector(visualizer.CameraPoseTransform.position, new Vector3(4f, 5f, 6f));
            Assert.That(visualizer.FrustumLine.enabled, Is.False);

            visualizer.SetVisible(true);
            Assert.That(visualizer.FrustumLine.enabled, Is.True);
            Assert.That(visualizer.FrustumLine.GetInstanceID(), Is.EqualTo(frustumId));
        }

        private static SettingsMessage Settings(string session, uint width, uint height)
        {
            return new SettingsMessage(
                1,
                session,
                "m",
                "unity_world",
                "Twc",
                "xyzw",
                new CameraSettings("pc", "test-camera", width, height, 30),
                "delta");
        }

        private static PoseMessage Pose(
            string session,
            double[] position,
            double[] orientation,
            PoseTrackingState state = PoseTrackingState.Tracking)
        {
            return new PoseMessage(
                1,
                session,
                0,
                0.0,
                position,
                orientation,
                state);
        }

        private static void AssertVector(Vector3 actual, Vector3 expected)
        {
            Assert.That(Vector3.Distance(actual, expected), Is.LessThan(0.0001f));
        }
    }
}
