using NUnit.Framework;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class OrbitCameraControllerTests
    {
        [Test]
        public void AppliesOrbitUsingUnscaledDeltaTimeProvidedByCaller()
        {
            var gameObject = new GameObject("Orbit Camera Test");
            try
            {
                gameObject.transform.position = new Vector3(0f, 1f, -10f);
                var controller = gameObject.AddComponent<OrbitCameraController>();

                bool changed = controller.ApplyCommand(
                    new OrbitCameraCommand(Vector2.right, Vector2.zero, 0f),
                    0.5f);

                Assert.That(changed, Is.True);
                Assert.That(controller.State.YawDegrees, Is.EqualTo(90f).Within(0.0001f));
                Assert.That(
                    Vector3.Distance(gameObject.transform.position, new Vector3(-10f, 1f, 0f)),
                    Is.LessThan(0.0001f));
            }
            finally
            {
                Object.DestroyImmediate(gameObject);
            }
        }

        [Test]
        public void DisabledControllerIgnoresCommands()
        {
            var gameObject = new GameObject("Orbit Camera Test");
            try
            {
                gameObject.transform.position = new Vector3(0f, 1f, -10f);
                var controller = gameObject.AddComponent<OrbitCameraController>();
                Vector3 initialPosition = gameObject.transform.position;
                controller.enabled = false;

                bool changed = controller.ApplyCommand(
                    new OrbitCameraCommand(new Vector2(1f, 1f), Vector2.zero, 1f),
                    1f);

                Assert.That(changed, Is.False);
                Assert.That(Vector3.Distance(gameObject.transform.position, initialPosition), Is.Zero);
            }
            finally
            {
                Object.DestroyImmediate(gameObject);
            }
        }

        [Test]
        public void AppliesCardinalPresetToCameraTransform()
        {
            var gameObject = new GameObject("Orbit Camera Test");
            try
            {
                gameObject.transform.position = new Vector3(0f, 1f, -10f);
                var controller = gameObject.AddComponent<OrbitCameraController>();

                bool changed = controller.ApplyCommand(
                    new OrbitCameraCommand(
                        Vector2.zero,
                        Vector2.zero,
                        0f,
                        viewPreset: OrbitCameraViewPreset.Right),
                    0f);

                Assert.That(changed, Is.True);
                Assert.That(
                    Vector3.Distance(
                        gameObject.transform.position,
                        new Vector3(10f, 1f, 0f)),
                    Is.LessThan(0.0001f));
                Assert.That(
                    Vector3.Distance(
                        gameObject.transform.forward,
                        Vector3.left),
                    Is.LessThan(0.0001f));
            }
            finally
            {
                Object.DestroyImmediate(gameObject);
            }
        }

        [Test]
        public void FramesVisibleTelemetryAndExcludesHiddenLayers()
        {
            var gameObject = new GameObject("Orbit Camera Test");
            try
            {
                gameObject.transform.position = new Vector3(0f, 1f, -10f);
                var camera = gameObject.AddComponent<Camera>();
                camera.fieldOfView = 60f;
                camera.aspect = 1f;
                var trajectory = gameObject.AddComponent<CameraTrajectoryVisualizer>();
                var pointCloud = gameObject.AddComponent<PointCloudVisualizer>();
                var controller = gameObject.AddComponent<OrbitCameraController>();

                trajectory.HandleMessage(TrajectoryHistoryTests.Settings("session-a"));
                trajectory.HandleMessage(TrajectoryHistoryTests.Pose("session-a", -100, 0, 0));
                trajectory.HandleMessage(TrajectoryHistoryTests.Pose("session-a", 100, 0, 0));
                trajectory.SetVisible(false);
                pointCloud.HandleMessage(PointCloudStateTests.Settings("session-a"));
                pointCloud.HandleMessage(PointCloudStateTests.Delta(
                    "session-a",
                    add: new System.Collections.Generic.IList<double>[]
                    {
                        new double[] { 1, 10, 2, 3 }
                    }));

                bool changed = controller.FrameVisibleTelemetry();

                Assert.That(changed, Is.True);
                Assert.That(
                    Vector3.Distance(
                        controller.State.FocusPoint,
                        new Vector3(10f, 2f, 3f)),
                    Is.LessThan(0.0001f));

                trajectory.SetVisible(true);
                controller.FrameVisibleTelemetry();
                Assert.That(controller.State.FocusPoint.x, Is.EqualTo(0f).Within(0.0001f));
            }
            finally
            {
                Object.DestroyImmediate(gameObject);
            }
        }

        [Test]
        public void FrameVisibleTelemetryDoesNothingWhenSourcesAreEmpty()
        {
            var gameObject = new GameObject("Orbit Camera Test");
            try
            {
                gameObject.transform.position = new Vector3(0f, 1f, -10f);
                gameObject.AddComponent<Camera>();
                var controller = gameObject.AddComponent<OrbitCameraController>();
                Vector3 position = gameObject.transform.position;

                Assert.That(controller.FrameVisibleTelemetry(), Is.False);
                Assert.That(Vector3.Distance(gameObject.transform.position, position), Is.Zero);
            }
            finally
            {
                Object.DestroyImmediate(gameObject);
            }
        }
    }
}
