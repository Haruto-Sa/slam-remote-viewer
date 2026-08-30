using NUnit.Framework;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class VisualizationVisibilityControllerTests
    {
        [Test]
        public void AppliesStateToEveryAvailableVisualizerAndRestoresDefaults()
        {
            var host = new GameObject("Visibility Controller Test");
            try
            {
                var pose = host.AddComponent<CameraPoseVisualizer>();
                var trajectory = host.AddComponent<CameraTrajectoryVisualizer>();
                var pointCloud = host.AddComponent<PointCloudVisualizer>();
                var worldReference = host.AddComponent<WorldReferenceVisualizer>();
                var diagnostics = host.AddComponent<TelemetryDiagnosticsOverlay>();
                var controller = host.AddComponent<VisualizationVisibilityController>();

                controller.ApplyCommand(
                    VisualizationVisibilityCommand.ToggleCameraPose |
                    VisualizationVisibilityCommand.ToggleTrajectory |
                    VisualizationVisibilityCommand.TogglePointCloud |
                    VisualizationVisibilityCommand.ToggleWorldReference |
                    VisualizationVisibilityCommand.ToggleDiagnostics);

                Assert.That(pose.IsVisible, Is.False);
                Assert.That(trajectory.IsVisible, Is.False);
                Assert.That(pointCloud.IsVisible, Is.False);
                Assert.That(worldReference.IsVisible, Is.False);
                Assert.That(diagnostics.IsVisible, Is.False);

                controller.ApplyCommand(VisualizationVisibilityCommand.RestoreDefaults);

                Assert.That(pose.IsVisible, Is.True);
                Assert.That(trajectory.IsVisible, Is.True);
                Assert.That(pointCloud.IsVisible, Is.True);
                Assert.That(worldReference.IsVisible, Is.True);
                Assert.That(diagnostics.IsVisible, Is.True);
            }
            finally
            {
                Object.DestroyImmediate(host);
            }
        }

        [Test]
        public void MissingOptionalVisualizersDoNotThrow()
        {
            var host = new GameObject("Visibility Controller Test");
            try
            {
                var controller = host.AddComponent<VisualizationVisibilityController>();

                Assert.DoesNotThrow(() => controller.ApplyCommand(
                    VisualizationVisibilityCommand.ToggleCameraPose |
                    VisualizationVisibilityCommand.ToggleTrajectory |
                    VisualizationVisibilityCommand.TogglePointCloud |
                    VisualizationVisibilityCommand.ToggleWorldReference |
                    VisualizationVisibilityCommand.ToggleDiagnostics));
            }
            finally
            {
                Object.DestroyImmediate(host);
            }
        }
    }
}
