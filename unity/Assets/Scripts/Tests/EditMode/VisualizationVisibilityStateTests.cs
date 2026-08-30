using System;
using NUnit.Framework;

namespace Slam.RemoteViewer.Tests
{
    public sealed class VisualizationVisibilityStateTests
    {
        [TestCase(VisualizationVisibilityCommand.ToggleCameraPose)]
        [TestCase(VisualizationVisibilityCommand.ToggleTrajectory)]
        [TestCase(VisualizationVisibilityCommand.TogglePointCloud)]
        [TestCase(VisualizationVisibilityCommand.ToggleWorldReference)]
        [TestCase(VisualizationVisibilityCommand.ToggleDiagnostics)]
        public void EachToggleChangesOnlyItsAssignedLayer(
            VisualizationVisibilityCommand command)
        {
            var state = new VisualizationVisibilityState(true, true, true, true, true);

            bool changed = state.Apply(command);

            Assert.That(changed, Is.True);
            Assert.That(
                state.CameraPoseVisible,
                Is.EqualTo(command != VisualizationVisibilityCommand.ToggleCameraPose));
            Assert.That(
                state.TrajectoryVisible,
                Is.EqualTo(command != VisualizationVisibilityCommand.ToggleTrajectory));
            Assert.That(
                state.PointCloudVisible,
                Is.EqualTo(command != VisualizationVisibilityCommand.TogglePointCloud));
            Assert.That(
                state.WorldReferenceVisible,
                Is.EqualTo(command != VisualizationVisibilityCommand.ToggleWorldReference));
            Assert.That(
                state.DiagnosticsVisible,
                Is.EqualTo(command != VisualizationVisibilityCommand.ToggleDiagnostics));
        }

        [Test]
        public void SupportsMultipleTogglesInOneCommand()
        {
            var state = new VisualizationVisibilityState(true, true, true, true, true);

            state.Apply(
                VisualizationVisibilityCommand.ToggleCameraPose |
                VisualizationVisibilityCommand.TogglePointCloud);

            Assert.That(state.CameraPoseVisible, Is.False);
            Assert.That(state.TrajectoryVisible, Is.True);
            Assert.That(state.PointCloudVisible, Is.False);
            Assert.That(state.WorldReferenceVisible, Is.True);
            Assert.That(state.DiagnosticsVisible, Is.True);
        }

        [Test]
        public void RestoreUsesConfiguredDefaultsDeterministically()
        {
            var state = new VisualizationVisibilityState(true, false, true, false, true);
            state.Apply(
                VisualizationVisibilityCommand.ToggleCameraPose |
                VisualizationVisibilityCommand.ToggleTrajectory |
                VisualizationVisibilityCommand.ToggleDiagnostics);

            bool changed = state.Apply(VisualizationVisibilityCommand.RestoreDefaults);

            Assert.That(changed, Is.True);
            Assert.That(state.CameraPoseVisible, Is.True);
            Assert.That(state.TrajectoryVisible, Is.False);
            Assert.That(state.PointCloudVisible, Is.True);
            Assert.That(state.WorldReferenceVisible, Is.False);
            Assert.That(state.DiagnosticsVisible, Is.True);
            Assert.That(
                state.Apply(VisualizationVisibilityCommand.RestoreDefaults),
                Is.False);
        }

        [Test]
        public void NoneDoesNotChangeStateAndUnknownFlagsAreRejected()
        {
            var state = new VisualizationVisibilityState(true, true, true, true, true);

            Assert.That(state.Apply(VisualizationVisibilityCommand.None), Is.False);
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                state.Apply((VisualizationVisibilityCommand)(1 << 20)));
        }
    }
}
