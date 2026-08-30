using System;

namespace Slam.RemoteViewer
{
    [Flags]
    public enum VisualizationVisibilityCommand
    {
        None = 0,
        ToggleCameraPose = 1 << 0,
        ToggleTrajectory = 1 << 1,
        TogglePointCloud = 1 << 2,
        ToggleWorldReference = 1 << 3,
        ToggleDiagnostics = 1 << 4,
        RestoreDefaults = 1 << 5
    }

    public sealed class VisualizationVisibilityState
    {
        private const VisualizationVisibilityCommand AllCommands =
            VisualizationVisibilityCommand.ToggleCameraPose |
            VisualizationVisibilityCommand.ToggleTrajectory |
            VisualizationVisibilityCommand.TogglePointCloud |
            VisualizationVisibilityCommand.ToggleWorldReference |
            VisualizationVisibilityCommand.ToggleDiagnostics |
            VisualizationVisibilityCommand.RestoreDefaults;

        private readonly bool defaultCameraPoseVisible;
        private readonly bool defaultTrajectoryVisible;
        private readonly bool defaultPointCloudVisible;
        private readonly bool defaultWorldReferenceVisible;
        private readonly bool defaultDiagnosticsVisible;

        public VisualizationVisibilityState(
            bool cameraPoseVisible,
            bool trajectoryVisible,
            bool pointCloudVisible,
            bool worldReferenceVisible,
            bool diagnosticsVisible)
        {
            defaultCameraPoseVisible = cameraPoseVisible;
            defaultTrajectoryVisible = trajectoryVisible;
            defaultPointCloudVisible = pointCloudVisible;
            defaultWorldReferenceVisible = worldReferenceVisible;
            defaultDiagnosticsVisible = diagnosticsVisible;
            RestoreDefaults();
        }

        public bool CameraPoseVisible { get; private set; }
        public bool TrajectoryVisible { get; private set; }
        public bool PointCloudVisible { get; private set; }
        public bool WorldReferenceVisible { get; private set; }
        public bool DiagnosticsVisible { get; private set; }

        public bool Apply(VisualizationVisibilityCommand command)
        {
            if ((command & ~AllCommands) != 0)
            {
                throw new ArgumentOutOfRangeException(nameof(command), "command contains unknown flags");
            }
            if ((command & VisualizationVisibilityCommand.RestoreDefaults) != 0)
            {
                return RestoreDefaults();
            }

            bool changed = false;
            if ((command & VisualizationVisibilityCommand.ToggleCameraPose) != 0)
            {
                CameraPoseVisible = !CameraPoseVisible;
                changed = true;
            }
            if ((command & VisualizationVisibilityCommand.ToggleTrajectory) != 0)
            {
                TrajectoryVisible = !TrajectoryVisible;
                changed = true;
            }
            if ((command & VisualizationVisibilityCommand.TogglePointCloud) != 0)
            {
                PointCloudVisible = !PointCloudVisible;
                changed = true;
            }
            if ((command & VisualizationVisibilityCommand.ToggleWorldReference) != 0)
            {
                WorldReferenceVisible = !WorldReferenceVisible;
                changed = true;
            }
            if ((command & VisualizationVisibilityCommand.ToggleDiagnostics) != 0)
            {
                DiagnosticsVisible = !DiagnosticsVisible;
                changed = true;
            }
            return changed;
        }

        private bool RestoreDefaults()
        {
            bool changed =
                CameraPoseVisible != defaultCameraPoseVisible ||
                TrajectoryVisible != defaultTrajectoryVisible ||
                PointCloudVisible != defaultPointCloudVisible ||
                WorldReferenceVisible != defaultWorldReferenceVisible ||
                DiagnosticsVisible != defaultDiagnosticsVisible;
            CameraPoseVisible = defaultCameraPoseVisible;
            TrajectoryVisible = defaultTrajectoryVisible;
            PointCloudVisible = defaultPointCloudVisible;
            WorldReferenceVisible = defaultWorldReferenceVisible;
            DiagnosticsVisible = defaultDiagnosticsVisible;
            return changed;
        }
    }
}
