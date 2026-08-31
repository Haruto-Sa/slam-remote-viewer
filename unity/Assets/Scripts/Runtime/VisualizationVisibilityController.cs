using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class VisualizationVisibilityController : MonoBehaviour
    {
        [Header("Visualizers")]
        [SerializeField]
        private CameraPoseVisualizer cameraPoseVisualizer;

        [SerializeField]
        private CameraTrajectoryVisualizer trajectoryVisualizer;

        [SerializeField]
        private PointCloudVisualizer pointCloudVisualizer;

        [SerializeField]
        private WorldReferenceVisualizer worldReferenceVisualizer;

        [SerializeField]
        private TelemetryDiagnosticsOverlay diagnosticsOverlay;

        [Header("Shortcuts")]
        [SerializeField]
        private KeyCode cameraPoseKey = KeyCode.P;

        [SerializeField]
        private KeyCode trajectoryKey = KeyCode.T;

        [SerializeField]
        private KeyCode pointCloudKey = KeyCode.C;

        [SerializeField]
        private KeyCode worldReferenceKey = KeyCode.G;

        [SerializeField]
        private KeyCode diagnosticsKey = KeyCode.D;

        [SerializeField]
        private KeyCode restoreDefaultsKey = KeyCode.V;

        [Header("Default visibility")]
        [SerializeField]
        private bool cameraPoseVisibleByDefault = true;

        [SerializeField]
        private bool trajectoryVisibleByDefault = true;

        [SerializeField]
        private bool pointCloudVisibleByDefault = true;

        [SerializeField]
        private bool worldReferenceVisibleByDefault = true;

        [SerializeField]
        private bool diagnosticsVisibleByDefault = true;

        private VisualizationVisibilityState state;

        public VisualizationVisibilityState State => state;
        public KeyCode CameraPoseKey => cameraPoseKey;
        public KeyCode TrajectoryKey => trajectoryKey;
        public KeyCode PointCloudKey => pointCloudKey;
        public KeyCode WorldReferenceKey => worldReferenceKey;
        public KeyCode DiagnosticsKey => diagnosticsKey;
        public KeyCode RestoreDefaultsKey => restoreDefaultsKey;

        private void Awake()
        {
            Initialize();
        }

        private void Update()
        {
            ApplyCommand(ReadCommand());
        }

        public bool ApplyCommand(VisualizationVisibilityCommand command)
        {
            if (state == null)
            {
                Initialize();
            }

            bool changed = state.Apply(command);
            if (changed)
            {
                ApplyState();
            }
            return changed;
        }

        private void Initialize()
        {
            ResolveVisualizers();
            state = new VisualizationVisibilityState(
                cameraPoseVisibleByDefault,
                trajectoryVisibleByDefault,
                pointCloudVisibleByDefault,
                worldReferenceVisibleByDefault,
                diagnosticsVisibleByDefault);
            ApplyState();
        }

        private void ResolveVisualizers()
        {
            if (cameraPoseVisualizer == null)
            {
                cameraPoseVisualizer = GetComponent<CameraPoseVisualizer>();
            }
            if (trajectoryVisualizer == null)
            {
                trajectoryVisualizer = GetComponent<CameraTrajectoryVisualizer>();
            }
            if (pointCloudVisualizer == null)
            {
                pointCloudVisualizer = GetComponent<PointCloudVisualizer>();
            }
            if (worldReferenceVisualizer == null)
            {
                worldReferenceVisualizer = GetComponent<WorldReferenceVisualizer>();
            }
            if (diagnosticsOverlay == null)
            {
                diagnosticsOverlay = GetComponent<TelemetryDiagnosticsOverlay>();
            }
        }

        private VisualizationVisibilityCommand ReadCommand()
        {
            VisualizationVisibilityCommand command = VisualizationVisibilityCommand.None;
            if (Input.GetKeyDown(cameraPoseKey))
            {
                command |= VisualizationVisibilityCommand.ToggleCameraPose;
            }
            if (Input.GetKeyDown(trajectoryKey))
            {
                command |= VisualizationVisibilityCommand.ToggleTrajectory;
            }
            if (Input.GetKeyDown(pointCloudKey))
            {
                command |= VisualizationVisibilityCommand.TogglePointCloud;
            }
            if (Input.GetKeyDown(worldReferenceKey))
            {
                command |= VisualizationVisibilityCommand.ToggleWorldReference;
            }
            if (Input.GetKeyDown(diagnosticsKey))
            {
                command |= VisualizationVisibilityCommand.ToggleDiagnostics;
            }
            if (Input.GetKeyDown(restoreDefaultsKey))
            {
                command |= VisualizationVisibilityCommand.RestoreDefaults;
            }
            return command;
        }

        private void ApplyState()
        {
            cameraPoseVisualizer?.SetVisible(state.CameraPoseVisible);
            trajectoryVisualizer?.SetVisible(state.TrajectoryVisible);
            pointCloudVisualizer?.SetVisible(state.PointCloudVisible);
            worldReferenceVisualizer?.SetVisible(state.WorldReferenceVisible);
            diagnosticsOverlay?.SetVisible(state.DiagnosticsVisible);
        }
    }
}
