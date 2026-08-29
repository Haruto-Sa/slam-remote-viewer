using System;
using Slam.RemoteViewer.Network;

namespace Slam.RemoteViewer
{
    public enum TelemetryHealthStatus
    {
        Stopped,
        Waiting,
        Receiving,
        Stale
    }

    public sealed class TelemetryDiagnosticsState
    {
        private double staleTimeoutSeconds;
        private double? latestMessageTimeSeconds;
        private bool isRunning;

        public TelemetryDiagnosticsState(double staleTimeoutSeconds)
        {
            StaleTimeoutSeconds = staleTimeoutSeconds;
        }

        public double StaleTimeoutSeconds
        {
            get => staleTimeoutSeconds;
            set
            {
                if (!IsFinite(value) || value <= 0.0)
                {
                    throw new ArgumentOutOfRangeException(
                        nameof(value),
                        "stale timeout must be positive and finite");
                }

                staleTimeoutSeconds = value;
            }
        }

        public string Endpoint { get; private set; }
        public string ActiveSession { get; private set; }
        public PoseTrackingState? TrackingState { get; private set; }
        public long AcceptedCount { get; private set; }
        public long RejectedCount { get; private set; }
        public long DroppedCount { get; private set; }
        public int QueueCount { get; private set; }
        public int? TrajectoryPointCount { get; private set; }
        public int? PointCloudPointCount { get; private set; }
        public long FaultCount { get; private set; }
        public string LastFault { get; private set; }
        public string LastRejectionReason { get; private set; }
        public bool HasReceivedMessage => latestMessageTimeSeconds.HasValue;

        public void Start(string endpoint)
        {
            Endpoint = endpoint;
            isRunning = true;
            ActiveSession = null;
            TrackingState = null;
            latestMessageTimeSeconds = null;
        }

        public void SetRunning(bool running)
        {
            isRunning = running;
        }

        public bool Observe(ITelemetryMessage message, double nowSeconds)
        {
            if (message == null)
            {
                throw new ArgumentNullException(nameof(message));
            }
            if (!IsFinite(nowSeconds))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(nowSeconds),
                    "observation time must be finite");
            }

            if (message is SettingsMessage settings)
            {
                ActiveSession = settings.Session;
                TrackingState = null;
            }
            else
            {
                if (string.IsNullOrEmpty(ActiveSession) ||
                    !string.Equals(ActiveSession, message.Session, StringComparison.Ordinal))
                {
                    return false;
                }

                if (message is PoseMessage pose)
                {
                    TrackingState = pose.State;
                }
            }

            latestMessageTimeSeconds = nowSeconds;
            return true;
        }

        public void UpdateMetrics(
            long acceptedCount,
            long rejectedCount,
            long droppedCount,
            int queueCount,
            int? trajectoryPointCount,
            int? pointCloudPointCount,
            long faultCount,
            string lastFault,
            string lastRejectionReason)
        {
            AcceptedCount = Math.Max(0L, acceptedCount);
            RejectedCount = Math.Max(0L, rejectedCount);
            DroppedCount = Math.Max(0L, droppedCount);
            QueueCount = Math.Max(0, queueCount);
            TrajectoryPointCount = ClampOptionalCount(trajectoryPointCount);
            PointCloudPointCount = ClampOptionalCount(pointCloudPointCount);
            FaultCount = Math.Max(0L, faultCount);
            LastFault = lastFault;
            LastRejectionReason = lastRejectionReason;
        }

        public TelemetryHealthStatus GetStatus(double nowSeconds)
        {
            ValidateNow(nowSeconds);
            if (!isRunning)
            {
                return TelemetryHealthStatus.Stopped;
            }
            if (!latestMessageTimeSeconds.HasValue)
            {
                return TelemetryHealthStatus.Waiting;
            }
            return GetLatestMessageAgeSeconds(nowSeconds) >= staleTimeoutSeconds
                ? TelemetryHealthStatus.Stale
                : TelemetryHealthStatus.Receiving;
        }

        public double? GetLatestMessageAgeSeconds(double nowSeconds)
        {
            ValidateNow(nowSeconds);
            if (!latestMessageTimeSeconds.HasValue)
            {
                return null;
            }

            return Math.Max(0.0, nowSeconds - latestMessageTimeSeconds.Value);
        }

        private static int? ClampOptionalCount(int? count)
        {
            return count.HasValue ? Math.Max(0, count.Value) : (int?)null;
        }

        private static void ValidateNow(double nowSeconds)
        {
            if (!IsFinite(nowSeconds))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(nowSeconds),
                    "current time must be finite");
            }
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
