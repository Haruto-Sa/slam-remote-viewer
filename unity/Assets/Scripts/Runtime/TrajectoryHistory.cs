using System;
using System.Collections.Generic;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class TrajectoryHistory
    {
        private readonly int capacity;
        private readonly float minimumDistanceSquared;
        private readonly List<List<Vector3>> segments = new List<List<Vector3>>();
        private string activeSession;
        private bool startNewSegment = true;

        public TrajectoryHistory(int capacity, float minimumDistance)
        {
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity), "capacity must be positive");
            }

            if (!IsFinite(minimumDistance) || minimumDistance < 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(minimumDistance),
                    "minimum distance must be non-negative and finite");
            }

            this.capacity = capacity;
            minimumDistanceSquared = minimumDistance * minimumDistance;
        }

        public string ActiveSession => activeSession;
        public int Capacity => capacity;
        public int PointCount { get; private set; }
        public int SegmentCount => segments.Count;

        public bool HandleMessage(ITelemetryMessage message)
        {
            if (message is SettingsMessage settings)
            {
                return ApplySettings(settings);
            }

            if (message is PoseMessage pose)
            {
                return ApplyPose(pose);
            }

            return false;
        }

        public IReadOnlyList<Vector3> GetSegment(int index)
        {
            return segments[index];
        }

        private bool ApplySettings(SettingsMessage settings)
        {
            if (settings == null || string.IsNullOrEmpty(settings.Session))
            {
                return false;
            }

            if (activeSession == settings.Session)
            {
                return false;
            }

            Clear();
            activeSession = settings.Session;
            return true;
        }

        private bool ApplyPose(PoseMessage pose)
        {
            if (pose == null || activeSession == null || pose.Session != activeSession)
            {
                return false;
            }

            if (pose.State != PoseTrackingState.Tracking)
            {
                startNewSegment = true;
                return false;
            }

            if (pose.Position == null || pose.Position.Count != 3)
            {
                return false;
            }

            var position = new Vector3(
                (float)pose.Position[0],
                (float)pose.Position[1],
                (float)pose.Position[2]);
            if (!IsFinite(position.x) || !IsFinite(position.y) || !IsFinite(position.z))
            {
                return false;
            }

            if (!startNewSegment && segments.Count > 0)
            {
                List<Vector3> current = segments[segments.Count - 1];
                Vector3 previous = current[current.Count - 1];
                if ((position - previous).sqrMagnitude < minimumDistanceSquared)
                {
                    return false;
                }
            }

            if (startNewSegment || segments.Count == 0)
            {
                segments.Add(new List<Vector3>());
                startNewSegment = false;
            }

            segments[segments.Count - 1].Add(position);
            PointCount++;
            TrimToCapacity();
            return true;
        }

        private void TrimToCapacity()
        {
            while (PointCount > capacity)
            {
                List<Vector3> oldest = segments[0];
                oldest.RemoveAt(0);
                PointCount--;
                if (oldest.Count == 0)
                {
                    segments.RemoveAt(0);
                }
            }
        }

        private void Clear()
        {
            segments.Clear();
            PointCount = 0;
            startNewSegment = true;
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
