using System;
using System.Collections.Generic;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class PointCloudState
    {
        private readonly Dictionary<ulong, Vector3> points = new Dictionary<ulong, Vector3>();
        private string activeSession;

        public string ActiveSession => activeSession;
        public int PointCount => points.Count;
        public long Revision { get; private set; }

        public bool HandleMessage(ITelemetryMessage message)
        {
            if (message is SettingsMessage settings)
            {
                return ApplySettings(settings);
            }

            if (message is PointCloudMessage pointCloud)
            {
                return ApplyDelta(pointCloud);
            }

            return false;
        }

        public bool TryGetPoint(ulong id, out Vector3 position)
        {
            return points.TryGetValue(id, out position);
        }

        public void CopyOrderedPositions(List<Vector3> destination)
        {
            if (destination == null)
            {
                throw new ArgumentNullException(nameof(destination));
            }

            var ids = new List<ulong>(points.Keys);
            ids.Sort();
            destination.Clear();
            foreach (ulong id in ids)
            {
                destination.Add(points[id]);
            }
        }

        private bool ApplySettings(SettingsMessage settings)
        {
            if (settings == null || string.IsNullOrEmpty(settings.Session) ||
                settings.Session == activeSession)
            {
                return false;
            }

            points.Clear();
            activeSession = settings.Session;
            Revision++;
            return true;
        }

        private bool ApplyDelta(PointCloudMessage pointCloud)
        {
            if (pointCloud == null || activeSession == null || pointCloud.Session != activeSession)
            {
                return false;
            }

            bool changed = false;

            foreach (ulong id in pointCloud.Remove)
            {
                changed |= points.Remove(id);
            }

            foreach (PointEntry point in pointCloud.Update)
            {
                changed |= SetPoint(point);
            }

            foreach (PointEntry point in pointCloud.Add)
            {
                changed |= SetPoint(point);
            }

            if (changed)
            {
                Revision++;
            }

            return changed;
        }

        private bool SetPoint(PointEntry point)
        {
            var position = new Vector3((float)point.X, (float)point.Y, (float)point.Z);
            if (!IsFinite(position.x) || !IsFinite(position.y) || !IsFinite(position.z))
            {
                return false;
            }

            if (points.TryGetValue(point.Id, out Vector3 previous) && previous == position)
            {
                return false;
            }

            points[point.Id] = position;
            return true;
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
