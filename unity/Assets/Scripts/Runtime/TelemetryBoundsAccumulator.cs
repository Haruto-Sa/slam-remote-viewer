using System;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class TelemetryBoundsAccumulator
    {
        private Bounds bounds;

        public bool HasBounds { get; private set; }

        public void Add(Bounds value)
        {
            ValidateBounds(value);
            if (!HasBounds)
            {
                bounds = value;
                HasBounds = true;
                return;
            }

            bounds.Encapsulate(value);
        }

        public bool TryGetBounds(out Bounds value)
        {
            value = bounds;
            return HasBounds;
        }

        private static void ValidateBounds(Bounds value)
        {
            Vector3 center = value.center;
            Vector3 size = value.size;
            if (!IsFinite(center.x) || !IsFinite(center.y) || !IsFinite(center.z) ||
                !IsFinite(size.x) || !IsFinite(size.y) || !IsFinite(size.z) ||
                size.x < 0f || size.y < 0f || size.z < 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(value),
                    "bounds must contain finite values and non-negative size");
            }
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
