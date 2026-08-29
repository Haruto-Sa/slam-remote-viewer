using System;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public static class CameraFrustumGeometry
    {
        public static Vector3[] Build(float aspectRatio, float verticalFieldOfViewDegrees, float depth)
        {
            if (!IsFinite(aspectRatio) || aspectRatio <= 0f)
            {
                throw new ArgumentOutOfRangeException(nameof(aspectRatio), "aspect ratio must be positive and finite");
            }

            if (!IsFinite(verticalFieldOfViewDegrees) ||
                verticalFieldOfViewDegrees <= 0f ||
                verticalFieldOfViewDegrees >= 180f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(verticalFieldOfViewDegrees),
                    "vertical field of view must be between zero and 180 degrees");
            }

            if (!IsFinite(depth) || depth <= 0f)
            {
                throw new ArgumentOutOfRangeException(nameof(depth), "depth must be positive and finite");
            }

            float halfHeight = Mathf.Tan(verticalFieldOfViewDegrees * 0.5f * Mathf.Deg2Rad) * depth;
            float halfWidth = halfHeight * aspectRatio;
            var origin = Vector3.zero;
            var topLeft = new Vector3(-halfWidth, halfHeight, depth);
            var topRight = new Vector3(halfWidth, halfHeight, depth);
            var bottomRight = new Vector3(halfWidth, -halfHeight, depth);
            var bottomLeft = new Vector3(-halfWidth, -halfHeight, depth);

            // One continuous line draws the image-plane rectangle and all four
            // rays. Some origin-to-corner edges are intentionally retraced.
            return new[]
            {
                origin,
                topLeft,
                topRight,
                bottomRight,
                bottomLeft,
                topLeft,
                origin,
                topRight,
                origin,
                bottomRight,
                origin,
                bottomLeft
            };
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
