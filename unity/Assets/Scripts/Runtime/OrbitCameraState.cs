using System;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public enum OrbitCameraViewPreset
    {
        Front,
        Right,
        Back,
        Left
    }

    public readonly struct OrbitCameraFrameRequest
    {
        public OrbitCameraFrameRequest(
            Bounds bounds,
            float verticalFieldOfViewDegrees,
            float aspectRatio,
            float padding,
            float nearClipDistance)
        {
            Bounds = bounds;
            VerticalFieldOfViewDegrees = verticalFieldOfViewDegrees;
            AspectRatio = aspectRatio;
            Padding = padding;
            NearClipDistance = nearClipDistance;
        }

        public Bounds Bounds { get; }
        public float VerticalFieldOfViewDegrees { get; }
        public float AspectRatio { get; }
        public float Padding { get; }
        public float NearClipDistance { get; }
    }

    public readonly struct OrbitCameraCommand
    {
        public OrbitCameraCommand(
            Vector2 orbit,
            Vector2 pan,
            float zoom,
            bool reset = false,
            bool pointerBlocked = false,
            OrbitCameraViewPreset? viewPreset = null,
            OrbitCameraFrameRequest? frameRequest = null)
        {
            Orbit = orbit;
            Pan = pan;
            Zoom = zoom;
            Reset = reset;
            PointerBlocked = pointerBlocked;
            ViewPreset = viewPreset;
            FrameRequest = frameRequest;
        }

        public Vector2 Orbit { get; }
        public Vector2 Pan { get; }
        public float Zoom { get; }
        public bool Reset { get; }
        public bool PointerBlocked { get; }
        public OrbitCameraViewPreset? ViewPreset { get; }
        public OrbitCameraFrameRequest? FrameRequest { get; }
    }

    public sealed class OrbitCameraState
    {
        private readonly Vector3 initialFocusPoint;
        private readonly float initialYawDegrees;
        private readonly float initialPitchDegrees;
        private readonly float initialDistance;
        private readonly float minimumDistance;
        private readonly float maximumDistance;
        private readonly float minimumPitchDegrees;
        private readonly float maximumPitchDegrees;

        public OrbitCameraState(
            Vector3 initialPosition,
            Vector3 initialFocusPoint,
            float minimumDistance,
            float maximumDistance,
            float minimumPitchDegrees,
            float maximumPitchDegrees)
        {
            ValidateVector(initialPosition, nameof(initialPosition));
            ValidateVector(initialFocusPoint, nameof(initialFocusPoint));
            ValidateRange(minimumDistance, maximumDistance, "distance", requirePositive: true);
            ValidateRange(minimumPitchDegrees, maximumPitchDegrees, "pitch", requirePositive: false);
            if (minimumPitchDegrees <= -90f || maximumPitchDegrees >= 90f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(minimumPitchDegrees),
                    "pitch limits must remain strictly between -90 and 90 degrees");
            }

            Vector3 lookDirection = initialFocusPoint - initialPosition;
            float rawDistance = lookDirection.magnitude;
            if (!IsFinite(rawDistance) || rawDistance <= 0.000001f)
            {
                throw new ArgumentException(
                    "initial position and focus point must be different",
                    nameof(initialPosition));
            }

            lookDirection /= rawDistance;
            this.initialFocusPoint = initialFocusPoint;
            this.minimumDistance = minimumDistance;
            this.maximumDistance = maximumDistance;
            this.minimumPitchDegrees = minimumPitchDegrees;
            this.maximumPitchDegrees = maximumPitchDegrees;
            initialYawDegrees = Mathf.Atan2(lookDirection.x, lookDirection.z) * Mathf.Rad2Deg;
            initialPitchDegrees = Mathf.Clamp(
                -Mathf.Asin(Mathf.Clamp(lookDirection.y, -1f, 1f)) * Mathf.Rad2Deg,
                minimumPitchDegrees,
                maximumPitchDegrees);
            initialDistance = Mathf.Clamp(rawDistance, minimumDistance, maximumDistance);

            Reset();
        }

        public Vector3 Position { get; private set; }
        public Quaternion Rotation { get; private set; }
        public Vector3 FocusPoint { get; private set; }
        public float YawDegrees { get; private set; }
        public float PitchDegrees { get; private set; }
        public float Distance { get; private set; }

        public bool ApplyCommand(
            OrbitCameraCommand command,
            float orbitDegreesPerUnit,
            float panUnitsPerUnit,
            float zoomUnitsPerUnit)
        {
            ValidateScale(orbitDegreesPerUnit, nameof(orbitDegreesPerUnit));
            ValidateScale(panUnitsPerUnit, nameof(panUnitsPerUnit));
            ValidateScale(zoomUnitsPerUnit, nameof(zoomUnitsPerUnit));

            if (command.Reset)
            {
                Reset();
                return true;
            }
            if (command.ViewPreset.HasValue)
            {
                return SetViewPreset(command.ViewPreset.Value);
            }
            if (command.FrameRequest.HasValue)
            {
                OrbitCameraFrameRequest request = command.FrameRequest.Value;
                return FrameBounds(
                    request.Bounds,
                    request.VerticalFieldOfViewDegrees,
                    request.AspectRatio,
                    request.Padding,
                    request.NearClipDistance);
            }
            if (command.PointerBlocked)
            {
                return false;
            }

            bool changed = false;
            if (command.Orbit.sqrMagnitude > 0f)
            {
                YawDegrees = NormalizeAngle(YawDegrees + command.Orbit.x * orbitDegreesPerUnit);
                PitchDegrees = Mathf.Clamp(
                    PitchDegrees + command.Orbit.y * orbitDegreesPerUnit,
                    minimumPitchDegrees,
                    maximumPitchDegrees);
                RebuildPose();
                changed = true;
            }
            if (command.Pan.sqrMagnitude > 0f)
            {
                Vector3 right = Rotation * Vector3.right;
                Vector3 up = Rotation * Vector3.up;
                FocusPoint +=
                    right * (-command.Pan.x * panUnitsPerUnit) +
                    up * (-command.Pan.y * panUnitsPerUnit);
                changed = true;
            }
            if (Mathf.Abs(command.Zoom) > 0f)
            {
                Distance = Mathf.Clamp(
                    Distance - command.Zoom * zoomUnitsPerUnit,
                    minimumDistance,
                    maximumDistance);
                changed = true;
            }

            if (changed)
            {
                RebuildPose();
            }
            return changed;
        }

        public void Orbit(float yawDeltaDegrees, float pitchDeltaDegrees)
        {
            ValidateFinite(yawDeltaDegrees, nameof(yawDeltaDegrees));
            ValidateFinite(pitchDeltaDegrees, nameof(pitchDeltaDegrees));
            YawDegrees = NormalizeAngle(YawDegrees + yawDeltaDegrees);
            PitchDegrees = Mathf.Clamp(
                PitchDegrees + pitchDeltaDegrees,
                minimumPitchDegrees,
                maximumPitchDegrees);
            RebuildPose();
        }

        public void Pan(float rightDistance, float upDistance)
        {
            ValidateFinite(rightDistance, nameof(rightDistance));
            ValidateFinite(upDistance, nameof(upDistance));
            FocusPoint += Rotation * Vector3.right * rightDistance;
            FocusPoint += Rotation * Vector3.up * upDistance;
            RebuildPose();
        }

        public void Zoom(float distanceDelta)
        {
            ValidateFinite(distanceDelta, nameof(distanceDelta));
            Distance = Mathf.Clamp(
                Distance + distanceDelta,
                minimumDistance,
                maximumDistance);
            RebuildPose();
        }

        public bool SetViewPreset(OrbitCameraViewPreset preset)
        {
            float targetYaw;
            switch (preset)
            {
                case OrbitCameraViewPreset.Front:
                    targetYaw = 0f;
                    break;
                case OrbitCameraViewPreset.Right:
                    targetYaw = -90f;
                    break;
                case OrbitCameraViewPreset.Back:
                    targetYaw = -180f;
                    break;
                case OrbitCameraViewPreset.Left:
                    targetYaw = 90f;
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(preset));
            }

            bool changed =
                !Mathf.Approximately(YawDegrees, targetYaw) ||
                !Mathf.Approximately(PitchDegrees, 0f);
            YawDegrees = targetYaw;
            PitchDegrees = 0f;
            RebuildPose();
            return changed;
        }

        public bool FrameBounds(
            Bounds bounds,
            float verticalFieldOfViewDegrees,
            float aspectRatio,
            float padding,
            float nearClipDistance)
        {
            ValidateBounds(bounds);
            if (!IsFinite(verticalFieldOfViewDegrees) ||
                verticalFieldOfViewDegrees <= 0f ||
                verticalFieldOfViewDegrees >= 180f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(verticalFieldOfViewDegrees),
                    "vertical field of view must be between zero and 180 degrees");
            }
            if (!IsFinite(aspectRatio) || aspectRatio <= 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(aspectRatio),
                    "aspect ratio must be positive and finite");
            }
            if (!IsFinite(padding) || padding < 1f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(padding),
                    "padding must be finite and at least one");
            }
            if (!IsFinite(nearClipDistance) || nearClipDistance <= 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(nearClipDistance),
                    "near clip distance must be positive and finite");
            }

            float verticalTangent = Mathf.Tan(
                verticalFieldOfViewDegrees * 0.5f * Mathf.Deg2Rad);
            float horizontalTangent = verticalTangent * aspectRatio;
            Quaternion inverseRotation = Quaternion.Inverse(Rotation);
            Vector3 center = bounds.center;
            Vector3 extents = bounds.extents;
            float requiredDistance = minimumDistance;

            for (var xSign = -1; xSign <= 1; xSign += 2)
            {
                for (var ySign = -1; ySign <= 1; ySign += 2)
                {
                    for (var zSign = -1; zSign <= 1; zSign += 2)
                    {
                        var offset = new Vector3(
                            extents.x * xSign,
                            extents.y * ySign,
                            extents.z * zSign);
                        Vector3 cameraSpace = inverseRotation * offset;
                        requiredDistance = Mathf.Max(
                            requiredDistance,
                            Mathf.Abs(cameraSpace.x) * padding / horizontalTangent -
                            cameraSpace.z,
                            Mathf.Abs(cameraSpace.y) * padding / verticalTangent -
                            cameraSpace.z,
                            nearClipDistance - cameraSpace.z);
                    }
                }
            }

            float targetDistance = Mathf.Clamp(
                requiredDistance,
                minimumDistance,
                maximumDistance);
            bool changed =
                Vector3.Distance(FocusPoint, center) > 0.000001f ||
                !Mathf.Approximately(Distance, targetDistance);
            FocusPoint = center;
            Distance = targetDistance;
            RebuildPose();
            return changed;
        }

        public void Reset()
        {
            FocusPoint = initialFocusPoint;
            YawDegrees = initialYawDegrees;
            PitchDegrees = initialPitchDegrees;
            Distance = initialDistance;
            RebuildPose();
        }

        private void RebuildPose()
        {
            float yawRadians = YawDegrees * Mathf.Deg2Rad;
            float pitchRadians = PitchDegrees * Mathf.Deg2Rad;
            float cosinePitch = Mathf.Cos(pitchRadians);
            var forward = new Vector3(
                Mathf.Sin(yawRadians) * cosinePitch,
                -Mathf.Sin(pitchRadians),
                Mathf.Cos(yawRadians) * cosinePitch);

            Position = FocusPoint - forward * Distance;
            Rotation = Quaternion.LookRotation(forward, Vector3.up);
        }

        private static float NormalizeAngle(float angle)
        {
            return Mathf.Repeat(angle + 180f, 360f) - 180f;
        }

        private static void ValidateRange(
            float minimum,
            float maximum,
            string name,
            bool requirePositive)
        {
            if (!IsFinite(minimum) || !IsFinite(maximum) || minimum >= maximum ||
                (requirePositive && minimum <= 0f))
            {
                throw new ArgumentOutOfRangeException(
                    name,
                    $"{name} limits must be finite and increasing");
            }
        }

        private static void ValidateVector(Vector3 value, string name)
        {
            if (!IsFinite(value.x) || !IsFinite(value.y) || !IsFinite(value.z))
            {
                throw new ArgumentOutOfRangeException(name, "vector must contain finite values");
            }
        }

        private static void ValidateBounds(Bounds value)
        {
            ValidateVector(value.center, nameof(value));
            Vector3 size = value.size;
            if (!IsFinite(size.x) || !IsFinite(size.y) || !IsFinite(size.z) ||
                size.x < 0f || size.y < 0f || size.z < 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(value),
                    "bounds size must be finite and non-negative");
            }
        }

        private static void ValidateScale(float value, string name)
        {
            if (!IsFinite(value) || value < 0f)
            {
                throw new ArgumentOutOfRangeException(name, "scale must be finite and non-negative");
            }
        }

        private static void ValidateFinite(float value, string name)
        {
            if (!IsFinite(value))
            {
                throw new ArgumentOutOfRangeException(name, "value must be finite");
            }
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
