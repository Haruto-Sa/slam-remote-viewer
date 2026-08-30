using System;
using NUnit.Framework;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class OrbitCameraStateTests
    {
        [Test]
        public void OrbitPreservesFocusAndDistance()
        {
            OrbitCameraState state = CreateState();
            Vector3 focus = state.FocusPoint;
            float distance = state.Distance;

            state.Orbit(45f, 30f);

            AssertVector(state.FocusPoint, focus);
            Assert.That(state.Distance, Is.EqualTo(distance).Within(0.0001f));
            Assert.That(Vector3.Distance(state.Position, focus), Is.EqualTo(distance).Within(0.0001f));
            AssertVector(state.Rotation * Vector3.forward, (focus - state.Position).normalized);
        }

        [Test]
        public void PitchCannotFlipThroughPoles()
        {
            OrbitCameraState state = CreateState();

            state.Orbit(0f, 1000f);
            Assert.That(state.PitchDegrees, Is.EqualTo(80f));

            state.Orbit(0f, -2000f);
            Assert.That(state.PitchDegrees, Is.EqualTo(-80f));
        }

        [Test]
        public void ZoomClampsMinimumAndMaximumDistance()
        {
            OrbitCameraState state = CreateState();

            state.Zoom(-1000f);
            Assert.That(state.Distance, Is.EqualTo(2f));

            state.Zoom(1000f);
            Assert.That(state.Distance, Is.EqualTo(20f));
        }

        [Test]
        public void PanMovesCameraAndFocusBySameAmount()
        {
            OrbitCameraState state = CreateState();
            Vector3 initialPosition = state.Position;
            Vector3 initialFocus = state.FocusPoint;

            state.Pan(2f, 1f);

            AssertVector(
                state.Position - initialPosition,
                state.FocusPoint - initialFocus);
            Assert.That(state.Distance, Is.EqualTo(10f).Within(0.0001f));
        }

        [Test]
        public void ResetRestoresDeterministicInitialPose()
        {
            OrbitCameraState state = CreateState();
            Vector3 initialPosition = state.Position;
            Vector3 initialFocus = state.FocusPoint;
            Quaternion initialRotation = state.Rotation;

            state.Orbit(50f, 25f);
            state.Pan(3f, -2f);
            state.Zoom(-4f);
            state.SetViewPreset(OrbitCameraViewPreset.Back);
            state.Reset();

            AssertVector(state.Position, initialPosition);
            AssertVector(state.FocusPoint, initialFocus);
            Assert.That(Quaternion.Angle(state.Rotation, initialRotation), Is.LessThan(0.0001f));
            Assert.That(state.Distance, Is.EqualTo(10f).Within(0.0001f));
        }

        [Test]
        public void BlockedPointerInputDoesNotChangePose()
        {
            OrbitCameraState state = CreateState();
            Vector3 initialPosition = state.Position;
            Vector3 initialFocus = state.FocusPoint;

            bool changed = state.ApplyCommand(
                new OrbitCameraCommand(
                    new Vector2(1f, 1f),
                    new Vector2(1f, 1f),
                    1f,
                    pointerBlocked: true),
                10f,
                2f,
                1f);

            Assert.That(changed, Is.False);
            AssertVector(state.Position, initialPosition);
            AssertVector(state.FocusPoint, initialFocus);
        }

        [TestCase(OrbitCameraViewPreset.Front, 0f, -1f, 0f, 1f, 0f)]
        [TestCase(OrbitCameraViewPreset.Right, 1f, 0f, -1f, 0f, -90f)]
        [TestCase(OrbitCameraViewPreset.Back, 0f, 1f, 0f, -1f, -180f)]
        [TestCase(OrbitCameraViewPreset.Left, -1f, 0f, 1f, 0f, 90f)]
        public void CardinalPresetPreservesFocusAndDistance(
            OrbitCameraViewPreset preset,
            float offsetX,
            float offsetZ,
            float forwardX,
            float forwardZ,
            float expectedYaw)
        {
            OrbitCameraState state = CreateState();
            state.Orbit(35f, 20f);
            state.Pan(2f, 1f);
            state.Zoom(-3f);
            Vector3 focus = state.FocusPoint;
            float distance = state.Distance;

            bool changed = state.SetViewPreset(preset);

            Assert.That(changed, Is.True);
            AssertVector(state.FocusPoint, focus);
            Assert.That(state.Distance, Is.EqualTo(distance).Within(0.0001f));
            Assert.That(state.PitchDegrees, Is.Zero.Within(0.0001f));
            Assert.That(state.YawDegrees, Is.EqualTo(expectedYaw).Within(0.0001f));
            AssertVector(
                state.Position,
                focus + new Vector3(offsetX, 0f, offsetZ) * distance);
            AssertVector(
                state.Rotation * Vector3.forward,
                new Vector3(forwardX, 0f, forwardZ));
        }

        [Test]
        public void RepeatedPresetSelectionDoesNotAccumulateError()
        {
            OrbitCameraState state = CreateState();
            state.Orbit(37f, 22f);
            state.SetViewPreset(OrbitCameraViewPreset.Right);
            Vector3 expectedPosition = state.Position;
            Quaternion expectedRotation = state.Rotation;

            for (var index = 0; index < 100; index++)
            {
                Assert.That(
                    state.SetViewPreset(OrbitCameraViewPreset.Right),
                    Is.False);
            }

            AssertVector(state.Position, expectedPosition);
            Assert.That(
                Quaternion.Angle(state.Rotation, expectedRotation),
                Is.LessThan(0.0001f));
        }

        [Test]
        public void PresetCommandWorksOverOverlayAndOrbitContinuesAfterward()
        {
            OrbitCameraState state = CreateState();

            bool changed = state.ApplyCommand(
                new OrbitCameraCommand(
                    Vector2.zero,
                    Vector2.zero,
                    0f,
                    pointerBlocked: true,
                    viewPreset: OrbitCameraViewPreset.Left),
                10f,
                2f,
                1f);
            state.Orbit(10f, 5f);

            Assert.That(changed, Is.True);
            Assert.That(state.YawDegrees, Is.EqualTo(100f).Within(0.0001f));
            Assert.That(state.PitchDegrees, Is.EqualTo(5f).Within(0.0001f));
        }

        [Test]
        public void PanZoomAndOrbitRemainUsableAfterPreset()
        {
            OrbitCameraState state = CreateState();
            state.SetViewPreset(OrbitCameraViewPreset.Back);
            Vector3 initialFocus = state.FocusPoint;
            float initialDistance = state.Distance;

            state.Pan(1f, 1f);
            state.Zoom(-2f);
            state.Orbit(15f, 10f);

            Assert.That(
                Vector3.Distance(state.FocusPoint, initialFocus),
                Is.GreaterThan(0.1f));
            Assert.That(state.Distance, Is.EqualTo(initialDistance - 2f).Within(0.0001f));
            Assert.That(state.YawDegrees, Is.EqualTo(-165f).Within(0.0001f));
            Assert.That(state.PitchDegrees, Is.EqualTo(10f).Within(0.0001f));
            AssertVector(
                state.Rotation * Vector3.forward,
                (state.FocusPoint - state.Position).normalized);
        }

        [Test]
        public void FrameBoundsCentersAndFitsGeometryWhilePreservingDirection()
        {
            OrbitCameraState state = CreateState();
            state.Orbit(30f, 15f);
            Quaternion initialRotation = state.Rotation;
            var bounds = new Bounds(
                new Vector3(5f, 2f, 3f),
                new Vector3(4f, 2f, 6f));

            bool changed = state.FrameBounds(bounds, 60f, 16f / 9f, 1.1f, 0.3f);

            Assert.That(changed, Is.True);
            AssertVector(state.FocusPoint, bounds.center);
            Assert.That(
                Quaternion.Angle(state.Rotation, initialRotation),
                Is.LessThan(0.0001f));
            AssertBoundsFit(state, bounds, 60f, 16f / 9f, 1.1f, 0.3f);
        }

        [Test]
        public void FrameBoundsRespectsDistanceLimits()
        {
            OrbitCameraState state = CreateState();

            state.FrameBounds(
                new Bounds(new Vector3(4f, 5f, 6f), Vector3.zero),
                60f,
                1f,
                1f,
                0.3f);
            Assert.That(state.Distance, Is.EqualTo(2f));

            state.FrameBounds(
                new Bounds(Vector3.zero, Vector3.one * 1000f),
                60f,
                1f,
                1f,
                0.3f);
            Assert.That(state.Distance, Is.EqualTo(20f));
        }

        [Test]
        public void RepeatedFramingDoesNotAccumulateError()
        {
            OrbitCameraState state = CreateState();
            var bounds = new Bounds(new Vector3(3f, 2f, 1f), new Vector3(6f, 4f, 2f));
            state.FrameBounds(bounds, 70f, 1.5f, 1.2f, 0.3f);
            Vector3 expectedPosition = state.Position;
            Quaternion expectedRotation = state.Rotation;
            float expectedDistance = state.Distance;

            for (var index = 0; index < 100; index++)
            {
                Assert.That(state.FrameBounds(bounds, 70f, 1.5f, 1.2f, 0.3f), Is.False);
            }

            AssertVector(state.Position, expectedPosition);
            Assert.That(state.Distance, Is.EqualTo(expectedDistance).Within(0.0001f));
            Assert.That(
                Quaternion.Angle(state.Rotation, expectedRotation),
                Is.LessThan(0.0001f));
        }

        [Test]
        public void RejectsInvalidConstructionAndCommandValues()
        {
            Assert.Throws<ArgumentException>(() => new OrbitCameraState(
                Vector3.zero,
                Vector3.zero,
                1f,
                10f,
                -80f,
                80f));
            Assert.Throws<ArgumentOutOfRangeException>(() => new OrbitCameraState(
                new Vector3(0f, 0f, -10f),
                Vector3.zero,
                0f,
                10f,
                -80f,
                80f));

            OrbitCameraState state = CreateState();
            Assert.Throws<ArgumentOutOfRangeException>(() => state.Zoom(float.NaN));
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                state.SetViewPreset((OrbitCameraViewPreset)999));
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                state.FrameBounds(new Bounds(), 0f, 1f, 1f, 0.3f));
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                state.FrameBounds(new Bounds(), 60f, 0f, 1f, 0.3f));
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                state.FrameBounds(new Bounds(), 60f, 1f, 0.9f, 0.3f));
        }

        private static OrbitCameraState CreateState()
        {
            return new OrbitCameraState(
                new Vector3(0f, 1f, -10f),
                new Vector3(0f, 1f, 0f),
                2f,
                20f,
                -80f,
                80f);
        }

        private static void AssertVector(Vector3 actual, Vector3 expected)
        {
            Assert.That(Vector3.Distance(actual, expected), Is.LessThan(0.0001f));
        }

        private static void AssertBoundsFit(
            OrbitCameraState state,
            Bounds bounds,
            float verticalFieldOfViewDegrees,
            float aspectRatio,
            float padding,
            float nearClipDistance)
        {
            float verticalTangent = Mathf.Tan(
                verticalFieldOfViewDegrees * 0.5f * Mathf.Deg2Rad);
            float horizontalTangent = verticalTangent * aspectRatio;
            Quaternion inverseRotation = Quaternion.Inverse(state.Rotation);
            Vector3 extents = bounds.extents;
            for (var xSign = -1; xSign <= 1; xSign += 2)
            {
                for (var ySign = -1; ySign <= 1; ySign += 2)
                {
                    for (var zSign = -1; zSign <= 1; zSign += 2)
                    {
                        Vector3 corner = bounds.center + new Vector3(
                            extents.x * xSign,
                            extents.y * ySign,
                            extents.z * zSign);
                        Vector3 cameraSpace = inverseRotation * (corner - state.Position);
                        Assert.That(cameraSpace.z, Is.GreaterThanOrEqualTo(nearClipDistance - 0.0001f));
                        Assert.That(
                            Mathf.Abs(cameraSpace.x) * padding,
                            Is.LessThanOrEqualTo(cameraSpace.z * horizontalTangent + 0.0001f));
                        Assert.That(
                            Mathf.Abs(cameraSpace.y) * padding,
                            Is.LessThanOrEqualTo(cameraSpace.z * verticalTangent + 0.0001f));
                    }
                }
            }
        }
    }
}
