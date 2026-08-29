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
    }
}
