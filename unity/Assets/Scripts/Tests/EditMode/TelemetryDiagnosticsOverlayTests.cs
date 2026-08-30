using NUnit.Framework;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class TelemetryDiagnosticsOverlayTests
    {
        [Test]
        public void HiddenOverlayStillProcessesMainThreadMessages()
        {
            var gameObject = new GameObject("Telemetry Diagnostics Test");
            try
            {
                var overlay = gameObject.AddComponent<TelemetryDiagnosticsOverlay>();
                overlay.ShowOverlay = false;

                overlay.HandleMessage(Settings("session-a"));

                Assert.That(overlay.enabled, Is.True);
                Assert.That(overlay.ShowOverlay, Is.False);
                Assert.That(overlay.IsVisible, Is.False);
                Assert.That(overlay.State.ActiveSession, Is.EqualTo("session-a"));
                Assert.That(overlay.State.HasReceivedMessage, Is.True);
            }
            finally
            {
                Object.DestroyImmediate(gameObject);
            }
        }

        [Test]
        public void VisibleOverlayContainsPointsInsideItsGuiPanel()
        {
            var gameObject = new GameObject("Telemetry Diagnostics Test");
            try
            {
                var overlay = gameObject.AddComponent<TelemetryDiagnosticsOverlay>();

                Assert.That(overlay.ContainsScreenPoint(new Vector2(20f, 780f), 800f), Is.True);
                Assert.That(overlay.ContainsScreenPoint(new Vector2(600f, 780f), 800f), Is.False);
                Assert.That(overlay.ContainsScreenPoint(new Vector2(20f, 200f), 800f), Is.False);
            }
            finally
            {
                Object.DestroyImmediate(gameObject);
            }
        }

        [Test]
        public void HiddenOverlayDoesNotBlockPointerInput()
        {
            var gameObject = new GameObject("Telemetry Diagnostics Test");
            try
            {
                var overlay = gameObject.AddComponent<TelemetryDiagnosticsOverlay>();
                overlay.ShowOverlay = false;

                Assert.That(overlay.ContainsScreenPoint(new Vector2(20f, 780f), 800f), Is.False);
            }
            finally
            {
                Object.DestroyImmediate(gameObject);
            }
        }

        private static SettingsMessage Settings(string session)
        {
            return new SettingsMessage(
                1,
                session,
                "m",
                "unity_world",
                "Twc",
                "xyzw",
                new CameraSettings("mock", "camera", 640, 480, 30),
                "delta");
        }
    }
}
