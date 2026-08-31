using NUnit.Framework;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class ViewerControlsOverlayTests
    {
        [Test]
        public void TogglesVisibilityWithoutDisablingTheComponent()
        {
            var gameObject = new GameObject("Viewer Controls Overlay Test");
            try
            {
                var overlay = gameObject.AddComponent<ViewerControlsOverlay>();

                overlay.ToggleVisible();

                Assert.That(overlay.IsVisible, Is.False);
                Assert.That(overlay.enabled, Is.True);

                overlay.ToggleVisible();

                Assert.That(overlay.IsVisible, Is.True);
            }
            finally
            {
                Object.DestroyImmediate(gameObject);
            }
        }

        [Test]
        public void ContentListsEveryCurrentViewerControl()
        {
            var cameraHost = new GameObject("Viewer Camera Test");
            var overlayHost = new GameObject("Viewer Controls Overlay Test");
            try
            {
                cameraHost.AddComponent<OrbitCameraController>();
                overlayHost.AddComponent<VisualizationVisibilityController>();
                var overlay = overlayHost.AddComponent<ViewerControlsOverlay>();

                string content = overlay.BuildContent();

                Assert.That(content, Does.Contain("RMB drag"));
                Assert.That(content, Does.Contain("MMB drag"));
                Assert.That(content, Does.Contain("Wheel"));
                Assert.That(content, Does.Contain("1 Front"));
                Assert.That(content, Does.Contain("2 Right"));
                Assert.That(content, Does.Contain("3 Back"));
                Assert.That(content, Does.Contain("4 Left"));
                Assert.That(content, Does.Contain("F              Frame visible telemetry"));
                Assert.That(content, Does.Contain("R              Reset camera"));
                Assert.That(content, Does.Contain("P Pose"));
                Assert.That(content, Does.Contain("T Trajectory"));
                Assert.That(content, Does.Contain("C Point cloud"));
                Assert.That(content, Does.Contain("G Grid"));
                Assert.That(content, Does.Contain("D Diagnostics"));
                Assert.That(content, Does.Contain("V Restore"));
                Assert.That(content, Does.Contain("H              Hide this help"));
            }
            finally
            {
                Object.DestroyImmediate(cameraHost);
                Object.DestroyImmediate(overlayHost);
            }
        }

        [Test]
        public void OnlyVisiblePanelBlocksPointerInput()
        {
            var gameObject = new GameObject("Viewer Controls Overlay Test");
            try
            {
                var overlay = gameObject.AddComponent<ViewerControlsOverlay>();

                Assert.That(overlay.ContainsScreenPoint(new Vector2(560f, 780f), 800f), Is.True);
                Assert.That(overlay.ContainsScreenPoint(new Vector2(20f, 780f), 800f), Is.False);

                overlay.SetVisible(false);

                Assert.That(overlay.ContainsScreenPoint(new Vector2(560f, 780f), 800f), Is.False);
            }
            finally
            {
                Object.DestroyImmediate(gameObject);
            }
        }

        [TestCase(KeyCode.Alpha1, "1")]
        [TestCase(KeyCode.H, "H")]
        public void FormatsShortcutKeysForDisplay(KeyCode key, string expected)
        {
            Assert.That(ViewerControlsOverlay.FormatKey(key), Is.EqualTo(expected));
        }
    }
}
