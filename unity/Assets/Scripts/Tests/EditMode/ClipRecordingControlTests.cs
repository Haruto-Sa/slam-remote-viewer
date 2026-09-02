using NUnit.Framework;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer.Tests
{
    public sealed class ClipRecordingControlTests
    {
        [Test]
        public void AppliesRecordingAndCompletedStateTransitions()
        {
            var state = new ClipRecordingViewState();

            state.Apply(Response(
                "{\"ok\":true,\"state\":\"recording\",\"session\":\"demo\"," +
                "\"elapsed_seconds\":1.25,\"message_count\":7," +
                "\"output_path\":null,\"error\":null}"));

            Assert.That(state.State, Is.EqualTo(ClipRecordingState.Recording));
            Assert.That(state.CanStart, Is.False);
            Assert.That(state.CanStop, Is.True);
            Assert.That(state.ElapsedSeconds, Is.EqualTo(1.25));
            Assert.That(state.MessageCount, Is.EqualTo(7));

            state.Apply(Response(
                "{\"ok\":true,\"state\":\"completed\",\"session\":\"demo\"," +
                "\"elapsed_seconds\":2.5,\"message_count\":12," +
                "\"output_path\":\"recordings/clips/demo-clip-001\",\"error\":null}"));

            Assert.That(state.State, Is.EqualTo(ClipRecordingState.Completed));
            Assert.That(state.CanStart, Is.True);
            Assert.That(state.CanStop, Is.False);
            Assert.That(state.OutputPath, Does.EndWith("demo-clip-001"));
        }

        [Test]
        public void FailedStateAllowsAnotherStartAndRetainsError()
        {
            var state = new ClipRecordingViewState();
            state.Apply(Response(
                "{\"ok\":false,\"state\":\"failed\",\"session\":null," +
                "\"elapsed_seconds\":0,\"message_count\":0," +
                "\"output_path\":null,\"error\":\"Pose has not been received\"}"));

            Assert.That(state.CanStart, Is.True);
            Assert.That(state.CanStop, Is.False);
            Assert.That(state.Error, Does.Contain("Pose"));
        }

        [Test]
        public void VisibleBottomPanelBlocksOnlyItsScreenRegion()
        {
            var host = new GameObject("Clip Recording Control Test");
            host.SetActive(false);
            try
            {
                var control = host.AddComponent<ClipRecordingControlBehaviour>();

                Rect panel = control.ResolvePanelRect(600f);
                Assert.That(panel.y, Is.EqualTo(364f));
                Assert.That(control.ContainsScreenPoint(new Vector2(20f, 200f), 600f), Is.True);
                Assert.That(control.ContainsScreenPoint(new Vector2(700f, 200f), 600f), Is.False);
                Assert.That(control.ContainsScreenPoint(new Vector2(20f, 580f), 600f), Is.False);
            }
            finally
            {
                Object.DestroyImmediate(host);
            }
        }

        private static ClipControlResponse Response(string json)
        {
            return ClipControlProtocol.ParseResponse(json);
        }
    }
}
