using System;
using System.Diagnostics;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using NUnit.Framework;

namespace Slam.RemoteViewer.Network.Tests
{
    public sealed class ClipControlProtocolTests
    {
        [Test]
        public void SerializesControlCommand()
        {
            Assert.That(
                ClipControlProtocol.SerializeCommand("start_clip"),
                Is.EqualTo("{\"command\":\"start_clip\"}"));
        }

        [Test]
        public void ParsesCompletedStatus()
        {
            ClipControlResponse response = ClipControlProtocol.ParseResponse(
                "{\"ok\":true,\"state\":\"completed\",\"session\":\"demo\"," +
                "\"elapsed_seconds\":2.5,\"message_count\":42," +
                "\"output_path\":\"recordings/clips/demo-clip-001\",\"error\":null}");

            Assert.That(response.Ok, Is.True);
            Assert.That(response.State, Is.EqualTo(ClipRecordingState.Completed));
            Assert.That(response.Session, Is.EqualTo("demo"));
            Assert.That(response.ElapsedSeconds, Is.EqualTo(2.5));
            Assert.That(response.MessageCount, Is.EqualTo(42));
            Assert.That(response.OutputPath, Does.EndWith("demo-clip-001"));
            Assert.That(response.Error, Is.Null);
        }

        [Test]
        public void RejectsMissingRequiredStatusFields()
        {
            Assert.Catch<Exception>(() =>
                ClipControlProtocol.ParseResponse("{\"ok\":true}"));
        }

        [Test]
        public void ClientSendsQueuedCommandAndReceivesResponse()
        {
            string endpoint = "inproc://slam-clip-control-" + Guid.NewGuid().ToString("N");
            AsyncIO.ForceDotNet.Force();
            using (var server = new ResponseSocket())
            using (var client = new ClipControlClient(endpoint))
            {
                server.Options.Linger = TimeSpan.Zero;
                server.Bind(endpoint);
                client.Start();
                client.StartClip();

                string request;
                Assert.That(
                    server.TryReceiveFrameString(TimeSpan.FromSeconds(2), out request),
                    Is.True);
                Assert.That(request, Is.EqualTo("{\"command\":\"start_clip\"}"));
                server.SendFrame(
                    "{\"ok\":true,\"state\":\"recording\",\"session\":\"demo\"," +
                    "\"elapsed_seconds\":0,\"message_count\":0," +
                    "\"output_path\":null,\"error\":null}");

                ClipControlResponse response = null;
                var timeout = Stopwatch.StartNew();
                while (response == null && timeout.Elapsed < TimeSpan.FromSeconds(2))
                {
                    client.TryDequeueResponse(out response);
                    Thread.Sleep(10);
                }

                Assert.That(response, Is.Not.Null);
                Assert.That(response.State, Is.EqualTo(ClipRecordingState.Recording));
            }
        }
    }
}
