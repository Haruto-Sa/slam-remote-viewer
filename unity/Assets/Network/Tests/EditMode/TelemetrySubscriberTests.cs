using System;
using System.Diagnostics;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using NUnit.Framework;

namespace Slam.RemoteViewer.Network.Tests
{
    public sealed class TelemetrySubscriberTests
    {
        [Test]
        public void ReceivesAllTopicsAndStopsCleanly()
        {
            string endpoint = "inproc://slam-unity-subscriber-" + Guid.NewGuid().ToString("N");
            var queue = new BoundedMessageQueue<ITelemetryMessage>(8);
            var ingress = new TelemetryIngress(queue);
            var subscriber = new TelemetrySubscriber(endpoint, ingress);

            AsyncIO.ForceDotNet.Force();
            using (var publisher = new PublisherSocket())
            {
                publisher.Options.Linger = TimeSpan.Zero;
                publisher.Bind(endpoint);
                subscriber.Start();
                Thread.Sleep(100);

                publisher.SendMoreFrame(TelemetryTopics.Pose)
                    .SendMoreFrame(TelemetryParserTests.Pose)
                    .SendFrame("unexpected-frame");
                Publish(publisher, TelemetryTopics.Settings, TelemetryParserTests.UnitySettings);
                Publish(publisher, TelemetryTopics.Pose, TelemetryParserTests.Pose);
                Publish(publisher, TelemetryTopics.PointCloud, TelemetryParserTests.PointCloud);

                var timeout = Stopwatch.StartNew();
                while (queue.Count < 3 && timeout.Elapsed < TimeSpan.FromSeconds(2))
                {
                    Thread.Sleep(10);
                }

                subscriber.Stop(TimeSpan.FromSeconds(2));

                Assert.That(queue.Count, Is.EqualTo(3));
                Assert.That(ingress.AcceptedCount, Is.EqualTo(3));
                Assert.That(ingress.RejectedCount, Is.EqualTo(1));
                Assert.That(subscriber.FaultCount, Is.EqualTo(0), subscriber.LastFault);
                Assert.That(subscriber.IsRunning, Is.False);
            }

            subscriber.Dispose();
        }

        private static void Publish(PublisherSocket publisher, string topic, string payload)
        {
            publisher.SendMoreFrame(topic).SendFrame(payload);
        }
    }
}
