using System;
using Slam.RemoteViewer.Network;
using UnityEngine;

namespace Slam.RemoteViewer
{
    public sealed class TelemetrySubscriberBehaviour : MonoBehaviour
    {
        [SerializeField]
        private string endpoint = TelemetrySubscriber.DefaultEndpoint;

        [SerializeField, Min(1)]
        private int queueCapacity = 1024;

        [SerializeField, Min(1)]
        private int maxMessagesPerFrame = 256;

        private BoundedMessageQueue<ITelemetryMessage> queue;
        private TelemetryIngress ingress;
        private TelemetrySubscriber subscriber;

        public event Action<ITelemetryMessage> MessageReceived;

        public string Endpoint => endpoint;
        public bool IsRunning => subscriber?.IsRunning ?? false;
        public long AcceptedCount => ingress?.AcceptedCount ?? 0;
        public long RejectedCount => ingress?.RejectedCount ?? 0;
        public long DroppedCount => ingress?.DroppedCount ?? 0;
        public int QueueCount => queue?.Count ?? 0;
        public long FaultCount => subscriber?.FaultCount ?? 0;
        public string LastFault => subscriber?.LastFault;
        public string LastRejectionReason => ingress?.LastRejectionReason;

        private void OnEnable()
        {
            queue = new BoundedMessageQueue<ITelemetryMessage>(Math.Max(1, queueCapacity));
            ingress = new TelemetryIngress(queue);
            subscriber = new TelemetrySubscriber(endpoint, ingress);
            subscriber.Start();
        }

        private void Update()
        {
            var remaining = Math.Max(1, maxMessagesPerFrame);
            while (remaining-- > 0 && queue.TryDequeue(out ITelemetryMessage message))
            {
                MessageReceived?.Invoke(message);
            }
        }

        private void OnDisable()
        {
            if (subscriber == null)
            {
                return;
            }

            try
            {
                subscriber.Dispose();
            }
            catch (Exception exception)
            {
                Debug.LogError("Failed to stop telemetry subscriber: " + exception.Message, this);
            }
            finally
            {
                subscriber = null;
            }
        }
    }
}
