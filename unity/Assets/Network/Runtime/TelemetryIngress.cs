using System;
using System.Threading;

namespace Slam.RemoteViewer.Network
{
    public sealed class TelemetryIngress
    {
        private readonly BoundedMessageQueue<ITelemetryMessage> queue;
        private string activeSession;
        private long acceptedCount;
        private long rejectedCount;

        public TelemetryIngress(BoundedMessageQueue<ITelemetryMessage> queue)
        {
            this.queue = queue ?? throw new ArgumentNullException(nameof(queue));
        }

        public long AcceptedCount => Interlocked.Read(ref acceptedCount);
        public long RejectedCount => Interlocked.Read(ref rejectedCount);
        public long DroppedCount => queue.DroppedCount;
        public string LastRejectionReason { get; private set; }

        public bool TryAccept(string topic, string payload)
        {
            if (!TelemetryParser.TryParse(topic, payload, out ITelemetryMessage message, out string error))
            {
                Reject(error);
                return false;
            }

            if (message is SettingsMessage settings)
            {
                activeSession = settings.Session;
            }
            else if (!string.Equals(activeSession, message.Session, StringComparison.Ordinal))
            {
                Reject("telemetry arrived before matching unity_world settings");
                return false;
            }

            queue.Enqueue(message);
            Interlocked.Increment(ref acceptedCount);
            return true;
        }

        public void Reject(string reason)
        {
            LastRejectionReason = reason ?? "unknown rejection";
            Interlocked.Increment(ref rejectedCount);
        }
    }
}
