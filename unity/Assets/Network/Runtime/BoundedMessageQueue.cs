using System;
using System.Collections.Generic;

namespace Slam.RemoteViewer.Network
{
    public sealed class BoundedMessageQueue<T>
    {
        private readonly object gate = new object();
        private readonly Queue<T> queue;
        private long droppedCount;

        public BoundedMessageQueue(int capacity)
        {
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity), "capacity must be positive");
            }

            Capacity = capacity;
            queue = new Queue<T>(capacity);
        }

        public int Capacity { get; }

        public int Count
        {
            get
            {
                lock (gate)
                {
                    return queue.Count;
                }
            }
        }

        public long DroppedCount
        {
            get
            {
                lock (gate)
                {
                    return droppedCount;
                }
            }
        }

        public bool Enqueue(T message)
        {
            bool dropped = false;
            lock (gate)
            {
                if (queue.Count == Capacity)
                {
                    queue.Dequeue();
                    droppedCount++;
                    dropped = true;
                }

                queue.Enqueue(message);
            }

            return dropped;
        }

        public bool TryDequeue(out T message)
        {
            lock (gate)
            {
                if (queue.Count == 0)
                {
                    message = default;
                    return false;
                }

                message = queue.Dequeue();
                return true;
            }
        }
    }
}
