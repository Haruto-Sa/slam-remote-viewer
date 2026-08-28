using System;
using NUnit.Framework;

namespace Slam.RemoteViewer.Network.Tests
{
    public sealed class BoundedMessageQueueTests
    {
        [Test]
        public void RejectsNonPositiveCapacity()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new BoundedMessageQueue<int>(0));
        }

        [Test]
        public void DropsOldestMessageWhenFull()
        {
            var queue = new BoundedMessageQueue<int>(2);

            Assert.That(queue.Enqueue(1), Is.False);
            Assert.That(queue.Enqueue(2), Is.False);
            Assert.That(queue.Enqueue(3), Is.True);

            Assert.That(queue.Count, Is.EqualTo(2));
            Assert.That(queue.DroppedCount, Is.EqualTo(1));
            Assert.That(queue.TryDequeue(out int first), Is.True);
            Assert.That(first, Is.EqualTo(2));
            Assert.That(queue.TryDequeue(out int second), Is.True);
            Assert.That(second, Is.EqualTo(3));
            Assert.That(queue.TryDequeue(out _), Is.False);
        }
    }
}
