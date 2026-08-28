using System;
using System.Text;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

namespace Slam.RemoteViewer.Network
{
    public sealed class TelemetrySubscriber : IDisposable
    {
        public const string DefaultEndpoint = "tcp://127.0.0.1:5556";

        private static readonly Encoding StrictUtf8 = new UTF8Encoding(false, true);
        private static readonly TimeSpan ReceiveTimeout = TimeSpan.FromMilliseconds(100);
        private readonly object lifecycleGate = new object();
        private readonly string endpoint;
        private readonly TelemetryIngress ingress;
        private CancellationTokenSource cancellation;
        private Thread thread;
        private bool disposed;
        private long faultCount;

        public TelemetrySubscriber(string endpoint, TelemetryIngress ingress)
        {
            if (string.IsNullOrWhiteSpace(endpoint))
            {
                throw new ArgumentException("endpoint must not be empty", nameof(endpoint));
            }

            this.endpoint = endpoint;
            this.ingress = ingress ?? throw new ArgumentNullException(nameof(ingress));
        }

        public bool IsRunning
        {
            get
            {
                lock (lifecycleGate)
                {
                    return thread != null && thread.IsAlive;
                }
            }
        }

        public long FaultCount => Interlocked.Read(ref faultCount);
        public string LastFault { get; private set; }

        public void Start()
        {
            lock (lifecycleGate)
            {
                ThrowIfDisposed();
                if (thread != null && thread.IsAlive)
                {
                    return;
                }

                cancellation = new CancellationTokenSource();
                thread = new Thread(() => Run(cancellation.Token))
                {
                    IsBackground = true,
                    Name = "SLAM telemetry subscriber"
                };
                thread.Start();
            }
        }

        public void Stop(TimeSpan timeout)
        {
            Thread threadToJoin;
            lock (lifecycleGate)
            {
                if (thread == null)
                {
                    return;
                }

                cancellation.Cancel();
                threadToJoin = thread;
            }

            if (!threadToJoin.Join(timeout))
            {
                throw new TimeoutException("telemetry subscriber did not stop before the timeout");
            }

            lock (lifecycleGate)
            {
                thread = null;
                cancellation.Dispose();
                cancellation = null;
            }
        }

        public void Dispose()
        {
            lock (lifecycleGate)
            {
                if (disposed)
                {
                    return;
                }
            }

            Stop(TimeSpan.FromSeconds(2));
            NetMQConfig.Cleanup(false);

            lock (lifecycleGate)
            {
                disposed = true;
            }
        }

        private void Run(CancellationToken token)
        {
            try
            {
                AsyncIO.ForceDotNet.Force();
                using (var socket = new SubscriberSocket())
                {
                    socket.Options.Linger = TimeSpan.Zero;
                    socket.Connect(endpoint);
                    socket.Subscribe(TelemetryTopics.Prefix);

                    while (!token.IsCancellationRequested)
                    {
                        NetMQMessage multipart = null;
                        if (!socket.TryReceiveMultipartMessage(ReceiveTimeout, ref multipart, 2))
                        {
                            continue;
                        }

                        HandleMultipart(multipart);
                    }
                }
            }
            catch (Exception exception)
            {
                LastFault = exception.GetType().Name + ": " + exception.Message;
                Interlocked.Increment(ref faultCount);
            }
        }

        private void HandleMultipart(NetMQMessage multipart)
        {
            if (multipart == null || multipart.FrameCount != 2)
            {
                ingress.Reject("expected exactly two multipart frames");
                return;
            }

            try
            {
                string topic = StrictUtf8.GetString(multipart[0].ToByteArray());
                string payload = StrictUtf8.GetString(multipart[1].ToByteArray());
                ingress.TryAccept(topic, payload);
            }
            catch (DecoderFallbackException)
            {
                ingress.Reject("topic or payload is not valid UTF-8");
            }
        }

        private void ThrowIfDisposed()
        {
            if (disposed)
            {
                throw new ObjectDisposedException(nameof(TelemetrySubscriber));
            }
        }
    }
}
