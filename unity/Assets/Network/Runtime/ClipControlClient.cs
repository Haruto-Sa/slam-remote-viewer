using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;

namespace Slam.RemoteViewer.Network
{
    [JsonConverter(typeof(StringEnumConverter))]
    public enum ClipRecordingState
    {
        Idle,
        Recording,
        Finalizing,
        Completed,
        Failed
    }

    public sealed class ClipControlResponse
    {
        [JsonProperty("ok", Required = Required.Always)]
        public bool Ok { get; private set; }

        [JsonProperty("state", Required = Required.Always)]
        public ClipRecordingState State { get; private set; }

        [JsonProperty("session")]
        public string Session { get; private set; }

        [JsonProperty("elapsed_seconds", Required = Required.Always)]
        public double ElapsedSeconds { get; private set; }

        [JsonProperty("message_count", Required = Required.Always)]
        public ulong MessageCount { get; private set; }

        [JsonProperty("output_path")]
        public string OutputPath { get; private set; }

        [JsonProperty("error")]
        public string Error { get; private set; }

        public static ClipControlResponse Failure(string error)
        {
            return new ClipControlResponse
            {
                Ok = false,
                State = ClipRecordingState.Failed,
                Error = error
            };
        }
    }

    public static class ClipControlProtocol
    {
        public static string SerializeCommand(string command)
        {
            if (string.IsNullOrWhiteSpace(command))
            {
                throw new ArgumentException("command must not be empty", nameof(command));
            }
            return JsonConvert.SerializeObject(new { command });
        }

        public static ClipControlResponse ParseResponse(string payload)
        {
            if (string.IsNullOrWhiteSpace(payload))
            {
                throw new JsonSerializationException("clip control response must not be empty");
            }
            return JsonConvert.DeserializeObject<ClipControlResponse>(payload)
                ?? throw new JsonSerializationException("clip control response must not be null");
        }
    }

    public sealed class ClipControlClient : IDisposable
    {
        public const string DefaultEndpoint = "tcp://127.0.0.1:5557";

        private static readonly TimeSpan RequestTimeout = TimeSpan.FromMilliseconds(750);
        private static readonly TimeSpan PollInterval = TimeSpan.FromMilliseconds(250);
        private readonly string endpoint;
        private readonly ConcurrentQueue<string> commands = new ConcurrentQueue<string>();
        private readonly ConcurrentQueue<ClipControlResponse> responses =
            new ConcurrentQueue<ClipControlResponse>();
        private readonly AutoResetEvent wake = new AutoResetEvent(false);
        private readonly object lifecycleGate = new object();
        private CancellationTokenSource cancellation;
        private Thread thread;
        private bool disposed;

        public ClipControlClient(string endpoint)
        {
            if (string.IsNullOrWhiteSpace(endpoint))
            {
                throw new ArgumentException("endpoint must not be empty", nameof(endpoint));
            }
            this.endpoint = endpoint;
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
                    Name = "SLAM clip control client"
                };
                thread.Start();
            }
        }

        public void StartClip()
        {
            Enqueue("start_clip");
        }

        public void StopClip()
        {
            Enqueue("stop_clip");
        }

        public bool TryDequeueResponse(out ClipControlResponse response)
        {
            return responses.TryDequeue(out response);
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
                wake.Set();
                threadToJoin = thread;
            }

            if (!threadToJoin.Join(timeout))
            {
                throw new TimeoutException("clip control client did not stop before the timeout");
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
            wake.Dispose();
            lock (lifecycleGate)
            {
                disposed = true;
            }
        }

        private void Enqueue(string command)
        {
            lock (lifecycleGate)
            {
                ThrowIfDisposed();
            }
            commands.Enqueue(command);
            wake.Set();
        }

        private void Run(CancellationToken token)
        {
            AsyncIO.ForceDotNet.Force();
            while (!token.IsCancellationRequested)
            {
                string command;
                if (!commands.TryDequeue(out command))
                {
                    wake.WaitOne(PollInterval);
                    if (token.IsCancellationRequested)
                    {
                        break;
                    }
                    if (!commands.TryDequeue(out command))
                    {
                        command = "status";
                    }
                }

                responses.Enqueue(Execute(command));
            }
        }

        private ClipControlResponse Execute(string command)
        {
            try
            {
                using (var socket = new RequestSocket())
                {
                    socket.Options.Linger = TimeSpan.Zero;
                    socket.Connect(endpoint);
                    socket.SendFrame(ClipControlProtocol.SerializeCommand(command));
                    string payload;
                    if (!socket.TryReceiveFrameString(RequestTimeout, out payload))
                    {
                        throw new TimeoutException("Receiver clip-control request timed out");
                    }
                    return ClipControlProtocol.ParseResponse(payload);
                }
            }
            catch (Exception exception)
            {
                return ClipControlResponse.Failure(
                    exception.GetType().Name + ": " + exception.Message);
            }
        }

        private void ThrowIfDisposed()
        {
            if (disposed)
            {
                throw new ObjectDisposedException(nameof(ClipControlClient));
            }
        }
    }
}
