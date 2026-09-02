using System;
using Slam.RemoteViewer.Network;

namespace Slam.RemoteViewer
{
    public sealed class ClipRecordingViewState
    {
        public ClipRecordingState State { get; private set; } = ClipRecordingState.Idle;
        public string Session { get; private set; }
        public double ElapsedSeconds { get; private set; }
        public ulong MessageCount { get; private set; }
        public string OutputPath { get; private set; }
        public string Error { get; private set; }

        public bool CanStart =>
            State == ClipRecordingState.Idle ||
            State == ClipRecordingState.Completed ||
            State == ClipRecordingState.Failed;
        public bool CanStop => State == ClipRecordingState.Recording;

        public void Apply(ClipControlResponse response)
        {
            if (response == null)
            {
                throw new ArgumentNullException(nameof(response));
            }

            State = response.State;
            Session = response.Session;
            ElapsedSeconds = Math.Max(0d, response.ElapsedSeconds);
            MessageCount = response.MessageCount;
            OutputPath = response.OutputPath;
            Error = response.Error;
        }
    }
}
