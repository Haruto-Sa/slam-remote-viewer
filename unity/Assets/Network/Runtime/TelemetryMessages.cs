using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;

namespace Slam.RemoteViewer.Network
{
    public interface ITelemetryMessage
    {
        string Topic { get; }
        string Session { get; }
        ulong? Sequence { get; }
    }

    public static class TelemetryTopics
    {
        public const string Prefix = "slam/v1/";
        public const string Settings = Prefix + "settings";
        public const string Pose = Prefix + "pose";
        public const string PointCloud = Prefix + "pointcloud";
    }

    public sealed class CameraSettings
    {
        [JsonConstructor]
        public CameraSettings(string type, string id, uint width, uint height, uint fps)
        {
            Type = type;
            Id = id;
            Width = width;
            Height = height;
            FramesPerSecond = fps;
        }

        [JsonProperty("type", Required = Required.Always)]
        public string Type { get; }

        [JsonProperty("id", Required = Required.Always)]
        public string Id { get; }

        [JsonProperty("width", Required = Required.Always)]
        public uint Width { get; }

        [JsonProperty("height", Required = Required.Always)]
        public uint Height { get; }

        [JsonProperty("fps", Required = Required.Always)]
        public uint FramesPerSecond { get; }
    }

    public sealed class SettingsMessage : ITelemetryMessage
    {
        [JsonConstructor]
        public SettingsMessage(
            uint v,
            string session,
            string unit,
            string frame,
            string poseConvention,
            string quaternion,
            CameraSettings camera,
            string pointcloudMode)
        {
            Version = v;
            Session = session;
            Unit = unit;
            Frame = frame;
            PoseConvention = poseConvention;
            QuaternionOrder = quaternion;
            Camera = camera;
            PointCloudMode = pointcloudMode;
        }

        public string Topic => TelemetryTopics.Settings;
        public ulong? Sequence => null;

        [JsonProperty("v", Required = Required.Always)]
        public uint Version { get; }

        [JsonProperty("session", Required = Required.Always)]
        public string Session { get; }

        [JsonProperty("unit", Required = Required.Always)]
        public string Unit { get; }

        [JsonProperty("frame", Required = Required.Always)]
        public string Frame { get; }

        [JsonProperty("pose_convention", Required = Required.Always)]
        public string PoseConvention { get; }

        [JsonProperty("quaternion", Required = Required.Always)]
        public string QuaternionOrder { get; }

        [JsonProperty("camera", Required = Required.Always)]
        public CameraSettings Camera { get; }

        [JsonProperty("pointcloud_mode", Required = Required.Always)]
        public string PointCloudMode { get; }
    }

    [JsonConverter(typeof(StringEnumConverter))]
    public enum PoseTrackingState
    {
        Initializing,
        Tracking,
        Lost,
        Relocalizing
    }

    public sealed class PoseMessage : ITelemetryMessage
    {
        [JsonConstructor]
        public PoseMessage(
            uint v,
            string session,
            ulong seq,
            double t,
            double[] p,
            double[] q,
            PoseTrackingState state)
        {
            Version = v;
            Session = session;
            SequenceNumber = seq;
            TimestampSeconds = t;
            Position = CopyAsReadOnly(p);
            OrientationXyzw = CopyAsReadOnly(q);
            State = state;
        }

        public string Topic => TelemetryTopics.Pose;
        public ulong? Sequence => SequenceNumber;

        [JsonProperty("v", Required = Required.Always)]
        public uint Version { get; }

        [JsonProperty("session", Required = Required.Always)]
        public string Session { get; }

        [JsonProperty("seq", Required = Required.Always)]
        public ulong SequenceNumber { get; }

        [JsonProperty("t", Required = Required.Always)]
        public double TimestampSeconds { get; }

        [JsonProperty("p", Required = Required.Always)]
        public IReadOnlyList<double> Position { get; }

        [JsonProperty("q", Required = Required.Always)]
        public IReadOnlyList<double> OrientationXyzw { get; }

        [JsonProperty("state", Required = Required.Always)]
        public PoseTrackingState State { get; }

        private static ReadOnlyCollection<double> CopyAsReadOnly(double[] values)
        {
            if (values == null)
            {
                throw new JsonSerializationException("pose vector must not be null");
            }

            return Array.AsReadOnly((double[])values.Clone());
        }
    }

    public readonly struct PointEntry
    {
        public PointEntry(ulong id, double x, double y, double z)
        {
            Id = id;
            X = x;
            Y = y;
            Z = z;
        }

        public ulong Id { get; }
        public double X { get; }
        public double Y { get; }
        public double Z { get; }
    }

    public sealed class PointCloudMessage : ITelemetryMessage
    {
        [JsonConstructor]
        public PointCloudMessage(
            uint v,
            string session,
            ulong seq,
            double t,
            IList<IList<double>> add,
            IList<IList<double>> update,
            IList<ulong> remove)
        {
            Version = v;
            Session = session;
            SequenceNumber = seq;
            TimestampSeconds = t;
            Add = ConvertEntries(add, "add");
            Update = ConvertEntries(update, "update");
            Remove = new ReadOnlyCollection<ulong>(new List<ulong>(remove ?? throw new JsonSerializationException("remove must not be null")));
        }

        public string Topic => TelemetryTopics.PointCloud;
        public ulong? Sequence => SequenceNumber;

        [JsonProperty("v", Required = Required.Always)]
        public uint Version { get; }

        [JsonProperty("session", Required = Required.Always)]
        public string Session { get; }

        [JsonProperty("seq", Required = Required.Always)]
        public ulong SequenceNumber { get; }

        [JsonProperty("t", Required = Required.Always)]
        public double TimestampSeconds { get; }

        [JsonProperty("add", Required = Required.Always)]
        public IReadOnlyList<PointEntry> Add { get; }

        [JsonProperty("update", Required = Required.Always)]
        public IReadOnlyList<PointEntry> Update { get; }

        [JsonProperty("remove", Required = Required.Always)]
        public IReadOnlyList<ulong> Remove { get; }

        private static ReadOnlyCollection<PointEntry> ConvertEntries(IList<IList<double>> values, string field)
        {
            if (values == null)
            {
                throw new JsonSerializationException(field + " must not be null");
            }

            var entries = new List<PointEntry>(values.Count);
            foreach (IList<double> value in values)
            {
                if (value == null || value.Count != 4)
                {
                    throw new JsonSerializationException(field + " entries must contain [id, x, y, z]");
                }

                double rawId = value[0];
                if (!IsFinite(rawId) || rawId < 0.0 || Math.Truncate(rawId) != rawId || rawId > TelemetryParser.MaximumPointId)
                {
                    throw new JsonSerializationException(field + " contains an invalid point ID");
                }

                entries.Add(new PointEntry((ulong)rawId, value[1], value[2], value[3]));
            }

            return entries.AsReadOnly();
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
