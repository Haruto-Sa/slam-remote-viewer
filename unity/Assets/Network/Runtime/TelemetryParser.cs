using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;

namespace Slam.RemoteViewer.Network
{
    public static class TelemetryParser
    {
        public const ulong MaximumPointId = 9007199254740991UL;

        private static readonly JsonSerializerSettings SerializerSettings = new JsonSerializerSettings
        {
            MissingMemberHandling = MissingMemberHandling.Ignore,
            FloatParseHandling = FloatParseHandling.Double,
            Converters = new List<JsonConverter> { new StringEnumConverter() }
        };

        public static bool TryParse(
            string topic,
            string payload,
            out ITelemetryMessage message,
            out string error)
        {
            message = null;
            error = null;

            if (string.IsNullOrEmpty(topic))
            {
                error = "topic must not be empty";
                return false;
            }

            if (payload == null)
            {
                error = "payload must not be null";
                return false;
            }

            try
            {
                switch (topic)
                {
                    case TelemetryTopics.Settings:
                        message = JsonConvert.DeserializeObject<SettingsMessage>(payload, SerializerSettings);
                        break;
                    case TelemetryTopics.Pose:
                        message = JsonConvert.DeserializeObject<PoseMessage>(payload, SerializerSettings);
                        break;
                    case TelemetryTopics.PointCloud:
                        message = JsonConvert.DeserializeObject<PointCloudMessage>(payload, SerializerSettings);
                        break;
                    default:
                        error = "unsupported topic: " + topic;
                        return false;
                }
            }
            catch (JsonException exception)
            {
                error = "invalid JSON for " + topic + ": " + exception.Message;
                return false;
            }
            catch (ArgumentException exception)
            {
                error = "invalid telemetry for " + topic + ": " + exception.Message;
                return false;
            }

            if (message == null)
            {
                error = "payload deserialized to null";
                return false;
            }

            return Validate(message, out error);
        }

        private static bool Validate(ITelemetryMessage message, out string error)
        {
            error = null;
            if (string.IsNullOrWhiteSpace(message.Session))
            {
                error = "session must not be empty";
                return false;
            }

            if (message is SettingsMessage settings)
            {
                if (settings.Version != 1 || settings.Unit != "m" || settings.Frame != "unity_world" ||
                    settings.PoseConvention != "Twc" || settings.QuaternionOrder != "xyzw" ||
                    settings.PointCloudMode != "delta" || settings.Camera == null)
                {
                    error = "settings do not match the Unity Protocol v1 contract";
                    return false;
                }

                return true;
            }

            if (message is PoseMessage pose)
            {
                if (pose.Version != 1 || !IsValidTimestamp(pose.TimestampSeconds) ||
                    !IsFiniteVector(pose.Position, 3) || !IsFiniteVector(pose.OrientationXyzw, 4))
                {
                    error = "pose does not match the Protocol v1 contract";
                    return false;
                }

                return true;
            }

            var pointCloud = (PointCloudMessage)message;
            if (pointCloud.Version != 1 || !IsValidTimestamp(pointCloud.TimestampSeconds) ||
                !AreFiniteEntries(pointCloud.Add) || !AreFiniteEntries(pointCloud.Update))
            {
                error = "point cloud does not match the Protocol v1 contract";
                return false;
            }

            foreach (ulong id in pointCloud.Remove)
            {
                if (id > MaximumPointId)
                {
                    error = "point cloud contains an out-of-range remove ID";
                    return false;
                }
            }

            return true;
        }

        private static bool IsValidTimestamp(double value)
        {
            return IsFinite(value) && value >= 0.0;
        }

        private static bool IsFiniteVector(IReadOnlyList<double> values, int expectedCount)
        {
            if (values == null || values.Count != expectedCount)
            {
                return false;
            }

            for (var index = 0; index < values.Count; index++)
            {
                if (!IsFinite(values[index]))
                {
                    return false;
                }
            }

            return true;
        }

        private static bool AreFiniteEntries(IReadOnlyList<PointEntry> entries)
        {
            if (entries == null)
            {
                return false;
            }

            foreach (PointEntry entry in entries)
            {
                if (entry.Id > MaximumPointId || !IsFinite(entry.X) || !IsFinite(entry.Y) || !IsFinite(entry.Z))
                {
                    return false;
                }
            }

            return true;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
