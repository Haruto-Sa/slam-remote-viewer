use std::fmt;

use serde::{Deserialize, Serialize};

pub const SETTINGS_TOPIC: &str = "slam/v1/settings";
pub const POSE_TOPIC: &str = "slam/v1/pose";
pub const POINTCLOUD_TOPIC: &str = "slam/v1/pointcloud";

pub const MAX_POINT_ID: u64 = 9_007_199_254_740_991;

#[derive(Debug, Deserialize, Serialize, PartialEq, Eq)]
pub struct SettingsMessage {
    pub v: u32,
    pub session: String,
    pub unit: String,
    pub frame: String,
    pub pose_convention: String,
    pub quaternion: String,
    pub camera: CameraSettings,
    pub pointcloud_mode: String,
}

#[derive(Debug, Deserialize, Serialize, PartialEq, Eq)]
pub struct CameraSettings {
    #[serde(rename = "type")]
    pub camera_type: String,
    pub id: String,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
}

#[derive(Debug, Deserialize, Serialize, PartialEq)]
pub struct PoseMessage {
    pub v: u32,
    pub session: String,
    pub seq: u64,
    pub t: f64,
    pub p: [f64; 3],
    pub q: [f64; 4],
    pub state: PoseState,
}

#[derive(Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum PoseState {
    Initializing,
    Tracking,
    Lost,
    Relocalizing,
}

pub type PointEntry = (u64, f64, f64, f64);

#[derive(Debug, Deserialize, Serialize, PartialEq)]
pub struct PointCloudMessage {
    pub v: u32,
    pub session: String,
    pub seq: u64,
    pub t: f64,
    pub add: Vec<PointEntry>,
    pub update: Vec<PointEntry>,
    pub remove: Vec<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ValidationError {
    UnsupportedVersion {
        actual: u32,
    },
    EmptySession,
    InvalidFixedValue {
        field: &'static str,
        expected: &'static str,
        actual: String,
    },
    InvalidTimestamp,
    NonFiniteNumber {
        field: &'static str,
    },
    PointIdOutOfRange {
        field: &'static str,
        id: u64,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    UnsupportedTopic {
        topic: String,
    },
    InvalidPayload {
        topic: String,
        reason: String,
    },
    Validation {
        topic: String,
        source: ValidationError,
    },
}

impl fmt::Display for ParseError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedTopic { topic } => {
                write!(formatter, "unsupported topic: {topic}")
            }
            Self::InvalidPayload { topic, reason } => {
                write!(formatter, "invalid payload for {topic}: {reason}")
            }
            Self::Validation { topic, source } => {
                write!(formatter, "validation failed for {topic}: {source}")
            }
        }
    }
}

impl std::error::Error for ParseError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Validation { source, .. } => Some(source),
            _ => None,
        }
    }
}

impl fmt::Display for ValidationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedVersion { actual } => {
                write!(formatter, "unsupported protocol version: {actual}")
            }
            Self::EmptySession => {
                write!(formatter, "session must not be empty")
            }
            Self::InvalidFixedValue {
                field,
                expected,
                actual,
            } => {
                write!(
                    formatter,
                    "{field} must be {expected:?}, received {actual:?}"
                )
            }
            Self::InvalidTimestamp => {
                write!(formatter, "t must be finite and non-negative")
            }
            Self::NonFiniteNumber { field } => {
                write!(formatter, "{field} must contain only finite numbers")
            }
            Self::PointIdOutOfRange { field, id } => {
                write!(
                    formatter,
                    "{field} contains point ID {id}, which exceeds {MAX_POINT_ID}"
                )
            }
        }
    }
}

impl std::error::Error for ValidationError {}

impl SettingsMessage {
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_common(self.v, &self.session)?;

        validate_fixed_value("unit", &self.unit, "m")?;
        validate_fixed_value("frame", &self.frame, "slam_world")?;
        validate_fixed_value("pose_convention", &self.pose_convention, "Twc")?;
        validate_fixed_value("quaternion", &self.quaternion, "xyzw")?;
        validate_fixed_value("pointcloud_mode", &self.pointcloud_mode, "delta")?;

        Ok(())
    }
}

impl PoseMessage {
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_common(self.v, &self.session)?;

        validate_timestamp(self.t)?;

        validate_finite_numbers("p", &self.p)?;
        validate_finite_numbers("q", &self.q)?;

        Ok(())
    }
}

impl PointCloudMessage {
    pub fn validate(&self) -> Result<(), ValidationError> {
        validate_common(self.v, &self.session)?;
        validate_timestamp(self.t)?;

        validate_point_entries("add", &self.add)?;
        validate_point_entries("update", &self.update)?;

        for &id in &self.remove {
            validate_point_id("remove", id)?;
        }
        Ok(())
    }
}

#[derive(Debug, PartialEq)]
pub enum TelemetryMessage {
    Settings(SettingsMessage),
    Pose(PoseMessage),
    PointCloud(PointCloudMessage),
}

impl TelemetryMessage {
    pub fn topic(&self) -> &'static str {
        match self {
            Self::Settings(_) => SETTINGS_TOPIC,
            Self::Pose(_) => POSE_TOPIC,
            Self::PointCloud(_) => POINTCLOUD_TOPIC,
        }
    }

    pub fn session(&self) -> &str {
        match self {
            Self::Settings(message) => &message.session,
            Self::Pose(message) => &message.session,
            Self::PointCloud(message) => &message.session,
        }
    }

    pub fn seq(&self) -> Option<u64> {
        match self {
            Self::Settings(_) => None,
            Self::Pose(message) => Some(message.seq),
            Self::PointCloud(message) => Some(message.seq),
        }
    }
}

fn validate_common(version: u32, session: &str) -> Result<(), ValidationError> {
    if version != 1 {
        return Err(ValidationError::UnsupportedVersion { actual: version });
    }
    if session.trim().is_empty() {
        return Err(ValidationError::EmptySession);
    }

    Ok(())
}

fn validate_fixed_value(
    field: &'static str,
    actual: &str,
    expected: &'static str,
) -> Result<(), ValidationError> {
    if actual != expected {
        return Err(ValidationError::InvalidFixedValue {
            field,
            expected,
            actual: actual.to_owned(),
        });
    }

    Ok(())
}

fn validate_finite_numbers<const N: usize>(
    field: &'static str,
    values: &[f64; N],
) -> Result<(), ValidationError> {
    if values.iter().any(|value| !value.is_finite()) {
        return Err(ValidationError::NonFiniteNumber { field });
    }
    Ok(())
}

fn validate_timestamp(timestamp: f64) -> Result<(), ValidationError> {
    if !timestamp.is_finite() || timestamp < 0.0 {
        return Err(ValidationError::InvalidTimestamp);
    }
    Ok(())
}

fn validate_point_entries(
    field: &'static str,
    entries: &[PointEntry],
) -> Result<(), ValidationError> {
    for &(id, x, y, z) in entries {
        validate_point_id(field, id)?;
        validate_finite_numbers(field, &[x, y, z])?;
    }

    Ok(())
}

fn validate_point_id(field: &'static str, id: u64) -> Result<(), ValidationError> {
    if id > MAX_POINT_ID {
        return Err(ValidationError::PointIdOutOfRange { field, id });
    }

    Ok(())
}

pub fn parse_telemetry(topic: &str, payload: &str) -> Result<TelemetryMessage, ParseError> {
    match topic {
        SETTINGS_TOPIC => {
            let message: SettingsMessage = parse_json(topic, payload)?;
            message
                .validate()
                .map_err(|source| ParseError::Validation {
                    topic: topic.to_owned(),
                    source,
                })?;

            Ok(TelemetryMessage::Settings(message))
        }
        POSE_TOPIC => {
            let message: PoseMessage = parse_json(topic, payload)?;

            message
                .validate()
                .map_err(|source| ParseError::Validation {
                    topic: topic.to_owned(),
                    source,
                })?;
            Ok(TelemetryMessage::Pose(message))
        }
        POINTCLOUD_TOPIC => {
            let message: PointCloudMessage = parse_json(topic, payload)?;
            message
                .validate()
                .map_err(|source| ParseError::Validation {
                    topic: topic.to_owned(),
                    source,
                })?;
            Ok(TelemetryMessage::PointCloud(message))
        }
        _ => Err(ParseError::UnsupportedTopic {
            topic: topic.to_owned(),
        }),
    }
}

fn parse_json<T>(topic: &str, payload: &str) -> Result<T, ParseError>
where
    T: serde::de::DeserializeOwned,
{
    serde_json::from_str(payload).map_err(|error| ParseError::InvalidPayload {
        topic: topic.to_owned(),
        reason: error.to_string(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const SETTINGS_EXAMPLE: &str = include_str!("../../protocol/settings.example.json");

    const POSE_EXAMPLE: &str = include_str!("../../protocol/pose.example.json");

    const POINTCLOUD_EXAMPLE: &str = include_str!("../../protocol/pointcloud.example.json");

    #[test]
    fn deserializes_protocol_v1_settings_example() {
        let settings: SettingsMessage =
            serde_json::from_str(SETTINGS_EXAMPLE).expect("settings example should deserialize");

        assert_eq!(settings.v, 1);
        assert_eq!(settings.session, "001");
        assert_eq!(settings.unit, "m");
        assert_eq!(settings.frame, "slam_world");
        assert_eq!(settings.pose_convention, "Twc");
        assert_eq!(settings.quaternion, "xyzw");
        assert_eq!(settings.pointcloud_mode, "delta");

        assert_eq!(settings.camera.camera_type, "pc");
        assert_eq!(settings.camera.id, "builtin_0");
        assert_eq!(settings.camera.width, 1280);
        assert_eq!(settings.camera.height, 720);
        assert_eq!(settings.camera.fps, 30);
    }

    #[test]
    fn ignores_unknown_settings_fields() {
        let mut value: serde_json::Value =
            serde_json::from_str(SETTINGS_EXAMPLE).expect("settings example should be valid JSON");

        value["future_field"] = serde_json::json!(true);
        value["camera"]["future_camera_field"] = serde_json::json!("ignored");

        let settings: SettingsMessage =
            serde_json::from_value(value).expect("unknown fields should be ignored");

        assert_eq!(settings.v, 1);
        assert_eq!(settings.camera.camera_type, "pc");
    }

    #[test]
    fn validates_protocol_v1_settings_example() {
        let settings: SettingsMessage =
            serde_json::from_str(SETTINGS_EXAMPLE).expect("settings example should deserialize");

        assert_eq!(settings.validate(), Ok(()));
    }

    #[test]
    fn rejects_unsupported_settings_version() {
        let mut value: serde_json::Value =
            serde_json::from_str(SETTINGS_EXAMPLE).expect("settings example should be valid JSON");

        value["v"] = serde_json::json!(2);

        let settings: SettingsMessage =
            serde_json::from_value(value).expect("settings should deserialize");

        assert_eq!(
            settings.validate(),
            Err(ValidationError::UnsupportedVersion { actual: 2 })
        );
    }

    #[test]
    fn rejects_empty_settings_session() {
        let mut value: serde_json::Value =
            serde_json::from_str(SETTINGS_EXAMPLE).expect("settings example should be valid JSON");

        value["session"] = serde_json::json!("   ");

        let settings: SettingsMessage =
            serde_json::from_value(value).expect("settings should deserialize");

        assert_eq!(settings.validate(), Err(ValidationError::EmptySession));
    }

    #[test]
    fn rejects_invalid_fixed_settings_values() {
        let cases = [
            ("unit", "cm", "m"),
            ("frame", "camera_world", "slam_world"),
            ("pose_convention", "Tcw", "Twc"),
            ("quaternion", "wxyz", "xyzw"),
            ("pointcloud_mode", "snapshot", "delta"),
        ];

        for (field, actual, expected) in cases {
            let mut value: serde_json::Value = serde_json::from_str(SETTINGS_EXAMPLE)
                .expect("settings example should be valid JSON");

            value[field] = serde_json::json!(actual);

            let settings: SettingsMessage =
                serde_json::from_value(value).expect("settings should deserialize");

            assert_eq!(
                settings.validate(),
                Err(ValidationError::InvalidFixedValue {
                    field,
                    expected,
                    actual: actual.to_owned(),
                }),
                "field {field} should be rejected"
            );
        }
    }

    #[test]
    #[allow(clippy::approx_constant)] // Assert the truncated value in the wire fixture exactly.
    fn deserializes_protocol_v1_pose_example() {
        let pose: PoseMessage =
            serde_json::from_str(POSE_EXAMPLE).expect("pose example should deserialize");

        assert_eq!(pose.v, 1);
        assert_eq!(pose.session, "001");
        assert_eq!(pose.seq, 1523);
        assert_eq!(pose.t, 123.456789);
        assert_eq!(pose.p, [1.2, 0.4, 2.1]);
        assert_eq!(pose.q, [0.0, 0.7071068, 0.0, 0.7071068]);
        assert_eq!(pose.state, PoseState::Tracking);
    }

    #[test]
    fn validates_protocol_v1_pose_example() {
        let pose: PoseMessage =
            serde_json::from_str(POSE_EXAMPLE).expect("pose example should deserialize");

        assert_eq!(pose.validate(), Ok(()));
    }

    #[test]
    fn rejects_negative_pose_timestamp() {
        let mut value: serde_json::Value =
            serde_json::from_str(POSE_EXAMPLE).expect("pose example should be valid JSON");

        value["t"] = serde_json::json!(-0.1);
        let pose: PoseMessage = serde_json::from_value(value).expect("pose should deserialize");

        assert_eq!(pose.validate(), Err(ValidationError::InvalidTimestamp));
    }

    #[test]
    fn rejects_invalid_pose_shapes_and_sequence_type() {
        let cases = [
            ("seq", serde_json::json!(-1)),
            ("p", serde_json::json!([1.0, 2.0])),
            ("q", serde_json::json!([0.0, 0.0, 1.0])),
        ];

        for (field, invalid_value) in cases {
            let mut value: serde_json::Value =
                serde_json::from_str(POSE_EXAMPLE).expect("pose example should be valid JSON");

            value[field] = invalid_value;

            let result = serde_json::from_value::<PoseMessage>(value);

            assert!(result.is_err(), "field {field} should be rejected");
        }
    }

    #[test]
    fn rejects_unknown_pose_state() {
        let mut value: serde_json::Value =
            serde_json::from_str(POSE_EXAMPLE).expect("pose example should be valid JSON");

        value["state"] = serde_json::json!("paused");

        let result = serde_json::from_value::<PoseMessage>(value);

        assert!(result.is_err());
    }

    #[test]
    fn rejects_non_finite_pose_numbers() {
        let mut pose: PoseMessage =
            serde_json::from_str(POSE_EXAMPLE).expect("pose example should deserialize");

        pose.p[1] = f64::NAN;

        assert_eq!(
            pose.validate(),
            Err(ValidationError::NonFiniteNumber { field: "p" })
        );

        let mut pose: PoseMessage =
            serde_json::from_str(POSE_EXAMPLE).expect("pose example should deserialize");

        pose.q[3] = f64::INFINITY;

        assert_eq!(
            pose.validate(),
            Err(ValidationError::NonFiniteNumber { field: "q" })
        );
    }

    #[test]
    fn deserializes_protocol_v1_pointcloud_example() {
        let pointcloud: PointCloudMessage = serde_json::from_str(POINTCLOUD_EXAMPLE)
            .expect("point-cloud example should deserialize");

        assert_eq!(pointcloud.v, 1);
        assert_eq!(pointcloud.session, "001");
        assert_eq!(pointcloud.seq, 82);
        assert_eq!(pointcloud.t, 123.5);
        assert_eq!(
            pointcloud.add,
            vec![(1001, 0.1, 0.2, 1.4), (1002, 0.2, 0.3, 1.5),]
        );
        assert!(pointcloud.update.is_empty());
        assert!(pointcloud.remove.is_empty());
    }

    #[test]
    fn rejects_invalid_pointcloud_entry_shapes_and_types() {
        let cases = [
            ("add", serde_json::json!([[1001, 0.1, 0.2]])),
            ("update", serde_json::json!([[1001, 0.1, 0.2, 0.3, 0.4]])),
            ("remove", serde_json::json!([-1])),
        ];
        for (field, invalid_value) in cases {
            let mut value: serde_json::Value = serde_json::from_str(POINTCLOUD_EXAMPLE)
                .expect("point-cloud example should be valid JSON");

            value[field] = invalid_value;

            let result = serde_json::from_value::<PointCloudMessage>(value);

            assert!(result.is_err(), "field {field} should be rejected");
        }
    }

    #[test]
    fn validates_protocol_v1_pointcloud_example() {
        let pointcloud: PointCloudMessage = serde_json::from_str(POINTCLOUD_EXAMPLE)
            .expect("point-cloud example should deserialize");

        assert_eq!(pointcloud.validate(), Ok(()));
    }

    #[test]
    fn rejects_invalid_pointcloud_timestamp() {
        let mut pointcloud: PointCloudMessage = serde_json::from_str(POINTCLOUD_EXAMPLE)
            .expect("point-cloud example should deserialize");

        pointcloud.t = -0.1;

        assert_eq!(
            pointcloud.validate(),
            Err(ValidationError::InvalidTimestamp),
        );

        pointcloud.t = f64::INFINITY;

        assert_eq!(
            pointcloud.validate(),
            Err(ValidationError::InvalidTimestamp),
        );
    }

    #[test]
    fn rejects_point_ids_above_json_safe_integer_limit() {
        let invalid_id = MAX_POINT_ID + 1;

        let cases = [
            ("add", serde_json::json!([[invalid_id, 0.1, 0.2, 0.3]])),
            ("update", serde_json::json!([[invalid_id, 0.1, 0.2, 0.3]])),
            ("remove", serde_json::json!([invalid_id])),
        ];
        for (field, invalid_value) in cases {
            let mut value: serde_json::Value = serde_json::from_str(POINTCLOUD_EXAMPLE)
                .expect("point-cloud example should be valid JSON");

            value[field] = invalid_value;

            let pointcloud: PointCloudMessage =
                serde_json::from_value(value).expect("point-cloud message should deserialize");

            assert_eq!(
                pointcloud.validate(),
                Err(ValidationError::PointIdOutOfRange {
                    field,
                    id: invalid_id
                }),
                "field {field} should be rejected"
            );
        }
    }

    #[test]
    fn rejects_non_finite_point_coordinates() {
        let mut pointcloud: PointCloudMessage = serde_json::from_str(POINTCLOUD_EXAMPLE)
            .expect("point-cloud example should deserialize");

        pointcloud.add[0].2 = f64::NAN;

        assert_eq!(
            pointcloud.validate(),
            Err(ValidationError::NonFiniteNumber { field: "add" })
        );

        let mut pointcloud: PointCloudMessage = serde_json::from_str(POINTCLOUD_EXAMPLE)
            .expect("point-cloud example should deserialize");

        pointcloud.update.push((2001, 0.1, f64::INFINITY, 0.3));

        assert_eq!(
            pointcloud.validate(),
            Err(ValidationError::NonFiniteNumber { field: "update" })
        );
    }

    #[test]
    fn parses_all_protocol_v1_topics() {
        assert!(matches!(
            parse_telemetry(SETTINGS_TOPIC, SETTINGS_EXAMPLE),
            Ok(TelemetryMessage::Settings(_))
        ));

        assert!(matches!(
            parse_telemetry(POSE_TOPIC, POSE_EXAMPLE),
            Ok(TelemetryMessage::Pose(_))
        ));
        assert!(matches!(
            parse_telemetry(POINTCLOUD_TOPIC, POINTCLOUD_EXAMPLE),
            Ok(TelemetryMessage::PointCloud(_))
        ));
    }

    #[test]
    fn rejects_unsupported_protocol_v1_topic() {
        let error = parse_telemetry("slam/v1/future", r#"{"v":1,"session":"001"}"#)
            .expect_err("unsupported topic should be rejected");

        assert_eq!(
            error,
            ParseError::UnsupportedTopic {
                topic: "slam/v1/future".to_owned(),
            }
        );
    }

    #[test]
    fn reports_topic_when_payload_cannot_be_deserialized() {
        let error = parse_telemetry(POSE_TOPIC, r#"{"v":1,"session":"001"}"#)
            .expect_err("incomplete pose should be rejected");

        assert!(matches!(
            error,
            ParseError::InvalidPayload { topic, .. }
                if topic == POSE_TOPIC
        ));
    }

    #[test]
    fn reports_topic_for_invalid_json_syntax() {
        let error = parse_telemetry(POSE_TOPIC, "{not-json}")
            .expect_err("invalid JSON syntax should be rejected");

        assert!(matches!(
            error,
            ParseError::InvalidPayload { topic, .. }
                if topic == POSE_TOPIC
        ));
    }

    #[test]
    fn exposes_common_telemetry_metadata() {
        let settings = parse_telemetry(SETTINGS_TOPIC, SETTINGS_EXAMPLE)
            .expect("settings example should parse");
        let pose = parse_telemetry(POSE_TOPIC, POSE_EXAMPLE).expect("pose example should parse");
        let pointcloud = parse_telemetry(POINTCLOUD_TOPIC, POINTCLOUD_EXAMPLE)
            .expect("point-cloud example should parse");

        assert_eq!(settings.topic(), SETTINGS_TOPIC);
        assert_eq!(settings.session(), "001");
        assert_eq!(settings.seq(), None);

        assert_eq!(pose.topic(), POSE_TOPIC);
        assert_eq!(pose.session(), "001");
        assert_eq!(pose.seq(), Some(1523));

        assert_eq!(pointcloud.topic(), POINTCLOUD_TOPIC);
        assert_eq!(pointcloud.session(), "001");
        assert_eq!(pointcloud.seq(), Some(82));
    }

    #[test]
    fn reports_topic_and_validation_error() {
        let mut value: serde_json::Value =
            serde_json::from_str(SETTINGS_EXAMPLE).expect("settings example should be valid JSON");

        value["v"] = serde_json::json!(2);

        let payload = serde_json::to_string(&value).expect("JSON should serialize");

        let error = parse_telemetry(SETTINGS_TOPIC, &payload)
            .expect_err("unsupported version should be rejected");

        assert_eq!(
            error,
            ParseError::Validation {
                topic: SETTINGS_TOPIC.to_owned(),
                source: ValidationError::UnsupportedVersion { actual: 2 },
            }
        );
    }
}
