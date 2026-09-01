use std::{
    collections::HashSet,
    error::Error,
    fmt,
    io::{self, Read},
};

use serde::{Deserialize, Serialize};

pub const BOUNDARY_VERSION: u32 = 1;
pub const MAX_PAYLOAD_BYTES: usize = 1024 * 1024;
pub const MAX_SAFE_JSON_INTEGER: u64 = 9_007_199_254_740_991;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum BoundaryMessage {
    Hello {
        boundary_version: u32,
        session_id: String,
        producer: String,
        camera: BoundaryCamera,
    },
    TrackingFrame {
        boundary_version: u32,
        session_id: String,
        frame_id: u64,
        timestamp_seconds: f64,
        tracking_state: BoundaryTrackingState,
        pose: Option<BoundaryPose>,
    },
    PointcloudDelta {
        boundary_version: u32,
        session_id: String,
        frame_id: u64,
        timestamp_seconds: f64,
        add: Vec<BoundaryMapPoint>,
        update: Vec<BoundaryMapPoint>,
        remove: Vec<u64>,
    },
    SessionEnd {
        boundary_version: u32,
        session_id: String,
        reason: String,
    },
}

impl BoundaryMessage {
    fn version(&self) -> u32 {
        match self {
            Self::Hello {
                boundary_version, ..
            }
            | Self::TrackingFrame {
                boundary_version, ..
            }
            | Self::PointcloudDelta {
                boundary_version, ..
            }
            | Self::SessionEnd {
                boundary_version, ..
            } => *boundary_version,
        }
    }

    fn session_id(&self) -> &str {
        match self {
            Self::Hello { session_id, .. }
            | Self::TrackingFrame { session_id, .. }
            | Self::PointcloudDelta { session_id, .. }
            | Self::SessionEnd { session_id, .. } => session_id,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoundaryCamera {
    pub camera_type: String,
    pub id: String,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoundaryTrackingState {
    Initializing,
    Tracking,
    Lost,
    Relocalizing,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoundaryPose {
    pub translation: [f64; 3],
    pub orientation_xyzw: [f64; 4],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoundaryMapPoint {
    pub id: u64,
    pub position: [f64; 3],
}

#[derive(Debug)]
pub enum BoundaryDecodeError {
    Io(io::Error),
    Oversized { length: usize, maximum: usize },
    InvalidJson(serde_json::Error),
}

impl fmt::Display for BoundaryDecodeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "failed to read SLAM boundary frame: {error}"),
            Self::Oversized { length, maximum } => write!(
                formatter,
                "SLAM boundary payload length {length} exceeds maximum {maximum}"
            ),
            Self::InvalidJson(error) => {
                write!(formatter, "invalid SLAM boundary JSON payload: {error}")
            }
        }
    }
}

impl Error for BoundaryDecodeError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::InvalidJson(error) => Some(error),
            Self::Oversized { .. } => None,
        }
    }
}

pub fn decode_frame(
    reader: &mut impl Read,
) -> Result<Option<BoundaryMessage>, BoundaryDecodeError> {
    let mut length_bytes = [0_u8; 4];
    match reader.read(&mut length_bytes[..1]) {
        Ok(0) => return Ok(None),
        Ok(_) => {}
        Err(error) => return Err(BoundaryDecodeError::Io(error)),
    }
    reader
        .read_exact(&mut length_bytes[1..])
        .map_err(BoundaryDecodeError::Io)?;

    let length = u32::from_be_bytes(length_bytes) as usize;
    if length > MAX_PAYLOAD_BYTES {
        return Err(BoundaryDecodeError::Oversized {
            length,
            maximum: MAX_PAYLOAD_BYTES,
        });
    }

    let mut payload = vec![0_u8; length];
    reader
        .read_exact(&mut payload)
        .map_err(BoundaryDecodeError::Io)?;
    serde_json::from_slice(&payload)
        .map(Some)
        .map_err(BoundaryDecodeError::InvalidJson)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BoundaryValidationError {
    ExpectedHello,
    AlreadyStarted,
    SessionEnded,
    UnsupportedVersion(u32),
    EmptyField(&'static str),
    InvalidCamera,
    SessionMismatch,
    UnsafeInteger { field: &'static str, value: u64 },
    NonFinite(&'static str),
    InvalidQuaternion,
    PoseRequiredWhileTracking,
    FrameIdRegressed,
    TimestampRegressed,
    DuplicatePointId(u64),
}

impl fmt::Display for BoundaryValidationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ExpectedHello => write!(formatter, "hello must be the first boundary message"),
            Self::AlreadyStarted => write!(formatter, "hello was received for an active session"),
            Self::SessionEnded => write!(formatter, "message was received after session_end"),
            Self::UnsupportedVersion(version) => {
                write!(formatter, "unsupported SLAM boundary version {version}")
            }
            Self::EmptyField(field) => write!(formatter, "{field} must not be empty"),
            Self::InvalidCamera => write!(formatter, "camera dimensions and fps must be positive"),
            Self::SessionMismatch => write!(formatter, "boundary session ID does not match hello"),
            Self::UnsafeInteger { field, value } => write!(
                formatter,
                "{field} value {value} exceeds the JSON-safe integer maximum"
            ),
            Self::NonFinite(field) => write!(formatter, "{field} must contain finite values"),
            Self::InvalidQuaternion => write!(formatter, "pose quaternion must have unit length"),
            Self::PoseRequiredWhileTracking => {
                write!(formatter, "tracking state requires an available pose")
            }
            Self::FrameIdRegressed => write!(formatter, "frame ID moved backwards"),
            Self::TimestampRegressed => write!(formatter, "timestamp moved backwards"),
            Self::DuplicatePointId(id) => {
                write!(
                    formatter,
                    "point ID {id} occurs more than once in one delta"
                )
            }
        }
    }
}

impl Error for BoundaryValidationError {}

#[derive(Debug, Default)]
pub struct BoundarySessionValidator {
    session_id: Option<String>,
    ended: bool,
    last_tracking: Option<(u64, f64)>,
    last_pointcloud: Option<(u64, f64)>,
}

impl BoundarySessionValidator {
    pub fn validate(&mut self, message: &BoundaryMessage) -> Result<(), BoundaryValidationError> {
        if message.version() != BOUNDARY_VERSION {
            return Err(BoundaryValidationError::UnsupportedVersion(
                message.version(),
            ));
        }

        match message {
            BoundaryMessage::Hello {
                session_id,
                producer,
                camera,
                ..
            } => {
                if self.session_id.is_some() {
                    return Err(BoundaryValidationError::AlreadyStarted);
                }
                validate_non_empty(session_id, "session_id")?;
                validate_non_empty(producer, "producer")?;
                validate_non_empty(&camera.camera_type, "camera.camera_type")?;
                validate_non_empty(&camera.id, "camera.id")?;
                if camera.width == 0 || camera.height == 0 || camera.fps == 0 {
                    return Err(BoundaryValidationError::InvalidCamera);
                }
                self.session_id = Some(session_id.clone());
            }
            BoundaryMessage::TrackingFrame {
                frame_id,
                timestamp_seconds,
                tracking_state,
                pose,
                ..
            } => {
                self.validate_active(message)?;
                validate_frame_metadata(*frame_id, *timestamp_seconds, self.last_tracking)?;
                if matches!(tracking_state, BoundaryTrackingState::Tracking) && pose.is_none() {
                    return Err(BoundaryValidationError::PoseRequiredWhileTracking);
                }
                if let Some(pose) = pose {
                    validate_pose(pose)?;
                }
                self.last_tracking = Some((*frame_id, *timestamp_seconds));
            }
            BoundaryMessage::PointcloudDelta {
                frame_id,
                timestamp_seconds,
                add,
                update,
                remove,
                ..
            } => {
                self.validate_active(message)?;
                validate_frame_metadata(*frame_id, *timestamp_seconds, self.last_pointcloud)?;
                validate_point_delta(add, update, remove)?;
                self.last_pointcloud = Some((*frame_id, *timestamp_seconds));
            }
            BoundaryMessage::SessionEnd { reason, .. } => {
                self.validate_active(message)?;
                validate_non_empty(reason, "reason")?;
                self.ended = true;
            }
        }
        Ok(())
    }

    fn validate_active(&self, message: &BoundaryMessage) -> Result<(), BoundaryValidationError> {
        let Some(session_id) = &self.session_id else {
            return Err(BoundaryValidationError::ExpectedHello);
        };
        if self.ended {
            return Err(BoundaryValidationError::SessionEnded);
        }
        if message.session_id() != session_id {
            return Err(BoundaryValidationError::SessionMismatch);
        }
        Ok(())
    }
}

fn validate_non_empty(value: &str, field: &'static str) -> Result<(), BoundaryValidationError> {
    if value.trim().is_empty() {
        Err(BoundaryValidationError::EmptyField(field))
    } else {
        Ok(())
    }
}

fn validate_frame_metadata(
    frame_id: u64,
    timestamp: f64,
    previous: Option<(u64, f64)>,
) -> Result<(), BoundaryValidationError> {
    validate_safe_integer(frame_id, "frame_id")?;
    if !timestamp.is_finite() || timestamp < 0.0 {
        return Err(BoundaryValidationError::NonFinite("timestamp_seconds"));
    }
    if let Some((previous_frame, previous_timestamp)) = previous {
        if frame_id < previous_frame {
            return Err(BoundaryValidationError::FrameIdRegressed);
        }
        if timestamp < previous_timestamp {
            return Err(BoundaryValidationError::TimestampRegressed);
        }
    }
    Ok(())
}

fn validate_safe_integer(value: u64, field: &'static str) -> Result<(), BoundaryValidationError> {
    if value > MAX_SAFE_JSON_INTEGER {
        Err(BoundaryValidationError::UnsafeInteger { field, value })
    } else {
        Ok(())
    }
}

fn validate_pose(pose: &BoundaryPose) -> Result<(), BoundaryValidationError> {
    if !pose.translation.iter().all(|value| value.is_finite()) {
        return Err(BoundaryValidationError::NonFinite("pose.translation"));
    }
    if !pose.orientation_xyzw.iter().all(|value| value.is_finite()) {
        return Err(BoundaryValidationError::NonFinite("pose.orientation_xyzw"));
    }
    let norm_squared = pose
        .orientation_xyzw
        .iter()
        .map(|value| value * value)
        .sum::<f64>();
    if (norm_squared - 1.0).abs() > 1.0e-3 {
        return Err(BoundaryValidationError::InvalidQuaternion);
    }
    Ok(())
}

fn validate_point_delta(
    add: &[BoundaryMapPoint],
    update: &[BoundaryMapPoint],
    remove: &[u64],
) -> Result<(), BoundaryValidationError> {
    let mut ids = HashSet::with_capacity(add.len() + update.len() + remove.len());
    for point in add.iter().chain(update) {
        validate_safe_integer(point.id, "point.id")?;
        if !point.position.iter().all(|value| value.is_finite()) {
            return Err(BoundaryValidationError::NonFinite("point.position"));
        }
        if !ids.insert(point.id) {
            return Err(BoundaryValidationError::DuplicatePointId(point.id));
        }
    }
    for id in remove {
        validate_safe_integer(*id, "remove.id")?;
        if !ids.insert(*id) {
            return Err(BoundaryValidationError::DuplicatePointId(*id));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use super::*;

    fn fixture(name: &str) -> BoundaryMessage {
        let json = match name {
            "hello" => include_str!("../tests/fixtures/slam_boundary/hello.json"),
            "tracking_frame" => {
                include_str!("../tests/fixtures/slam_boundary/tracking_frame.json")
            }
            "pointcloud_delta" => {
                include_str!("../tests/fixtures/slam_boundary/pointcloud_delta.json")
            }
            "session_end" => include_str!("../tests/fixtures/slam_boundary/session_end.json"),
            "unsupported_version" => {
                include_str!("../tests/fixtures/slam_boundary/unsupported_version.json")
            }
            _ => panic!("unknown fixture {name}"),
        };
        serde_json::from_str(json).expect("fixture must match the boundary schema")
    }

    #[test]
    fn validates_complete_fixture_session() {
        let mut validator = BoundarySessionValidator::default();
        for name in ["hello", "tracking_frame", "pointcloud_delta", "session_end"] {
            validator
                .validate(&fixture(name))
                .unwrap_or_else(|error| panic!("{name} should be valid: {error}"));
        }
    }

    #[test]
    fn decodes_big_endian_length_prefixed_json() {
        let payload = include_bytes!("../tests/fixtures/slam_boundary/hello.json");
        let mut framed = Vec::from((payload.len() as u32).to_be_bytes());
        framed.extend_from_slice(payload);

        let decoded = decode_frame(&mut Cursor::new(framed))
            .expect("frame should decode")
            .expect("message should exist");

        assert_eq!(decoded, fixture("hello"));
    }

    #[test]
    fn returns_none_only_for_clean_end_of_stream() {
        assert_eq!(decode_frame(&mut Cursor::new(Vec::new())).unwrap(), None);
        assert!(matches!(
            decode_frame(&mut Cursor::new(vec![0, 0])),
            Err(BoundaryDecodeError::Io(_))
        ));
    }

    #[test]
    fn rejects_oversized_payload_before_reading_it() {
        let length = (MAX_PAYLOAD_BYTES as u32 + 1).to_be_bytes();
        assert!(matches!(
            decode_frame(&mut Cursor::new(length)),
            Err(BoundaryDecodeError::Oversized { .. })
        ));
    }

    #[test]
    fn rejects_malformed_json_and_unknown_fields() {
        for payload in [
            br#"{"type":"hello""#.as_slice(),
            br#"{"type":"session_end","boundary_version":1,"session_id":"fixture-session","reason":"shutdown","future":true}"#
                .as_slice(),
        ] {
            let mut framed = Vec::from((payload.len() as u32).to_be_bytes());
            framed.extend_from_slice(payload);
            assert!(matches!(
                decode_frame(&mut Cursor::new(framed)),
                Err(BoundaryDecodeError::InvalidJson(_))
            ));
        }
    }

    #[test]
    fn rejects_message_before_hello_and_unsupported_version() {
        let mut validator = BoundarySessionValidator::default();
        assert_eq!(
            validator.validate(&fixture("tracking_frame")),
            Err(BoundaryValidationError::ExpectedHello)
        );
        assert_eq!(
            validator.validate(&fixture("unsupported_version")),
            Err(BoundaryValidationError::UnsupportedVersion(2))
        );
    }

    #[test]
    fn rejects_regression_and_session_mismatch() {
        let mut validator = BoundarySessionValidator::default();
        validator.validate(&fixture("hello")).unwrap();
        validator.validate(&fixture("tracking_frame")).unwrap();

        let mut regressed = fixture("tracking_frame");
        if let BoundaryMessage::TrackingFrame { frame_id, .. } = &mut regressed {
            *frame_id = 6;
        }
        assert_eq!(
            validator.validate(&regressed),
            Err(BoundaryValidationError::FrameIdRegressed)
        );

        let mut wrong_session = fixture("pointcloud_delta");
        if let BoundaryMessage::PointcloudDelta { session_id, .. } = &mut wrong_session {
            *session_id = "another-session".to_owned();
        }
        assert_eq!(
            validator.validate(&wrong_session),
            Err(BoundaryValidationError::SessionMismatch)
        );
    }

    #[test]
    fn rejects_invalid_pose_and_duplicate_point_operations() {
        let mut validator = BoundarySessionValidator::default();
        validator.validate(&fixture("hello")).unwrap();

        let mut invalid_pose = fixture("tracking_frame");
        if let BoundaryMessage::TrackingFrame { pose, .. } = &mut invalid_pose {
            pose.as_mut().unwrap().orientation_xyzw = [0.0; 4];
        }
        assert_eq!(
            validator.validate(&invalid_pose),
            Err(BoundaryValidationError::InvalidQuaternion)
        );

        let mut duplicate = fixture("pointcloud_delta");
        if let BoundaryMessage::PointcloudDelta { remove, .. } = &mut duplicate {
            remove.push(1001);
        }
        assert_eq!(
            validator.validate(&duplicate),
            Err(BoundaryValidationError::DuplicatePointId(1001))
        );
    }

    #[test]
    fn rejects_tracking_without_pose_and_unsafe_ids() {
        let mut validator = BoundarySessionValidator::default();
        validator.validate(&fixture("hello")).unwrap();

        let mut missing_pose = fixture("tracking_frame");
        if let BoundaryMessage::TrackingFrame { pose, .. } = &mut missing_pose {
            *pose = None;
        }
        assert_eq!(
            validator.validate(&missing_pose),
            Err(BoundaryValidationError::PoseRequiredWhileTracking)
        );

        let mut unsafe_id = fixture("pointcloud_delta");
        if let BoundaryMessage::PointcloudDelta { add, .. } = &mut unsafe_id {
            add[0].id = MAX_SAFE_JSON_INTEGER + 1;
        }
        assert_eq!(
            validator.validate(&unsafe_id),
            Err(BoundaryValidationError::UnsafeInteger {
                field: "point.id",
                value: MAX_SAFE_JSON_INTEGER + 1,
            })
        );
    }

    #[test]
    fn rejects_messages_after_session_end() {
        let mut validator = BoundarySessionValidator::default();
        validator.validate(&fixture("hello")).unwrap();
        validator.validate(&fixture("session_end")).unwrap();
        assert_eq!(
            validator.validate(&fixture("tracking_frame")),
            Err(BoundaryValidationError::SessionEnded)
        );
    }
}
