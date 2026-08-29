use std::{
    collections::BTreeMap,
    error::Error,
    fmt,
    fs::File,
    io::{self, BufRead, BufReader},
    path::{Component, Path, PathBuf},
    time::Duration,
};

use serde::Deserialize;
use serde_json::value::RawValue;

use crate::{
    MAX_PAYLOAD_BYTES,
    protocol::{
        POINTCLOUD_TOPIC, POSE_TOPIC, PointCloudMessage, PoseMessage, SETTINGS_TOPIC,
        SettingsMessage,
    },
};

#[derive(Debug)]
pub enum PlaybackError {
    Io {
        path: PathBuf,
        line: Option<usize>,
        source: io::Error,
    },
    InvalidMetadata {
        path: PathBuf,
        reason: String,
    },
    InvalidTelemetry {
        path: PathBuf,
        line: usize,
        reason: String,
    },
    InvalidSpeed(f64),
    InvalidSchedule {
        line: usize,
        reason: String,
    },
}

impl fmt::Display for PlaybackError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io {
                path,
                line: Some(line),
                source,
            } => write!(
                formatter,
                "failed to read {} line {line}: {source}",
                path.display()
            ),
            Self::Io {
                path,
                line: None,
                source,
            } => write!(formatter, "failed to read {}: {source}", path.display()),
            Self::InvalidMetadata { path, reason } => {
                write!(formatter, "invalid metadata {}: {reason}", path.display())
            }
            Self::InvalidTelemetry { path, line, reason } => write!(
                formatter,
                "invalid telemetry {} line {line}: {reason}",
                path.display()
            ),
            Self::InvalidSpeed(speed) => {
                write!(
                    formatter,
                    "playback speed must be positive and finite, received {speed}"
                )
            }
            Self::InvalidSchedule { line, reason } => {
                write!(
                    formatter,
                    "invalid playback schedule at telemetry line {line}: {reason}"
                )
            }
        }
    }
}

impl Error for PlaybackError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct RecordingMetadata {
    pub protocol_version: u32,
    pub session: String,
    pub frame: String,
    pub unit: String,
    pub message_count: u64,
    pub pose_count: u64,
    pub pointcloud_message_count: u64,
    pub point_count: usize,
    pub telemetry_file: String,
    pub pointcloud_file: String,
}

#[derive(Debug)]
pub struct PlaybackMessage {
    topic: &'static str,
    payload: Box<RawValue>,
    timestamp: Option<f64>,
    line: usize,
}

impl PlaybackMessage {
    pub fn topic(&self) -> &'static str {
        self.topic
    }

    pub fn payload(&self) -> &[u8] {
        self.payload.get().as_bytes()
    }

    pub fn timestamp(&self) -> Option<f64> {
        self.timestamp
    }

    pub fn line(&self) -> usize {
        self.line
    }
}

#[derive(Debug)]
pub struct LoadedSession {
    directory: PathBuf,
    metadata: RecordingMetadata,
    messages: Vec<PlaybackMessage>,
}

impl LoadedSession {
    pub fn directory(&self) -> &Path {
        &self.directory
    }

    pub fn metadata(&self) -> &RecordingMetadata {
        &self.metadata
    }

    pub fn messages(&self) -> &[PlaybackMessage] {
        &self.messages
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RecordedEnvelope {
    topic: String,
    payload: Box<RawValue>,
}

pub fn load_session(directory: &Path) -> Result<LoadedSession, PlaybackError> {
    let metadata_path = directory.join("metadata.json");
    let metadata_file = File::open(&metadata_path).map_err(|source| PlaybackError::Io {
        path: metadata_path.clone(),
        line: None,
        source,
    })?;
    let metadata: RecordingMetadata =
        serde_json::from_reader(metadata_file).map_err(|error| PlaybackError::InvalidMetadata {
            path: metadata_path.clone(),
            reason: error.to_string(),
        })?;
    validate_metadata(&metadata_path, &metadata)?;

    let telemetry_path = directory.join(&metadata.telemetry_file);
    let telemetry_file = File::open(&telemetry_path).map_err(|source| PlaybackError::Io {
        path: telemetry_path.clone(),
        line: None,
        source,
    })?;
    let reader = BufReader::new(telemetry_file);
    let mut messages = Vec::new();
    let mut pose_count = 0_u64;
    let mut pointcloud_message_count = 0_u64;
    let mut points = BTreeMap::new();

    for (index, line) in reader.lines().enumerate() {
        let line_number = index + 1;
        let line = line.map_err(|source| PlaybackError::Io {
            path: telemetry_path.clone(),
            line: Some(line_number),
            source,
        })?;
        if line.trim().is_empty() {
            return Err(invalid_telemetry(
                &telemetry_path,
                line_number,
                "blank lines are not allowed",
            ));
        }

        let envelope: RecordedEnvelope =
            serde_json::from_str(&line).map_err(|error| PlaybackError::InvalidTelemetry {
                path: telemetry_path.clone(),
                line: line_number,
                reason: error.to_string(),
            })?;
        if envelope.payload.get().len() > MAX_PAYLOAD_BYTES {
            return Err(invalid_telemetry(
                &telemetry_path,
                line_number,
                format!(
                    "payload contains {} bytes, maximum is {MAX_PAYLOAD_BYTES}",
                    envelope.payload.get().len()
                ),
            ));
        }

        let (topic, timestamp) = match envelope.topic.as_str() {
            SETTINGS_TOPIC => {
                let settings: SettingsMessage = parse_payload(
                    &telemetry_path,
                    line_number,
                    SETTINGS_TOPIC,
                    envelope.payload.get(),
                )?;
                validate_settings(&telemetry_path, line_number, &metadata, &settings)?;
                (SETTINGS_TOPIC, None)
            }
            POSE_TOPIC => {
                let pose: PoseMessage = parse_payload(
                    &telemetry_path,
                    line_number,
                    POSE_TOPIC,
                    envelope.payload.get(),
                )?;
                pose.validate().map_err(|error| {
                    invalid_telemetry(&telemetry_path, line_number, error.to_string())
                })?;
                validate_session(
                    &telemetry_path,
                    line_number,
                    &metadata.session,
                    &pose.session,
                )?;
                pose_count += 1;
                (POSE_TOPIC, Some(pose.t))
            }
            POINTCLOUD_TOPIC => {
                let pointcloud: PointCloudMessage = parse_payload(
                    &telemetry_path,
                    line_number,
                    POINTCLOUD_TOPIC,
                    envelope.payload.get(),
                )?;
                pointcloud.validate().map_err(|error| {
                    invalid_telemetry(&telemetry_path, line_number, error.to_string())
                })?;
                validate_session(
                    &telemetry_path,
                    line_number,
                    &metadata.session,
                    &pointcloud.session,
                )?;
                apply_delta(&mut points, &pointcloud);
                pointcloud_message_count += 1;
                (POINTCLOUD_TOPIC, Some(pointcloud.t))
            }
            topic => {
                return Err(invalid_telemetry(
                    &telemetry_path,
                    line_number,
                    format!("unsupported topic {topic:?}"),
                ));
            }
        };

        if messages.is_empty() && topic != SETTINGS_TOPIC {
            return Err(invalid_telemetry(
                &telemetry_path,
                line_number,
                "the first message must be Settings",
            ));
        }
        messages.push(PlaybackMessage {
            topic,
            payload: envelope.payload,
            timestamp,
            line: line_number,
        });
    }

    validate_statistics(
        &metadata_path,
        &metadata,
        messages.len(),
        pose_count,
        pointcloud_message_count,
        points.len(),
    )?;

    Ok(LoadedSession {
        directory: directory.to_owned(),
        metadata,
        messages,
    })
}

pub fn playback_schedule(
    messages: &[PlaybackMessage],
    speed: f64,
) -> Result<Vec<Duration>, PlaybackError> {
    if !speed.is_finite() || speed <= 0.0 {
        return Err(PlaybackError::InvalidSpeed(speed));
    }

    let mut first_timestamp = None;
    let mut latest_offset = Duration::ZERO;
    let mut offsets = Vec::with_capacity(messages.len());

    for message in messages {
        if let Some(timestamp) = message.timestamp {
            let baseline = *first_timestamp.get_or_insert(timestamp);
            let seconds = ((timestamp - baseline) / speed).max(0.0);
            let timestamp_offset = Duration::try_from_secs_f64(seconds).map_err(|error| {
                PlaybackError::InvalidSchedule {
                    line: message.line,
                    reason: error.to_string(),
                }
            })?;
            latest_offset = latest_offset.max(timestamp_offset);
        }
        offsets.push(latest_offset);
    }

    Ok(offsets)
}

fn validate_metadata(path: &Path, metadata: &RecordingMetadata) -> Result<(), PlaybackError> {
    if metadata.protocol_version != 1 {
        return Err(invalid_metadata(
            path,
            format!(
                "protocol_version must be 1, received {}",
                metadata.protocol_version
            ),
        ));
    }
    if metadata.session.trim().is_empty() {
        return Err(invalid_metadata(path, "session must not be empty"));
    }
    if metadata.frame != "unity_world" {
        return Err(invalid_metadata(
            path,
            format!(
                "frame must be \"unity_world\", received {:?}",
                metadata.frame
            ),
        ));
    }
    if metadata.unit != "m" {
        return Err(invalid_metadata(
            path,
            format!("unit must be \"m\", received {:?}", metadata.unit),
        ));
    }
    validate_filename(path, "telemetry_file", &metadata.telemetry_file)?;
    validate_filename(path, "pointcloud_file", &metadata.pointcloud_file)
}

fn validate_filename(path: &Path, field: &str, filename: &str) -> Result<(), PlaybackError> {
    let mut components = Path::new(filename).components();
    if !matches!(components.next(), Some(Component::Normal(_))) || components.next().is_some() {
        return Err(invalid_metadata(
            path,
            format!("{field} must be a filename without directory components"),
        ));
    }
    Ok(())
}

fn validate_settings(
    path: &Path,
    line: usize,
    metadata: &RecordingMetadata,
    settings: &SettingsMessage,
) -> Result<(), PlaybackError> {
    validate_session(path, line, &metadata.session, &settings.session)?;
    let fixed_values = [
        ("unit", settings.unit.as_str(), "m"),
        ("frame", settings.frame.as_str(), "unity_world"),
        ("pose_convention", settings.pose_convention.as_str(), "Twc"),
        ("quaternion", settings.quaternion.as_str(), "xyzw"),
        (
            "pointcloud_mode",
            settings.pointcloud_mode.as_str(),
            "delta",
        ),
    ];
    if settings.v != 1 {
        return Err(invalid_telemetry(
            path,
            line,
            format!("unsupported protocol version: {}", settings.v),
        ));
    }
    for (field, actual, expected) in fixed_values {
        if actual != expected {
            return Err(invalid_telemetry(
                path,
                line,
                format!("{field} must be {expected:?}, received {actual:?}"),
            ));
        }
    }
    Ok(())
}

fn validate_session(
    path: &Path,
    line: usize,
    expected: &str,
    actual: &str,
) -> Result<(), PlaybackError> {
    if actual != expected {
        return Err(invalid_telemetry(
            path,
            line,
            format!("session must be {expected:?}, received {actual:?}"),
        ));
    }
    Ok(())
}

fn validate_statistics(
    path: &Path,
    metadata: &RecordingMetadata,
    message_count: usize,
    pose_count: u64,
    pointcloud_message_count: u64,
    point_count: usize,
) -> Result<(), PlaybackError> {
    let actual_message_count = u64::try_from(message_count).unwrap_or(u64::MAX);
    let values = [
        (
            "message_count",
            metadata.message_count,
            actual_message_count,
        ),
        ("pose_count", metadata.pose_count, pose_count),
        (
            "pointcloud_message_count",
            metadata.pointcloud_message_count,
            pointcloud_message_count,
        ),
    ];
    for (field, expected, actual) in values {
        if expected != actual {
            return Err(invalid_metadata(
                path,
                format!("{field} is {expected}, but telemetry contains {actual}"),
            ));
        }
    }
    if metadata.point_count != point_count {
        return Err(invalid_metadata(
            path,
            format!(
                "point_count is {}, but telemetry reconstructs {point_count}",
                metadata.point_count
            ),
        ));
    }
    if message_count == 0 {
        return Err(invalid_metadata(path, "telemetry must contain Settings"));
    }
    Ok(())
}

fn parse_payload<T: serde::de::DeserializeOwned>(
    path: &Path,
    line: usize,
    topic: &str,
    payload: &str,
) -> Result<T, PlaybackError> {
    serde_json::from_str(payload)
        .map_err(|error| invalid_telemetry(path, line, format!("invalid {topic} payload: {error}")))
}

fn apply_delta(points: &mut BTreeMap<u64, [f64; 3]>, delta: &PointCloudMessage) {
    for id in &delta.remove {
        points.remove(id);
    }
    for &(id, x, y, z) in &delta.update {
        points.insert(id, [x, y, z]);
    }
    for &(id, x, y, z) in &delta.add {
        points.insert(id, [x, y, z]);
    }
}

fn invalid_metadata(path: &Path, reason: impl Into<String>) -> PlaybackError {
    PlaybackError::InvalidMetadata {
        path: path.to_owned(),
        reason: reason.into(),
    }
}

fn invalid_telemetry(path: &Path, line: usize, reason: impl Into<String>) -> PlaybackError {
    PlaybackError::InvalidTelemetry {
        path: path.to_owned(),
        line,
        reason: reason.into(),
    }
}

#[cfg(test)]
mod tests {
    use std::{
        fs,
        sync::atomic::{AtomicU64, Ordering},
    };

    use super::*;

    static NEXT_DIRECTORY: AtomicU64 = AtomicU64::new(0);

    const SETTINGS_PAYLOAD: &str = r#"{"v":1,"session":"playback-test","unit":"m","frame":"unity_world","pose_convention":"Twc","quaternion":"xyzw","camera":{"type":"mock","id":"mock-camera-0","width":640,"height":480,"fps":30},"pointcloud_mode":"delta"}"#;
    const POSE_PAYLOAD: &str = r#"{"v":1,"session":"playback-test","seq":0,"t":10.0,"p":[1.0,2.0,3.0],"q":[0.0,0.0,0.0,1.0],"state":"tracking"}"#;
    const POINTS_ONE: &str = r#"{"v":1,"session":"playback-test","seq":0,"t":10.5,"add":[[2,2.0,2.0,2.0],[1,1.0,1.0,1.0]],"update":[],"remove":[]}"#;
    const POINTS_TWO: &str = r#"{"v":1,"session":"playback-test","seq":1,"t":11.0,"add":[],"update":[[1,10.0,10.0,10.0],[3,3.0,3.0,3.0]],"remove":[2]}"#;

    struct TestDirectory(PathBuf);

    impl TestDirectory {
        fn valid() -> Self {
            let id = NEXT_DIRECTORY.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir()
                .join(format!("slam-session-playback-{}-{id}", std::process::id()));
            if path.exists() {
                fs::remove_dir_all(&path).expect("stale test directory should be removable");
            }
            fs::create_dir(&path).expect("test directory should be creatable");
            let telemetry = [
                envelope(SETTINGS_TOPIC, SETTINGS_PAYLOAD),
                envelope(POSE_TOPIC, POSE_PAYLOAD),
                envelope(POINTCLOUD_TOPIC, POINTS_ONE),
                envelope(POINTCLOUD_TOPIC, POINTS_TWO),
            ]
            .join("\n");
            fs::write(path.join("telemetry.ndjson"), format!("{telemetry}\n"))
                .expect("telemetry fixture should be writable");
            write_metadata(&path, 4, 1, 2, 2);
            Self(path)
        }
    }

    impl Drop for TestDirectory {
        fn drop(&mut self) {
            fs::remove_dir_all(&self.0).expect("test directory should be removable");
        }
    }

    fn envelope(topic: &str, payload: &str) -> String {
        format!(r#"{{"topic":"{topic}","payload":{payload}}}"#)
    }

    fn write_metadata(
        directory: &Path,
        messages: u64,
        poses: u64,
        pointcloud_messages: u64,
        points: usize,
    ) {
        fs::write(
            directory.join("metadata.json"),
            format!(
                r#"{{"protocol_version":1,"session":"playback-test","frame":"unity_world","unit":"m","message_count":{messages},"pose_count":{poses},"pointcloud_message_count":{pointcloud_messages},"point_count":{points},"telemetry_file":"telemetry.ndjson","pointcloud_file":"pointcloud.ply"}}"#
            ),
        )
        .expect("metadata fixture should be writable");
    }

    #[test]
    fn loads_valid_recording_and_preserves_order_and_payload_bytes() {
        let directory = TestDirectory::valid();

        let session = load_session(&directory.0).expect("recording should load");

        assert_eq!(session.metadata().session, "playback-test");
        assert_eq!(session.messages().len(), 4);
        assert_eq!(session.messages()[0].topic(), SETTINGS_TOPIC);
        assert_eq!(session.messages()[1].topic(), POSE_TOPIC);
        assert_eq!(session.messages()[2].topic(), POINTCLOUD_TOPIC);
        assert_eq!(session.messages()[3].topic(), POINTCLOUD_TOPIC);
        assert_eq!(session.messages()[1].payload(), POSE_PAYLOAD.as_bytes());
    }

    #[test]
    fn computes_timestamp_schedule_at_configured_speed() {
        let directory = TestDirectory::valid();
        let session = load_session(&directory.0).expect("recording should load");

        let normal = playback_schedule(session.messages(), 1.0).expect("schedule");
        let double = playback_schedule(session.messages(), 2.0).expect("schedule");

        assert_eq!(
            normal,
            [
                Duration::ZERO,
                Duration::ZERO,
                Duration::from_millis(500),
                Duration::from_secs(1),
            ]
        );
        assert_eq!(
            double,
            [
                Duration::ZERO,
                Duration::ZERO,
                Duration::from_millis(250),
                Duration::from_millis(500),
            ]
        );
    }

    #[test]
    fn clamps_out_of_order_timestamps_without_reordering_messages() {
        let directory = TestDirectory::valid();
        let mut session = load_session(&directory.0).expect("recording should load");
        session.messages[3].timestamp = Some(10.25);

        let schedule = playback_schedule(session.messages(), 1.0).expect("schedule");

        assert_eq!(schedule[2], Duration::from_millis(500));
        assert_eq!(schedule[3], Duration::from_millis(500));
    }

    #[test]
    fn rejects_non_positive_or_non_finite_speed() {
        let directory = TestDirectory::valid();
        let session = load_session(&directory.0).expect("recording should load");

        for speed in [0.0, -1.0, f64::INFINITY, f64::NAN] {
            assert!(matches!(
                playback_schedule(session.messages(), speed),
                Err(PlaybackError::InvalidSpeed(_))
            ));
        }
    }

    #[test]
    fn reports_malformed_ndjson_with_path_and_line() {
        let directory = TestDirectory::valid();
        fs::write(
            directory.0.join("telemetry.ndjson"),
            format!("{}\nnot-json\n", envelope(SETTINGS_TOPIC, SETTINGS_PAYLOAD)),
        )
        .expect("fixture should be writable");

        let error = load_session(&directory.0).expect_err("malformed line should fail");
        let text = error.to_string();
        assert!(text.contains("telemetry.ndjson line 2"));
        assert!(text.contains("expected ident"));
    }

    #[test]
    fn rejects_truncated_recording_using_metadata_counts() {
        let directory = TestDirectory::valid();
        let telemetry = [
            envelope(SETTINGS_TOPIC, SETTINGS_PAYLOAD),
            envelope(POSE_TOPIC, POSE_PAYLOAD),
        ]
        .join("\n");
        fs::write(directory.0.join("telemetry.ndjson"), telemetry)
            .expect("fixture should be writable");

        let error = load_session(&directory.0).expect_err("truncated recording should fail");
        assert!(error.to_string().contains("message_count is 4"));
        assert!(error.to_string().contains("telemetry contains 2"));
    }

    #[test]
    fn rejects_final_point_count_mismatch() {
        let directory = TestDirectory::valid();
        write_metadata(&directory.0, 4, 1, 2, 99);

        let error = load_session(&directory.0).expect_err("point mismatch should fail");
        assert!(error.to_string().contains("point_count is 99"));
        assert!(error.to_string().contains("reconstructs 2"));
    }

    #[test]
    fn requires_settings_as_first_message() {
        let directory = TestDirectory::valid();
        fs::write(
            directory.0.join("telemetry.ndjson"),
            format!("{}\n", envelope(POSE_TOPIC, POSE_PAYLOAD)),
        )
        .expect("fixture should be writable");
        write_metadata(&directory.0, 1, 1, 0, 0);

        let error = load_session(&directory.0).expect_err("missing Settings should fail");
        assert!(error.to_string().contains("first message must be Settings"));
    }

    #[test]
    fn rejects_session_mismatch() {
        let directory = TestDirectory::valid();
        let mismatched = POSE_PAYLOAD.replace("playback-test", "different-session");
        let telemetry = [
            envelope(SETTINGS_TOPIC, SETTINGS_PAYLOAD),
            envelope(POSE_TOPIC, &mismatched),
        ]
        .join("\n");
        fs::write(directory.0.join("telemetry.ndjson"), telemetry)
            .expect("fixture should be writable");
        write_metadata(&directory.0, 2, 1, 0, 0);

        let error = load_session(&directory.0).expect_err("session mismatch should fail");
        assert!(error.to_string().contains("line 2"));
        assert!(
            error
                .to_string()
                .contains("session must be \"playback-test\"")
        );
    }

    #[test]
    fn rejects_metadata_path_traversal() {
        let directory = TestDirectory::valid();
        let metadata = fs::read_to_string(directory.0.join("metadata.json"))
            .expect("metadata should be readable")
            .replace("telemetry.ndjson", "../telemetry.ndjson");
        fs::write(directory.0.join("metadata.json"), metadata)
            .expect("metadata should be writable");

        let error = load_session(&directory.0).expect_err("path traversal should fail");
        assert!(
            error
                .to_string()
                .contains("telemetry_file must be a filename")
        );
    }
}
