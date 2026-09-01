use std::{
    collections::BTreeMap,
    error::Error,
    fmt,
    fs::{self, File},
    io::{self, BufRead, BufReader, BufWriter, Read, Write},
    path::{Path, PathBuf},
    sync::mpsc::{self, Sender},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use serde::{Deserialize, Serialize};
use serde_json::value::RawValue;

use crate::{
    MAX_PAYLOAD_BYTES,
    protocol::{
        POINTCLOUD_TOPIC, POSE_TOPIC, PointCloudMessage, PoseMessage, SETTINGS_TOPIC,
        SettingsMessage, TelemetryMessage,
    },
};

const CHECKPOINT_FILE: &str = "recording.inprogress.json";
const CHECKPOINT_INTERVAL: Duration = Duration::from_secs(1);
const CHECKPOINT_MESSAGE_INTERVAL: u64 = 64;
const RECOVERED_TELEMETRY_FILE: &str = "telemetry.recovered.ndjson";

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SessionRejection {
    SettingsRequired { session: String },
    MismatchedSession { expected: String, actual: String },
}

impl fmt::Display for SessionRejection {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SettingsRequired { session } => {
                write!(
                    formatter,
                    "settings required before telemetry for session={session}"
                )
            }
            Self::MismatchedSession { expected, actual } => write!(
                formatter,
                "session mismatch: active={expected} received={actual}"
            ),
        }
    }
}

impl Error for SessionRejection {}

#[derive(Debug, Default)]
pub struct SessionGate {
    active_session: Option<String>,
}

impl SessionGate {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn accept(&mut self, message: &TelemetryMessage) -> Result<(), SessionRejection> {
        if let TelemetryMessage::Settings(settings) = message {
            self.active_session = Some(settings.session.clone());
            return Ok(());
        }

        let Some(active_session) = &self.active_session else {
            return Err(SessionRejection::SettingsRequired {
                session: message.session().to_owned(),
            });
        };

        if message.session() != active_session {
            return Err(SessionRejection::MismatchedSession {
                expected: active_session.clone(),
                actual: message.session().to_owned(),
            });
        }

        Ok(())
    }
}

#[derive(Debug)]
pub enum RecordingError {
    Io {
        operation: &'static str,
        path: PathBuf,
        source: io::Error,
    },
    Serialization {
        path: PathBuf,
        source: serde_json::Error,
    },
    InvalidRecovery {
        path: PathBuf,
        line: Option<usize>,
        reason: String,
    },
    WorkerStopped,
    WorkerPanicked,
}

impl fmt::Display for RecordingError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io {
                operation,
                path,
                source,
            } => write!(
                formatter,
                "failed to {operation} recording path {}: {source}",
                path.display()
            ),
            Self::Serialization { path, source } => write!(
                formatter,
                "failed to serialize recording data for {}: {source}",
                path.display()
            ),
            Self::InvalidRecovery {
                path,
                line: Some(line),
                reason,
            } => write!(
                formatter,
                "invalid recoverable telemetry {} line {line}: {reason}",
                path.display()
            ),
            Self::InvalidRecovery {
                path,
                line: None,
                reason,
            } => write!(
                formatter,
                "invalid recording recovery data {}: {reason}",
                path.display()
            ),
            Self::WorkerStopped => write!(formatter, "recording worker stopped unexpectedly"),
            Self::WorkerPanicked => write!(formatter, "recording worker panicked"),
        }
    }
}

impl Error for RecordingError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            Self::Serialization { source, .. } => Some(source),
            Self::InvalidRecovery { .. } | Self::WorkerStopped | Self::WorkerPanicked => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordingSummary {
    pub session: String,
    pub directory: PathBuf,
    pub message_count: u64,
    pub pose_count: u64,
    pub pointcloud_message_count: u64,
    pub point_count: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecoverySummary {
    pub session: String,
    pub directory: PathBuf,
    pub message_count: u64,
    pub point_count: usize,
    pub discarded_trailing_bytes: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecoveryFailure {
    pub directory: PathBuf,
    pub reason: String,
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct RecoveryReport {
    pub recovered: Vec<RecoverySummary>,
    pub already_finalized: Vec<PathBuf>,
    pub failures: Vec<RecoveryFailure>,
}

pub struct TelemetryRecorder {
    sender: Option<Sender<TelemetryMessage>>,
    worker: Option<JoinHandle<Result<Vec<RecordingSummary>, RecordingError>>>,
}

impl TelemetryRecorder {
    pub fn start(root: impl Into<PathBuf>) -> Self {
        let root = root.into();
        let (sender, receiver) = mpsc::channel();
        let worker = thread::spawn(move || record_messages(&root, receiver));

        Self {
            sender: Some(sender),
            worker: Some(worker),
        }
    }

    pub fn record(&self, message: &TelemetryMessage) -> Result<(), RecordingError> {
        self.sender
            .as_ref()
            .ok_or(RecordingError::WorkerStopped)?
            .send(message.clone())
            .map_err(|_| RecordingError::WorkerStopped)
    }

    pub fn finish(mut self) -> Result<Vec<RecordingSummary>, RecordingError> {
        self.sender.take();
        self.worker
            .take()
            .ok_or(RecordingError::WorkerStopped)?
            .join()
            .map_err(|_| RecordingError::WorkerPanicked)?
    }
}

fn record_messages(
    root: &Path,
    receiver: mpsc::Receiver<TelemetryMessage>,
) -> Result<Vec<RecordingSummary>, RecordingError> {
    let mut active: Option<SessionRecording> = None;
    let mut summaries = Vec::new();

    loop {
        match receiver.recv_timeout(CHECKPOINT_INTERVAL) {
            Ok(message) => {
                if let TelemetryMessage::Settings(settings) = &message
                    && active
                        .as_ref()
                        .is_none_or(|recording| recording.session != settings.session)
                {
                    if let Some(recording) = active.take() {
                        summaries.push(recording.finish()?);
                    }
                    active = Some(SessionRecording::start(root, settings)?);
                }

                if let Some(recording) = &mut active
                    && recording.session == message.session()
                {
                    recording.append(&message)?;
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {
                if let Some(recording) = &mut active {
                    recording.checkpoint(false)?;
                }
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }

    if let Some(recording) = active {
        summaries.push(recording.finish()?);
    }

    Ok(summaries)
}

struct SessionRecording {
    session: String,
    frame: String,
    unit: String,
    directory: PathBuf,
    telemetry_path: PathBuf,
    telemetry: BufWriter<File>,
    points: BTreeMap<u64, [f64; 3]>,
    message_count: u64,
    pose_count: u64,
    pointcloud_message_count: u64,
    messages_since_checkpoint: u64,
    last_checkpoint: Instant,
}

impl SessionRecording {
    fn start(root: &Path, settings: &SettingsMessage) -> Result<Self, RecordingError> {
        create_directory(root)?;
        let directory = create_session_directory(root, &settings.session)?;
        let telemetry_path = directory.join("telemetry.ndjson");
        let telemetry_file =
            File::create(&telemetry_path).map_err(|source| RecordingError::Io {
                operation: "create telemetry log",
                path: telemetry_path.clone(),
                source,
            })?;

        let recording = Self {
            session: settings.session.clone(),
            frame: settings.frame.clone(),
            unit: settings.unit.clone(),
            directory,
            telemetry_path,
            telemetry: BufWriter::new(telemetry_file),
            points: BTreeMap::new(),
            message_count: 0,
            pose_count: 0,
            pointcloud_message_count: 0,
            messages_since_checkpoint: 0,
            last_checkpoint: Instant::now(),
        };
        recording.write_checkpoint()?;
        Ok(recording)
    }

    fn append(&mut self, message: &TelemetryMessage) -> Result<(), RecordingError> {
        write_recorded_message(&mut self.telemetry, &self.telemetry_path, message)?;
        self.message_count += 1;

        match message {
            TelemetryMessage::Settings(_) => {}
            TelemetryMessage::Pose(_) => self.pose_count += 1,
            TelemetryMessage::PointCloud(delta) => {
                self.pointcloud_message_count += 1;
                apply_delta(&mut self.points, delta);
            }
        }

        self.messages_since_checkpoint += 1;
        self.checkpoint(false)?;

        Ok(())
    }

    fn finish(mut self) -> Result<RecordingSummary, RecordingError> {
        self.checkpoint(true)?;

        let summary = RecordingSummary {
            session: self.session.clone(),
            directory: self.directory.clone(),
            message_count: self.message_count,
            pose_count: self.pose_count,
            pointcloud_message_count: self.pointcloud_message_count,
            point_count: self.points.len(),
        };

        let ply_path = self.directory.join("pointcloud.ply");
        write_atomic(&ply_path, |writer| {
            write_ply(writer, &sanitize_session(&self.session), &self.points)
        })?;

        let metadata_path = self.directory.join("metadata.json");
        let metadata = RecordingMetadata {
            protocol_version: 1,
            session: &self.session,
            frame: &self.frame,
            unit: &self.unit,
            message_count: self.message_count,
            pose_count: self.pose_count,
            pointcloud_message_count: self.pointcloud_message_count,
            point_count: self.points.len(),
            telemetry_file: "telemetry.ndjson",
            pointcloud_file: "pointcloud.ply",
            finalization: "clean",
            discarded_trailing_bytes: 0,
        };
        write_atomic(&metadata_path, |writer| {
            serde_json::to_writer_pretty(writer, &metadata).map_err(io::Error::other)
        })?;

        let checkpoint_path = self.directory.join(CHECKPOINT_FILE);
        fs::remove_file(&checkpoint_path).map_err(|source| RecordingError::Io {
            operation: "remove completed recording checkpoint",
            path: checkpoint_path,
            source,
        })?;

        Ok(summary)
    }

    fn checkpoint(&mut self, force: bool) -> Result<(), RecordingError> {
        if !force
            && self.messages_since_checkpoint < CHECKPOINT_MESSAGE_INTERVAL
            && self.last_checkpoint.elapsed() < CHECKPOINT_INTERVAL
        {
            return Ok(());
        }

        self.telemetry
            .flush()
            .map_err(|source| RecordingError::Io {
                operation: "flush telemetry checkpoint",
                path: self.telemetry_path.clone(),
                source,
            })?;
        self.telemetry
            .get_ref()
            .sync_data()
            .map_err(|source| RecordingError::Io {
                operation: "sync telemetry checkpoint",
                path: self.telemetry_path.clone(),
                source,
            })?;
        self.write_checkpoint()?;
        self.messages_since_checkpoint = 0;
        self.last_checkpoint = Instant::now();
        Ok(())
    }

    fn write_checkpoint(&self) -> Result<(), RecordingError> {
        let checkpoint_path = self.directory.join(CHECKPOINT_FILE);
        let checkpoint = InProgressCheckpoint {
            protocol_version: 1,
            recording_state: "in_progress".to_owned(),
            session: self.session.clone(),
            frame: self.frame.clone(),
            unit: self.unit.clone(),
            flushed_message_count: self.message_count,
            telemetry_file: "telemetry.ndjson".to_owned(),
        };
        write_atomic(&checkpoint_path, |writer| {
            serde_json::to_writer_pretty(writer, &checkpoint).map_err(io::Error::other)
        })
    }
}

#[derive(Serialize)]
struct RecordedMessage<'a, T> {
    topic: &'static str,
    payload: &'a T,
}

#[derive(Serialize)]
struct RecordingMetadata<'a> {
    protocol_version: u32,
    session: &'a str,
    frame: &'a str,
    unit: &'a str,
    message_count: u64,
    pose_count: u64,
    pointcloud_message_count: u64,
    point_count: usize,
    telemetry_file: &'static str,
    pointcloud_file: &'static str,
    finalization: &'static str,
    discarded_trailing_bytes: u64,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct InProgressCheckpoint {
    protocol_version: u32,
    recording_state: String,
    session: String,
    frame: String,
    unit: String,
    flushed_message_count: u64,
    telemetry_file: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RecoveryEnvelope {
    topic: String,
    payload: Box<RawValue>,
}

#[derive(Default)]
struct RecoveredTelemetryState {
    settings_seen: bool,
    message_count: u64,
    pose_count: u64,
    pointcloud_message_count: u64,
    points: BTreeMap<u64, [f64; 3]>,
    valid_bytes: u64,
    discarded_trailing_bytes: u64,
}

pub fn recover_incomplete_recordings(root: &Path) -> Result<RecoveryReport, RecordingError> {
    if !root.exists() {
        return Ok(RecoveryReport::default());
    }

    let entries = fs::read_dir(root).map_err(|source| RecordingError::Io {
        operation: "scan recording root",
        path: root.to_owned(),
        source,
    })?;
    let mut directories = Vec::new();
    for entry in entries {
        let entry = entry.map_err(|source| RecordingError::Io {
            operation: "read recording-root entry",
            path: root.to_owned(),
            source,
        })?;
        let path = entry.path();
        let file_type = entry.file_type().map_err(|source| RecordingError::Io {
            operation: "inspect recording-root entry",
            path: path.clone(),
            source,
        })?;
        if file_type.is_dir() && entry.file_name() != "clips" {
            directories.push(path);
        }
    }
    directories.sort();

    let mut report = RecoveryReport::default();
    for directory in directories {
        let checkpoint_path = directory.join(CHECKPOINT_FILE);
        if !checkpoint_path.is_file() {
            continue;
        }

        let metadata_path = directory.join("metadata.json");
        if metadata_path.is_file() {
            match fs::remove_file(&checkpoint_path) {
                Ok(()) => report.already_finalized.push(directory),
                Err(error) => report.failures.push(RecoveryFailure {
                    directory,
                    reason: format!("failed to remove stale checkpoint: {error}"),
                }),
            }
            continue;
        }

        match recover_directory(&directory, &checkpoint_path) {
            Ok(summary) => report.recovered.push(summary),
            Err(error) => report.failures.push(RecoveryFailure {
                directory,
                reason: error.to_string(),
            }),
        }
    }

    Ok(report)
}

fn recover_directory(
    directory: &Path,
    checkpoint_path: &Path,
) -> Result<RecoverySummary, RecordingError> {
    let checkpoint_file = File::open(checkpoint_path).map_err(|source| RecordingError::Io {
        operation: "open recording checkpoint",
        path: checkpoint_path.to_owned(),
        source,
    })?;
    let checkpoint: InProgressCheckpoint =
        serde_json::from_reader(checkpoint_file).map_err(|source| {
            RecordingError::Serialization {
                path: checkpoint_path.to_owned(),
                source,
            }
        })?;
    validate_checkpoint(checkpoint_path, &checkpoint)?;

    let telemetry_path = directory.join(&checkpoint.telemetry_file);
    let recovered = scan_recoverable_telemetry(&telemetry_path, &checkpoint)?;
    if recovered.message_count < checkpoint.flushed_message_count {
        return Err(invalid_recovery(
            &telemetry_path,
            None,
            format!(
                "checkpoint reports {} flushed messages, but only {} complete messages remain",
                checkpoint.flushed_message_count, recovered.message_count
            ),
        ));
    }

    let recovered_telemetry_path = directory.join(RECOVERED_TELEMETRY_FILE);
    copy_prefix_atomic(
        &telemetry_path,
        &recovered_telemetry_path,
        recovered.valid_bytes,
    )?;

    let ply_path = directory.join("pointcloud.ply");
    write_atomic(&ply_path, |writer| {
        write_ply(
            writer,
            &sanitize_session(&checkpoint.session),
            &recovered.points,
        )
    })?;

    let metadata_path = directory.join("metadata.json");
    let metadata = RecordingMetadata {
        protocol_version: 1,
        session: &checkpoint.session,
        frame: &checkpoint.frame,
        unit: &checkpoint.unit,
        message_count: recovered.message_count,
        pose_count: recovered.pose_count,
        pointcloud_message_count: recovered.pointcloud_message_count,
        point_count: recovered.points.len(),
        telemetry_file: RECOVERED_TELEMETRY_FILE,
        pointcloud_file: "pointcloud.ply",
        finalization: "recovered",
        discarded_trailing_bytes: recovered.discarded_trailing_bytes,
    };
    write_atomic(&metadata_path, |writer| {
        serde_json::to_writer_pretty(writer, &metadata).map_err(io::Error::other)
    })?;
    fs::remove_file(checkpoint_path).map_err(|source| RecordingError::Io {
        operation: "remove recovered recording checkpoint",
        path: checkpoint_path.to_owned(),
        source,
    })?;

    Ok(RecoverySummary {
        session: checkpoint.session,
        directory: directory.to_owned(),
        message_count: recovered.message_count,
        point_count: recovered.points.len(),
        discarded_trailing_bytes: recovered.discarded_trailing_bytes,
    })
}

fn validate_checkpoint(
    path: &Path,
    checkpoint: &InProgressCheckpoint,
) -> Result<(), RecordingError> {
    if checkpoint.protocol_version != 1 {
        return Err(invalid_recovery(
            path,
            None,
            format!(
                "protocol_version must be 1, received {}",
                checkpoint.protocol_version
            ),
        ));
    }
    if checkpoint.recording_state != "in_progress" {
        return Err(invalid_recovery(
            path,
            None,
            "recording_state must be \"in_progress\"",
        ));
    }
    if checkpoint.session.trim().is_empty() {
        return Err(invalid_recovery(path, None, "session must not be empty"));
    }
    if checkpoint.frame != "unity_world" || checkpoint.unit != "m" {
        return Err(invalid_recovery(
            path,
            None,
            "checkpoint must use unity_world coordinates in metres",
        ));
    }
    if Path::new(&checkpoint.telemetry_file)
        .file_name()
        .and_then(|name| name.to_str())
        != Some(checkpoint.telemetry_file.as_str())
    {
        return Err(invalid_recovery(
            path,
            None,
            "telemetry_file must be a plain filename",
        ));
    }
    Ok(())
}

fn scan_recoverable_telemetry(
    path: &Path,
    checkpoint: &InProgressCheckpoint,
) -> Result<RecoveredTelemetryState, RecordingError> {
    let file = File::open(path).map_err(|source| RecordingError::Io {
        operation: "open incomplete telemetry",
        path: path.to_owned(),
        source,
    })?;
    let mut reader = BufReader::new(file);
    let mut recovered = RecoveredTelemetryState::default();
    let mut line = Vec::new();
    let mut line_number = 0_usize;

    loop {
        line.clear();
        let bytes = reader
            .read_until(b'\n', &mut line)
            .map_err(|source| RecordingError::Io {
                operation: "read incomplete telemetry",
                path: path.to_owned(),
                source,
            })?;
        if bytes == 0 {
            break;
        }
        line_number += 1;
        if line.last() != Some(&b'\n') {
            recovered.discarded_trailing_bytes = bytes as u64;
            break;
        }

        let payload = &line[..line.len() - 1];
        if payload.is_empty() {
            return Err(invalid_recovery(
                path,
                Some(line_number),
                "blank lines are not allowed",
            ));
        }
        apply_recovered_line(path, line_number, payload, checkpoint, &mut recovered)?;
        recovered.valid_bytes += bytes as u64;
    }

    if !recovered.settings_seen {
        return Err(invalid_recovery(
            path,
            None,
            "telemetry must contain Settings as its first complete message",
        ));
    }
    Ok(recovered)
}

fn apply_recovered_line(
    path: &Path,
    line: usize,
    bytes: &[u8],
    checkpoint: &InProgressCheckpoint,
    recovered: &mut RecoveredTelemetryState,
) -> Result<(), RecordingError> {
    let envelope: RecoveryEnvelope = serde_json::from_slice(bytes).map_err(|error| {
        invalid_recovery(path, Some(line), format!("invalid envelope JSON: {error}"))
    })?;
    if envelope.payload.get().len() > MAX_PAYLOAD_BYTES {
        return Err(invalid_recovery(
            path,
            Some(line),
            format!("payload exceeds {MAX_PAYLOAD_BYTES} bytes"),
        ));
    }

    let message = match envelope.topic.as_str() {
        SETTINGS_TOPIC => {
            let settings: SettingsMessage =
                serde_json::from_str(envelope.payload.get()).map_err(|error| {
                    invalid_recovery(path, Some(line), format!("invalid Settings: {error}"))
                })?;
            validate_recovered_settings(path, line, checkpoint, &settings)?;
            TelemetryMessage::Settings(settings)
        }
        POSE_TOPIC => {
            let pose: PoseMessage =
                serde_json::from_str(envelope.payload.get()).map_err(|error| {
                    invalid_recovery(path, Some(line), format!("invalid Pose: {error}"))
                })?;
            pose.validate().map_err(|error| {
                invalid_recovery(path, Some(line), format!("invalid Pose: {error}"))
            })?;
            TelemetryMessage::Pose(pose)
        }
        POINTCLOUD_TOPIC => {
            let pointcloud: PointCloudMessage = serde_json::from_str(envelope.payload.get())
                .map_err(|error| {
                    invalid_recovery(path, Some(line), format!("invalid PointCloud: {error}"))
                })?;
            pointcloud.validate().map_err(|error| {
                invalid_recovery(path, Some(line), format!("invalid PointCloud: {error}"))
            })?;
            TelemetryMessage::PointCloud(pointcloud)
        }
        topic => {
            return Err(invalid_recovery(
                path,
                Some(line),
                format!("unsupported topic {topic:?}"),
            ));
        }
    };

    if recovered.message_count == 0 && !matches!(message, TelemetryMessage::Settings(_)) {
        return Err(invalid_recovery(
            path,
            Some(line),
            "the first message must be Settings",
        ));
    }
    if message.session() != checkpoint.session {
        return Err(invalid_recovery(
            path,
            Some(line),
            format!(
                "session must be {:?}, received {:?}",
                checkpoint.session,
                message.session()
            ),
        ));
    }

    recovered.message_count += 1;
    match message {
        TelemetryMessage::Settings(_) => recovered.settings_seen = true,
        TelemetryMessage::Pose(_) => recovered.pose_count += 1,
        TelemetryMessage::PointCloud(delta) => {
            recovered.pointcloud_message_count += 1;
            apply_delta(&mut recovered.points, &delta);
        }
    }
    Ok(())
}

fn validate_recovered_settings(
    path: &Path,
    line: usize,
    checkpoint: &InProgressCheckpoint,
    settings: &SettingsMessage,
) -> Result<(), RecordingError> {
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
        return Err(invalid_recovery(
            path,
            Some(line),
            format!("unsupported protocol version: {}", settings.v),
        ));
    }
    if settings.session != checkpoint.session {
        return Err(invalid_recovery(
            path,
            Some(line),
            "Settings session does not match the checkpoint",
        ));
    }
    for (field, actual, expected) in fixed_values {
        if actual != expected {
            return Err(invalid_recovery(
                path,
                Some(line),
                format!("{field} must be {expected:?}, received {actual:?}"),
            ));
        }
    }
    Ok(())
}

fn copy_prefix_atomic(source: &Path, destination: &Path, bytes: u64) -> Result<(), RecordingError> {
    write_atomic(destination, |writer| {
        let file = File::open(source)?;
        let mut prefix = file.take(bytes);
        io::copy(&mut prefix, writer)?;
        Ok(())
    })
}

fn invalid_recovery(path: &Path, line: Option<usize>, reason: impl Into<String>) -> RecordingError {
    RecordingError::InvalidRecovery {
        path: path.to_owned(),
        line,
        reason: reason.into(),
    }
}

pub(crate) fn write_recorded_message(
    writer: &mut BufWriter<File>,
    path: &Path,
    message: &TelemetryMessage,
) -> Result<(), RecordingError> {
    let result = match message {
        TelemetryMessage::Settings(payload) => serde_json::to_writer(
            &mut *writer,
            &RecordedMessage {
                topic: message.topic(),
                payload,
            },
        ),
        TelemetryMessage::Pose(payload) => serde_json::to_writer(
            &mut *writer,
            &RecordedMessage {
                topic: message.topic(),
                payload,
            },
        ),
        TelemetryMessage::PointCloud(payload) => serde_json::to_writer(
            &mut *writer,
            &RecordedMessage {
                topic: message.topic(),
                payload,
            },
        ),
    };

    result.map_err(|source| RecordingError::Serialization {
        path: path.to_owned(),
        source,
    })?;
    writer
        .write_all(b"\n")
        .map_err(|source| RecordingError::Io {
            operation: "append telemetry log",
            path: path.to_owned(),
            source,
        })
}

pub(crate) fn apply_delta(points: &mut BTreeMap<u64, [f64; 3]>, delta: &PointCloudMessage) {
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

pub(crate) fn write_ply(
    writer: &mut dyn Write,
    safe_session: &str,
    points: &BTreeMap<u64, [f64; 3]>,
) -> io::Result<()> {
    writeln!(writer, "ply")?;
    writeln!(writer, "format ascii 1.0")?;
    writeln!(writer, "comment session {safe_session}")?;
    writeln!(writer, "element vertex {}", points.len())?;
    writeln!(writer, "property double x")?;
    writeln!(writer, "property double y")?;
    writeln!(writer, "property double z")?;
    writeln!(writer, "end_header")?;
    for position in points.values() {
        writeln!(writer, "{} {} {}", position[0], position[1], position[2])?;
    }
    Ok(())
}

pub(crate) fn create_directory(path: &Path) -> Result<(), RecordingError> {
    fs::create_dir_all(path).map_err(|source| RecordingError::Io {
        operation: "create directory",
        path: path.to_owned(),
        source,
    })
}

fn create_session_directory(root: &Path, session: &str) -> Result<PathBuf, RecordingError> {
    let base = sanitize_session(session);
    for suffix in 1_u64.. {
        let name = if suffix == 1 {
            base.clone()
        } else {
            format!("{base}-{suffix}")
        };
        let candidate = root.join(name);
        match fs::create_dir(&candidate) {
            Ok(()) => return Ok(candidate),
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(source) => {
                return Err(RecordingError::Io {
                    operation: "create session directory",
                    path: candidate,
                    source,
                });
            }
        }
    }
    unreachable!("u64 session-directory suffixes cannot be exhausted")
}

pub(crate) fn sanitize_session(session: &str) -> String {
    let sanitized: String = session
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() || matches!(character, '-' | '_' | '.') {
                character
            } else {
                '_'
            }
        })
        .collect();

    if sanitized.is_empty() || sanitized == "." || sanitized == ".." {
        "session".to_owned()
    } else {
        sanitized
    }
}

pub(crate) fn write_atomic(
    destination: &Path,
    write: impl FnOnce(&mut dyn Write) -> io::Result<()>,
) -> Result<(), RecordingError> {
    let temporary = destination.with_extension(format!(
        "{}.tmp",
        destination
            .extension()
            .and_then(|extension| extension.to_str())
            .unwrap_or("output")
    ));
    let mut file = File::create(&temporary).map_err(|source| RecordingError::Io {
        operation: "create temporary file",
        path: temporary.clone(),
        source,
    })?;
    write(&mut file).map_err(|source| RecordingError::Io {
        operation: "write temporary file",
        path: temporary.clone(),
        source,
    })?;
    file.flush().map_err(|source| RecordingError::Io {
        operation: "flush temporary file",
        path: temporary.clone(),
        source,
    })?;
    file.sync_all().map_err(|source| RecordingError::Io {
        operation: "sync temporary file",
        path: temporary.clone(),
        source,
    })?;
    fs::rename(&temporary, destination).map_err(|source| RecordingError::Io {
        operation: "finalize file",
        path: destination.to_owned(),
        source,
    })
}

#[cfg(test)]
mod tests {
    use std::{
        fs::{self, OpenOptions},
        io::Write as _,
        sync::atomic::{AtomicU64, Ordering},
    };

    use super::*;
    use crate::{
        playback,
        protocol::{POINTCLOUD_TOPIC, POSE_TOPIC, SETTINGS_TOPIC, parse_telemetry},
    };

    const SETTINGS: &str = include_str!("../../protocol/settings.example.json");
    const POSE: &str = include_str!("../../protocol/pose.example.json");

    static NEXT_TEST_DIRECTORY: AtomicU64 = AtomicU64::new(0);

    struct TestDirectory(PathBuf);

    impl TestDirectory {
        fn new(name: &str) -> Self {
            let id = NEXT_TEST_DIRECTORY.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "slam-receiver-recording-{}-{name}-{id}",
                std::process::id()
            ));
            if path.exists() {
                fs::remove_dir_all(&path).expect("stale test directory should be removable");
            }
            Self(path)
        }
    }

    impl Drop for TestDirectory {
        fn drop(&mut self) {
            if self.0.is_dir() {
                fs::remove_dir_all(&self.0).expect("test directory should be removable");
            } else if self.0.is_file() {
                fs::remove_file(&self.0).expect("test file should be removable");
            }
        }
    }

    fn settings(session: &str) -> TelemetryMessage {
        let mut message = parse_telemetry(SETTINGS_TOPIC, SETTINGS).expect("settings should parse");
        let TelemetryMessage::Settings(settings) = &mut message else {
            unreachable!()
        };
        settings.session = session.to_owned();
        settings.frame = "unity_world".to_owned();
        message
    }

    fn pointcloud(
        session: &str,
        add: Vec<(u64, f64, f64, f64)>,
        update: Vec<(u64, f64, f64, f64)>,
        remove: Vec<u64>,
    ) -> TelemetryMessage {
        TelemetryMessage::PointCloud(PointCloudMessage {
            v: 1,
            session: session.to_owned(),
            seq: 0,
            t: 0.0,
            add,
            update,
            remove,
        })
    }

    fn leave_incomplete_recording(
        root: &Path,
        session: &str,
        messages: &[TelemetryMessage],
    ) -> PathBuf {
        let settings_message = settings(session);
        let TelemetryMessage::Settings(settings) = &settings_message else {
            unreachable!()
        };
        let mut recording =
            SessionRecording::start(root, settings).expect("incomplete recording should start");
        recording
            .append(&settings_message)
            .expect("settings should be recorded");
        for message in messages {
            recording
                .append(message)
                .expect("telemetry should be recorded");
        }
        recording
            .checkpoint(true)
            .expect("telemetry should be checkpointed");
        let directory = recording.directory.clone();
        drop(recording);
        directory
    }

    #[test]
    fn session_gate_requires_settings_and_rejects_mismatches() {
        let mut gate = SessionGate::new();
        let pose = parse_telemetry(POSE_TOPIC, POSE).expect("pose should parse");

        assert!(matches!(
            gate.accept(&pose),
            Err(SessionRejection::SettingsRequired { .. })
        ));
        gate.accept(&settings("session-a"))
            .expect("settings should establish session");
        assert!(matches!(
            gate.accept(&pose),
            Err(SessionRejection::MismatchedSession { .. })
        ));
    }

    #[test]
    fn applies_deltas_and_exports_points_in_id_order() {
        let directory = TestDirectory::new("deltas");
        let recorder = TelemetryRecorder::start(&directory.0);
        recorder.record(&settings("test")).expect("settings");
        recorder
            .record(&pointcloud(
                "test",
                vec![(2, 2.0, 2.0, 2.0), (1, 1.0, 1.0, 1.0)],
                vec![],
                vec![],
            ))
            .expect("first delta");
        recorder
            .record(&pointcloud(
                "test",
                vec![(2, 20.0, 20.0, 20.0)],
                vec![(3, 3.0, 3.0, 3.0), (1, 10.0, 10.0, 10.0)],
                vec![2, 999],
            ))
            .expect("second delta");

        let summaries = recorder.finish().expect("recording should finish");
        assert_eq!(summaries.len(), 1);
        assert_eq!(summaries[0].point_count, 3);

        let ply = fs::read_to_string(summaries[0].directory.join("pointcloud.ply"))
            .expect("PLY should be readable");
        assert!(ply.contains("element vertex 3"));
        assert!(ply.ends_with("10 10 10\n20 20 20\n3 3 3\n"));
    }

    #[test]
    fn session_change_finalizes_separate_outputs() {
        let directory = TestDirectory::new("sessions");
        let recorder = TelemetryRecorder::start(&directory.0);
        recorder.record(&settings("first")).expect("first settings");
        recorder
            .record(&pointcloud(
                "first",
                vec![(1, 1.0, 0.0, 0.0)],
                vec![],
                vec![],
            ))
            .expect("first points");
        recorder
            .record(&settings("second"))
            .expect("second settings");
        recorder
            .record(&pointcloud(
                "second",
                vec![(2, 2.0, 0.0, 0.0)],
                vec![],
                vec![],
            ))
            .expect("second points");

        let summaries = recorder.finish().expect("recording should finish");
        assert_eq!(summaries.len(), 2);
        assert_eq!(summaries[0].session, "first");
        assert_eq!(summaries[1].session, "second");
        assert_ne!(summaries[0].directory, summaries[1].directory);
        assert_eq!(summaries[0].point_count, 1);
        assert_eq!(summaries[1].point_count, 1);
    }

    #[test]
    fn records_messages_in_arrival_order_and_writes_metadata() {
        let directory = TestDirectory::new("telemetry");
        let recorder = TelemetryRecorder::start(&directory.0);
        recorder.record(&settings("test")).expect("settings");
        recorder
            .record(&pointcloud(
                "test",
                vec![(1, 1.0, 2.0, 3.0)],
                vec![],
                vec![],
            ))
            .expect("points");
        let summary = recorder
            .finish()
            .expect("recording should finish")
            .remove(0);

        let telemetry = fs::read_to_string(summary.directory.join("telemetry.ndjson"))
            .expect("telemetry should be readable");
        let lines: Vec<_> = telemetry.lines().collect();
        assert_eq!(lines.len(), 2);
        assert!(lines[0].contains(SETTINGS_TOPIC));
        assert!(lines[1].contains(POINTCLOUD_TOPIC));

        let metadata: serde_json::Value = serde_json::from_str(
            &fs::read_to_string(summary.directory.join("metadata.json"))
                .expect("metadata should be readable"),
        )
        .expect("metadata should be JSON");
        assert_eq!(metadata["session"], "test");
        assert_eq!(metadata["frame"], "unity_world");
        assert_eq!(metadata["message_count"], 2);
        assert_eq!(metadata["point_count"], 1);
        assert_eq!(metadata["finalization"], "clean");
        assert_eq!(metadata["discarded_trailing_bytes"], 0);
        assert!(!summary.directory.join(CHECKPOINT_FILE).exists());
    }

    #[test]
    fn recovers_incomplete_recording_and_rebuilds_final_points() {
        let root = TestDirectory::new("recover");
        let directory = leave_incomplete_recording(
            &root.0,
            "recover-test",
            &[
                pointcloud(
                    "recover-test",
                    vec![(1, 1.0, 2.0, 3.0), (2, 4.0, 5.0, 6.0)],
                    vec![],
                    vec![],
                ),
                pointcloud(
                    "recover-test",
                    vec![(3, 7.0, 8.0, 9.0)],
                    vec![(1, 10.0, 20.0, 30.0)],
                    vec![2],
                ),
            ],
        );

        let report = recover_incomplete_recordings(&root.0).expect("recovery should run");
        assert_eq!(report.recovered.len(), 1);
        assert!(report.failures.is_empty());
        assert_eq!(report.recovered[0].message_count, 3);
        assert_eq!(report.recovered[0].point_count, 2);
        assert_eq!(report.recovered[0].discarded_trailing_bytes, 0);
        assert!(!directory.join(CHECKPOINT_FILE).exists());

        let metadata: serde_json::Value = serde_json::from_slice(
            &fs::read(directory.join("metadata.json")).expect("metadata should be readable"),
        )
        .expect("metadata should be JSON");
        assert_eq!(metadata["finalization"], "recovered");
        assert_eq!(metadata["telemetry_file"], RECOVERED_TELEMETRY_FILE);
        assert_eq!(metadata["message_count"], 3);
        assert_eq!(metadata["point_count"], 2);

        let ply = fs::read_to_string(directory.join("pointcloud.ply"))
            .expect("recovered PLY should be readable");
        assert!(ply.contains("element vertex 2"));
        assert!(ply.ends_with("10 20 30\n7 8 9\n"));

        let loaded = playback::load_session(&directory)
            .expect("recovered recording should remain replayable");
        assert_eq!(loaded.messages().len(), 3);

        let second = recover_incomplete_recordings(&root.0).expect("second scan should succeed");
        assert!(second.recovered.is_empty());
        assert!(second.already_finalized.is_empty());
        assert!(second.failures.is_empty());
    }

    #[test]
    fn discards_only_an_unterminated_final_line_and_preserves_source() {
        let root = TestDirectory::new("truncated-tail");
        let directory = leave_incomplete_recording(
            &root.0,
            "truncated-test",
            &[pointcloud(
                "truncated-test",
                vec![(1, 1.0, 2.0, 3.0)],
                vec![],
                vec![],
            )],
        );
        let telemetry_path = directory.join("telemetry.ndjson");
        let truncated = br#"{"topic":"slam/v1/pose","payload":{"v":1"#;
        OpenOptions::new()
            .append(true)
            .open(&telemetry_path)
            .expect("telemetry should open")
            .write_all(truncated)
            .expect("truncated line should append");
        let source_before = fs::read(&telemetry_path).expect("source should be readable");

        let report = recover_incomplete_recordings(&root.0).expect("recovery should run");
        assert!(report.failures.is_empty());
        assert_eq!(
            report.recovered[0].discarded_trailing_bytes,
            truncated.len() as u64
        );
        assert_eq!(
            fs::read(&telemetry_path).expect("source should remain readable"),
            source_before
        );

        let recovered = fs::read(directory.join(RECOVERED_TELEMETRY_FILE))
            .expect("recovered telemetry should be readable");
        assert_eq!(
            recovered,
            &source_before[..source_before.len() - truncated.len()]
        );
        playback::load_session(&directory).expect("trimmed recording should replay");
    }

    #[test]
    fn complete_invalid_line_fails_without_modifying_source() {
        let root = TestDirectory::new("invalid-line");
        let directory = leave_incomplete_recording(&root.0, "invalid-test", &[]);
        let telemetry_path = directory.join("telemetry.ndjson");
        OpenOptions::new()
            .append(true)
            .open(&telemetry_path)
            .expect("telemetry should open")
            .write_all(b"not-json\n")
            .expect("invalid line should append");
        let source_before = fs::read(&telemetry_path).expect("source should be readable");

        let report = recover_incomplete_recordings(&root.0).expect("scan should complete");
        assert!(report.recovered.is_empty());
        assert_eq!(report.failures.len(), 1);
        assert!(report.failures[0].reason.contains("line 2"));
        assert!(directory.join(CHECKPOINT_FILE).exists());
        assert!(!directory.join("metadata.json").exists());
        assert_eq!(
            fs::read(&telemetry_path).expect("source should remain readable"),
            source_before
        );
    }

    #[test]
    fn stale_checkpoint_never_overwrites_finalized_recording() {
        let root = TestDirectory::new("finalized");
        let recorder = TelemetryRecorder::start(&root.0);
        recorder.record(&settings("finished")).expect("settings");
        let summary = recorder
            .finish()
            .expect("recording should finish")
            .remove(0);
        let metadata_path = summary.directory.join("metadata.json");
        let metadata_before = fs::read(&metadata_path).expect("metadata should be readable");
        fs::write(summary.directory.join(CHECKPOINT_FILE), b"stale checkpoint")
            .expect("stale marker should be writable");

        let report = recover_incomplete_recordings(&root.0).expect("scan should complete");
        assert!(report.recovered.is_empty());
        assert_eq!(report.already_finalized, vec![summary.directory.clone()]);
        assert!(report.failures.is_empty());
        assert_eq!(
            fs::read(metadata_path).expect("metadata should remain readable"),
            metadata_before
        );
        assert!(!summary.directory.join(CHECKPOINT_FILE).exists());
    }

    #[test]
    fn recovery_ignores_clip_directories() {
        let root = TestDirectory::new("ignore-clips");
        let clip_directory = root.0.join("clips").join("unfinished-clip");
        fs::create_dir_all(&clip_directory).expect("clip directory should be created");
        let marker = clip_directory.join(CHECKPOINT_FILE);
        fs::write(&marker, b"not a full-session checkpoint")
            .expect("clip marker should be writable");

        let report = recover_incomplete_recordings(&root.0).expect("scan should complete");
        assert!(report.recovered.is_empty());
        assert!(report.already_finalized.is_empty());
        assert!(report.failures.is_empty());
        assert!(marker.exists());
    }

    #[test]
    fn reports_output_directory_write_failures() {
        let path = TestDirectory::new("write-failure");
        fs::write(&path.0, b"not a directory").expect("test file should be writable");
        let recorder = TelemetryRecorder::start(&path.0);
        recorder
            .record(&settings("test"))
            .expect("message should enqueue");

        let error = recorder
            .finish()
            .expect_err("invalid output root should fail");
        assert!(error.to_string().contains("create directory"));
        assert!(
            error
                .to_string()
                .contains(path.0.to_string_lossy().as_ref())
        );
    }

    #[test]
    fn sanitizes_session_directory_names() {
        assert_eq!(
            sanitize_session("../../unsafe session"),
            ".._.._unsafe_session"
        );
        assert_eq!(sanitize_session("."), "session");
        assert_eq!(sanitize_session("safe-01"), "safe-01");
    }
}
