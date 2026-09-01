use std::{
    collections::BTreeMap,
    error::Error,
    fmt,
    fs::{self, File},
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
    sync::mpsc::{self, Sender},
    thread::{self, JoinHandle},
};

use serde::Serialize;

use crate::protocol::{PointCloudMessage, SettingsMessage, TelemetryMessage};

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
            Self::WorkerStopped | Self::WorkerPanicked => None,
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

    for message in receiver {
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

        Ok(Self {
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
        })
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

        Ok(())
    }

    fn finish(mut self) -> Result<RecordingSummary, RecordingError> {
        self.telemetry
            .flush()
            .map_err(|source| RecordingError::Io {
                operation: "flush telemetry log",
                path: self.telemetry_path.clone(),
                source,
            })?;

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
        };
        write_atomic(&metadata_path, |writer| {
            serde_json::to_writer_pretty(writer, &metadata).map_err(io::Error::other)
        })?;

        Ok(summary)
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
    fs::rename(&temporary, destination).map_err(|source| RecordingError::Io {
        operation: "finalize file",
        path: destination.to_owned(),
        source,
    })
}

#[cfg(test)]
mod tests {
    use std::{
        fs,
        sync::atomic::{AtomicU64, Ordering},
    };

    use super::*;
    use crate::protocol::{POINTCLOUD_TOPIC, POSE_TOPIC, SETTINGS_TOPIC, parse_telemetry};

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
