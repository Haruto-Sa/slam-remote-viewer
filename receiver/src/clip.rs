use std::{
    collections::BTreeMap,
    error::Error,
    fmt,
    fs::{self, File},
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex,
        mpsc::{self, Sender},
    },
    thread::{self, JoinHandle},
};

use serde::Serialize;

use crate::{
    protocol::{PointCloudMessage, PoseMessage, SettingsMessage, TelemetryMessage},
    recording::{
        RecordingError, apply_delta, create_directory, sanitize_session, write_atomic, write_ply,
        write_recorded_message,
    },
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ClipState {
    Idle,
    Recording,
    Finalizing,
    Completed,
    Failed,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ClipStatus {
    pub state: ClipState,
    pub session: Option<String>,
    pub elapsed_seconds: f64,
    pub message_count: u64,
    pub output_path: Option<PathBuf>,
    pub error: Option<String>,
}

impl Default for ClipStatus {
    fn default() -> Self {
        Self {
            state: ClipState::Idle,
            session: None,
            elapsed_seconds: 0.0,
            message_count: 0,
            output_path: None,
            error: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ClipCommandError {
    InvalidState(ClipState),
    WorkerStopped,
}

impl fmt::Display for ClipCommandError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidState(state) => {
                write!(
                    formatter,
                    "clip command is not valid while state is {state:?}"
                )
            }
            Self::WorkerStopped => write!(formatter, "clip recording worker stopped unexpectedly"),
        }
    }
}

impl Error for ClipCommandError {}

#[derive(Debug, Clone, PartialEq)]
pub struct ClipSummary {
    pub session: String,
    pub directory: PathBuf,
    pub message_count: u64,
    pub interval_message_count: u64,
    pub point_count: usize,
}

enum ClipWorkerCommand {
    Observe(Box<TelemetryMessage>),
    Start,
    Stop,
}

pub struct ClipRecorder {
    sender: Option<Sender<ClipWorkerCommand>>,
    status: Arc<Mutex<ClipStatus>>,
    worker: Option<JoinHandle<Vec<ClipSummary>>>,
}

impl ClipRecorder {
    pub fn start(root: impl Into<PathBuf>) -> Self {
        let root = root.into();
        let (sender, receiver) = mpsc::channel();
        let status = Arc::new(Mutex::new(ClipStatus::default()));
        let worker_status = Arc::clone(&status);
        let worker = thread::spawn(move || run_worker(&root, receiver, &worker_status));
        Self {
            sender: Some(sender),
            status,
            worker: Some(worker),
        }
    }

    pub fn observe(&self, message: &TelemetryMessage) -> Result<(), ClipCommandError> {
        self.send(ClipWorkerCommand::Observe(Box::new(message.clone())))
    }

    pub fn start_clip(&self) -> Result<(), ClipCommandError> {
        {
            let mut status = self
                .status
                .lock()
                .map_err(|_| ClipCommandError::WorkerStopped)?;
            if matches!(status.state, ClipState::Recording | ClipState::Finalizing) {
                return Err(ClipCommandError::InvalidState(status.state));
            }
            *status = ClipStatus {
                state: ClipState::Recording,
                ..ClipStatus::default()
            };
        }
        if let Err(error) = self.send(ClipWorkerCommand::Start) {
            set_failed(&self.status, error.to_string(), None);
            return Err(error);
        }
        Ok(())
    }

    pub fn stop_clip(&self) -> Result<(), ClipCommandError> {
        {
            let mut status = self
                .status
                .lock()
                .map_err(|_| ClipCommandError::WorkerStopped)?;
            if status.state != ClipState::Recording {
                return Err(ClipCommandError::InvalidState(status.state));
            }
            status.state = ClipState::Finalizing;
        }
        if let Err(error) = self.send(ClipWorkerCommand::Stop) {
            set_failed(&self.status, error.to_string(), None);
            return Err(error);
        }
        Ok(())
    }

    pub fn status(&self) -> ClipStatus {
        self.status.lock().map_or_else(
            |_| ClipStatus {
                state: ClipState::Failed,
                error: Some("clip status lock was poisoned".to_owned()),
                ..ClipStatus::default()
            },
            |status| status.clone(),
        )
    }

    pub fn finish(mut self) -> Result<Vec<ClipSummary>, ClipCommandError> {
        self.sender.take();
        self.worker
            .take()
            .ok_or(ClipCommandError::WorkerStopped)?
            .join()
            .map_err(|_| ClipCommandError::WorkerStopped)
    }

    fn send(&self, command: ClipWorkerCommand) -> Result<(), ClipCommandError> {
        self.sender
            .as_ref()
            .ok_or(ClipCommandError::WorkerStopped)?
            .send(command)
            .map_err(|_| ClipCommandError::WorkerStopped)
    }
}

#[derive(Default)]
struct SourceState {
    settings: Option<SettingsMessage>,
    latest_pose: Option<PoseMessage>,
    points: BTreeMap<u64, [f64; 3]>,
    latest_timestamp: Option<f64>,
    latest_pointcloud_seq: Option<u64>,
    message_count: u64,
}

impl SourceState {
    fn session(&self) -> Option<&str> {
        self.settings
            .as_ref()
            .map(|settings| settings.session.as_str())
    }

    fn observe(&mut self, message: &TelemetryMessage) {
        self.message_count += 1;
        match message {
            TelemetryMessage::Settings(settings) => self.settings = Some(settings.clone()),
            TelemetryMessage::Pose(pose) => {
                self.latest_timestamp = Some(pose.t);
                self.latest_pose = Some(pose.clone());
            }
            TelemetryMessage::PointCloud(delta) => {
                self.latest_timestamp = Some(delta.t);
                self.latest_pointcloud_seq = Some(delta.seq);
                apply_delta(&mut self.points, delta);
            }
        }
    }
}

struct ClipDraft {
    session: String,
    frame: String,
    unit: String,
    source_start_message_index: u64,
    source_start_time_seconds: f64,
    source_end_message_index_exclusive: u64,
    source_end_time_seconds: f64,
    interval_message_count: u64,
    messages: Vec<TelemetryMessage>,
    points: BTreeMap<u64, [f64; 3]>,
}

impl ClipDraft {
    fn start(source: &SourceState) -> Result<Self, &'static str> {
        let settings = source
            .settings
            .as_ref()
            .ok_or("Settings have not been received")?;
        let pose = source
            .latest_pose
            .as_ref()
            .ok_or("a camera Pose has not been received")?;
        let start_time = source.latest_timestamp.unwrap_or(pose.t);
        let mut initial_pose = pose.clone();
        initial_pose.t = start_time;
        let snapshot = TelemetryMessage::PointCloud(PointCloudMessage {
            v: settings.v,
            session: settings.session.clone(),
            seq: source.latest_pointcloud_seq.unwrap_or(0),
            t: start_time,
            add: source
                .points
                .iter()
                .map(|(&id, position)| (id, position[0], position[1], position[2]))
                .collect(),
            update: Vec::new(),
            remove: Vec::new(),
        });

        Ok(Self {
            session: settings.session.clone(),
            frame: settings.frame.clone(),
            unit: settings.unit.clone(),
            source_start_message_index: source.message_count,
            source_start_time_seconds: start_time,
            source_end_message_index_exclusive: source.message_count,
            source_end_time_seconds: start_time,
            interval_message_count: 0,
            messages: vec![
                TelemetryMessage::Settings(settings.clone()),
                TelemetryMessage::Pose(initial_pose),
                snapshot,
            ],
            points: source.points.clone(),
        })
    }

    fn observe(&mut self, message: &TelemetryMessage, source: &SourceState) {
        self.messages.push(message.clone());
        self.interval_message_count += 1;
        self.source_end_message_index_exclusive = source.message_count;
        if let Some(timestamp) = message_timestamp(message) {
            self.source_end_time_seconds = self
                .source_end_time_seconds
                .max(timestamp)
                .max(self.source_start_time_seconds);
        }
        if let TelemetryMessage::PointCloud(delta) = message {
            apply_delta(&mut self.points, delta);
        }
    }
}

fn run_worker(
    root: &Path,
    receiver: mpsc::Receiver<ClipWorkerCommand>,
    status: &Arc<Mutex<ClipStatus>>,
) -> Vec<ClipSummary> {
    let mut source = SourceState::default();
    let mut active: Option<ClipDraft> = None;
    let mut summaries = Vec::new();

    for command in receiver {
        match command {
            ClipWorkerCommand::Observe(message) => {
                let session_changed = matches!(
                    message.as_ref(),
                    TelemetryMessage::Settings(settings)
                        if source.session().is_some_and(|session| session != settings.session)
                );
                if session_changed {
                    if active.is_some() {
                        finalize_active(root, &mut active, status, &mut summaries);
                    }
                    source = SourceState::default();
                }

                source.observe(message.as_ref());
                if let Some(draft) = &mut active
                    && draft.session == message.session()
                {
                    draft.observe(message.as_ref(), &source);
                    set_recording_status(status, draft);
                }
            }
            ClipWorkerCommand::Start => match ClipDraft::start(&source) {
                Ok(draft) => {
                    set_recording_status(status, &draft);
                    active = Some(draft);
                }
                Err(reason) => set_failed(status, reason.to_owned(), source.session()),
            },
            ClipWorkerCommand::Stop => {
                finalize_active(root, &mut active, status, &mut summaries);
            }
        }
    }

    if active.is_some() {
        finalize_active(root, &mut active, status, &mut summaries);
    }
    summaries
}

fn finalize_active(
    root: &Path,
    active: &mut Option<ClipDraft>,
    status: &Arc<Mutex<ClipStatus>>,
    summaries: &mut Vec<ClipSummary>,
) {
    let Some(draft) = active.take() else {
        set_failed(status, "no clip is currently recording".to_owned(), None);
        return;
    };

    set_finalizing_status(status, &draft);
    match write_clip(root, draft) {
        Ok(summary) => {
            set_completed_status(status, &summary);
            summaries.push(summary);
        }
        Err(error) => set_failed(status, error.to_string(), None),
    }
}

fn write_clip(root: &Path, draft: ClipDraft) -> Result<ClipSummary, RecordingError> {
    let clips_root = root.join("clips");
    create_directory(&clips_root)?;
    let directory = create_clip_directory(&clips_root, &draft.session)?;
    let telemetry_path = directory.join("telemetry.ndjson");
    let telemetry_file = File::create(&telemetry_path).map_err(|source| RecordingError::Io {
        operation: "create clip telemetry log",
        path: telemetry_path.clone(),
        source,
    })?;
    let mut telemetry = BufWriter::new(telemetry_file);
    for message in &draft.messages {
        write_recorded_message(&mut telemetry, &telemetry_path, message)?;
    }
    telemetry.flush().map_err(|source| RecordingError::Io {
        operation: "flush clip telemetry log",
        path: telemetry_path.clone(),
        source,
    })?;

    let message_count = draft.messages.len() as u64;
    let pose_count = draft
        .messages
        .iter()
        .filter(|message| matches!(message, TelemetryMessage::Pose(_)))
        .count() as u64;
    let pointcloud_message_count = draft
        .messages
        .iter()
        .filter(|message| matches!(message, TelemetryMessage::PointCloud(_)))
        .count() as u64;

    let ply_path = directory.join("pointcloud.ply");
    write_atomic(&ply_path, |writer| {
        write_ply(writer, &sanitize_session(&draft.session), &draft.points)
    })?;

    let metadata_path = directory.join("metadata.json");
    let metadata = ClipMetadata {
        protocol_version: 1,
        recording_type: "clip",
        session: &draft.session,
        source_session: &draft.session,
        frame: &draft.frame,
        unit: &draft.unit,
        message_count,
        pose_count,
        pointcloud_message_count,
        point_count: draft.points.len(),
        source_start_message_index: draft.source_start_message_index,
        source_end_message_index_exclusive: draft.source_end_message_index_exclusive,
        source_start_time_seconds: draft.source_start_time_seconds,
        source_end_time_seconds: draft.source_end_time_seconds,
        interval_message_count: draft.interval_message_count,
        telemetry_file: "telemetry.ndjson",
        pointcloud_file: "pointcloud.ply",
    };
    write_atomic(&metadata_path, |writer| {
        serde_json::to_writer_pretty(writer, &metadata).map_err(io::Error::other)
    })?;

    Ok(ClipSummary {
        session: draft.session,
        directory,
        message_count,
        interval_message_count: draft.interval_message_count,
        point_count: draft.points.len(),
    })
}

#[derive(Serialize)]
struct ClipMetadata<'a> {
    protocol_version: u32,
    recording_type: &'static str,
    session: &'a str,
    source_session: &'a str,
    frame: &'a str,
    unit: &'a str,
    message_count: u64,
    pose_count: u64,
    pointcloud_message_count: u64,
    point_count: usize,
    source_start_message_index: u64,
    source_end_message_index_exclusive: u64,
    source_start_time_seconds: f64,
    source_end_time_seconds: f64,
    interval_message_count: u64,
    telemetry_file: &'static str,
    pointcloud_file: &'static str,
}

fn create_clip_directory(root: &Path, session: &str) -> Result<PathBuf, RecordingError> {
    let base = format!("{}-clip", sanitize_session(session));
    for suffix in 1_u64.. {
        let candidate = root.join(format!("{base}-{suffix:03}"));
        match fs::create_dir(&candidate) {
            Ok(()) => return Ok(candidate),
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(source) => {
                return Err(RecordingError::Io {
                    operation: "create clip directory",
                    path: candidate,
                    source,
                });
            }
        }
    }
    unreachable!("u64 clip-directory suffixes cannot be exhausted")
}

fn message_timestamp(message: &TelemetryMessage) -> Option<f64> {
    match message {
        TelemetryMessage::Settings(_) => None,
        TelemetryMessage::Pose(pose) => Some(pose.t),
        TelemetryMessage::PointCloud(points) => Some(points.t),
    }
}

fn set_recording_status(status: &Arc<Mutex<ClipStatus>>, draft: &ClipDraft) {
    if let Ok(mut status) = status.lock() {
        *status = ClipStatus {
            state: ClipState::Recording,
            session: Some(draft.session.clone()),
            elapsed_seconds: (draft.source_end_time_seconds - draft.source_start_time_seconds)
                .max(0.0),
            message_count: draft.interval_message_count,
            output_path: None,
            error: None,
        };
    }
}

fn set_finalizing_status(status: &Arc<Mutex<ClipStatus>>, draft: &ClipDraft) {
    if let Ok(mut status) = status.lock() {
        status.state = ClipState::Finalizing;
        status.session = Some(draft.session.clone());
        status.elapsed_seconds =
            (draft.source_end_time_seconds - draft.source_start_time_seconds).max(0.0);
        status.message_count = draft.interval_message_count;
        status.output_path = None;
        status.error = None;
    }
}

fn set_completed_status(status: &Arc<Mutex<ClipStatus>>, summary: &ClipSummary) {
    if let Ok(mut status) = status.lock() {
        status.state = ClipState::Completed;
        status.session = Some(summary.session.clone());
        status.output_path = Some(summary.directory.clone());
        status.error = None;
    }
}

fn set_failed(status: &Arc<Mutex<ClipStatus>>, error: String, session: Option<&str>) {
    if let Ok(mut status) = status.lock() {
        *status = ClipStatus {
            state: ClipState::Failed,
            session: session.map(str::to_owned),
            error: Some(error),
            ..ClipStatus::default()
        };
    }
}

#[cfg(test)]
mod tests {
    use std::{
        fs,
        sync::atomic::{AtomicU64, Ordering},
        thread,
        time::{Duration, Instant},
    };

    use super::*;
    use crate::{
        playback::load_session,
        protocol::{CameraSettings, PoseState},
    };

    static NEXT_TEST_DIRECTORY: AtomicU64 = AtomicU64::new(0);

    struct TestDirectory(PathBuf);

    impl TestDirectory {
        fn new(name: &str) -> Self {
            let id = NEXT_TEST_DIRECTORY.fetch_add(1, Ordering::Relaxed);
            Self(std::env::temp_dir().join(format!(
                "slam-receiver-clips-{}-{name}-{id}",
                std::process::id()
            )))
        }
    }

    impl Drop for TestDirectory {
        fn drop(&mut self) {
            if self.0.exists() {
                fs::remove_dir_all(&self.0).expect("test directory should be removable");
            }
        }
    }

    #[test]
    fn creates_replayable_clip_with_initial_state_and_interval_deltas() {
        let root = TestDirectory::new("initial-state");
        let recorder = ClipRecorder::start(&root.0);
        recorder.observe(&settings("source")).expect("settings");
        recorder
            .observe(&pose("source", 1, 10.0, 1.0))
            .expect("pose");
        recorder
            .observe(&points(
                "source",
                1,
                10.5,
                vec![(7, 1.0, 2.0, 3.0)],
                vec![],
                vec![],
            ))
            .expect("initial points");
        recorder.start_clip().expect("start clip");
        recorder
            .observe(&pose("source", 2, 11.0, 2.0))
            .expect("interval pose");
        recorder
            .observe(&points(
                "source",
                2,
                11.5,
                vec![(8, 8.0, 8.0, 8.0)],
                vec![(7, 4.0, 5.0, 6.0)],
                vec![],
            ))
            .expect("interval points");
        recorder.stop_clip().expect("stop clip");

        let summaries = recorder.finish().expect("worker should finish");
        assert_eq!(summaries.len(), 1);
        assert_eq!(summaries[0].interval_message_count, 2);
        assert_eq!(summaries[0].message_count, 5);
        assert_eq!(summaries[0].point_count, 2);

        let loaded = load_session(&summaries[0].directory).expect("clip should replay");
        assert_eq!(loaded.messages().len(), 5);
        assert_eq!(loaded.messages()[0].topic(), "slam/v1/settings");
        assert_eq!(loaded.messages()[1].topic(), "slam/v1/pose");
        assert_eq!(loaded.messages()[2].topic(), "slam/v1/pointcloud");

        let metadata: serde_json::Value = serde_json::from_slice(
            &fs::read(summaries[0].directory.join("metadata.json")).expect("metadata"),
        )
        .expect("metadata JSON");
        assert_eq!(metadata["recording_type"], "clip");
        assert_eq!(metadata["source_session"], "source");
        assert_eq!(metadata["source_start_message_index"], 3);
        assert_eq!(metadata["source_end_message_index_exclusive"], 5);
        assert_eq!(metadata["interval_message_count"], 2);

        let ply = fs::read_to_string(summaries[0].directory.join("pointcloud.ply")).expect("PLY");
        assert!(ply.contains("element vertex 2"));
        assert!(ply.ends_with("4 5 6\n8 8 8\n"));
    }

    #[test]
    fn rejects_overlapping_clips_and_supports_multiple_completed_clips() {
        let root = TestDirectory::new("multiple");
        let recorder = ClipRecorder::start(&root.0);
        recorder.observe(&settings("source")).expect("settings");
        recorder
            .observe(&pose("source", 1, 1.0, 0.0))
            .expect("pose");

        recorder.start_clip().expect("first start");
        assert!(matches!(
            recorder.start_clip(),
            Err(ClipCommandError::InvalidState(ClipState::Recording))
        ));
        recorder.stop_clip().expect("first stop");
        wait_for_state(&recorder, ClipState::Completed);

        recorder.start_clip().expect("second start");
        recorder
            .observe(&pose("source", 2, 2.0, 1.0))
            .expect("pose");
        recorder.stop_clip().expect("second stop");

        let summaries = recorder.finish().expect("worker should finish");
        assert_eq!(summaries.len(), 2);
        assert_ne!(summaries[0].directory, summaries[1].directory);
    }

    #[test]
    fn reports_failed_state_when_start_has_no_pose() {
        let root = TestDirectory::new("missing-pose");
        let recorder = ClipRecorder::start(&root.0);
        recorder.observe(&settings("source")).expect("settings");
        recorder.start_clip().expect("command should enqueue");

        wait_for_state(&recorder, ClipState::Failed);
        assert!(recorder.status().error.unwrap().contains("Pose"));
        recorder.finish().expect("worker should finish");
    }

    fn wait_for_state(recorder: &ClipRecorder, expected: ClipState) {
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            if recorder.status().state == expected {
                return;
            }
            thread::sleep(Duration::from_millis(5));
        }
        panic!(
            "clip state did not become {expected:?}: {:?}",
            recorder.status()
        );
    }

    fn settings(session: &str) -> TelemetryMessage {
        TelemetryMessage::Settings(SettingsMessage {
            v: 1,
            session: session.to_owned(),
            unit: "m".to_owned(),
            frame: "unity_world".to_owned(),
            pose_convention: "Twc".to_owned(),
            quaternion: "xyzw".to_owned(),
            camera: CameraSettings {
                camera_type: "mock".to_owned(),
                id: "camera".to_owned(),
                width: 640,
                height: 480,
                fps: 30,
            },
            pointcloud_mode: "delta".to_owned(),
        })
    }

    fn pose(session: &str, seq: u64, t: f64, x: f64) -> TelemetryMessage {
        TelemetryMessage::Pose(PoseMessage {
            v: 1,
            session: session.to_owned(),
            seq,
            t,
            p: [x, 0.0, 0.0],
            q: [0.0, 0.0, 0.0, 1.0],
            state: PoseState::Tracking,
        })
    }

    fn points(
        session: &str,
        seq: u64,
        t: f64,
        add: Vec<(u64, f64, f64, f64)>,
        update: Vec<(u64, f64, f64, f64)>,
        remove: Vec<u64>,
    ) -> TelemetryMessage {
        TelemetryMessage::PointCloud(PointCloudMessage {
            v: 1,
            session: session.to_owned(),
            seq,
            t,
            add,
            update,
            remove,
        })
    }
}
