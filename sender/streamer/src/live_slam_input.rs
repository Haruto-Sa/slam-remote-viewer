use std::{
    error::Error,
    fmt, fs, io,
    os::unix::{
        fs::{FileTypeExt, MetadataExt},
        net::{UnixListener, UnixStream},
    },
    path::{Path, PathBuf},
};

use crate::{
    pose_source::{SlamPose, SlamTrackingState},
    slam_boundary::{
        BoundaryCamera, BoundaryDecodeError, BoundaryMapPoint, BoundaryMessage,
        BoundarySessionValidator, BoundaryTrackingState, BoundaryValidationError, decode_frame,
    },
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SlamCameraInfo {
    pub camera_type: String,
    pub id: String,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LiveTrackingFrame {
    pub frame_id: u64,
    pub timestamp_seconds: f64,
    pub tracking_state: SlamTrackingState,
    pub pose: Option<SlamPose>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SlamMapPoint {
    pub id: u64,
    pub position: [f64; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub struct LivePointCloudDelta {
    pub frame_id: u64,
    pub timestamp_seconds: f64,
    pub add: Vec<SlamMapPoint>,
    pub update: Vec<SlamMapPoint>,
    pub remove: Vec<u64>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum LiveSlamEvent {
    SessionStarted {
        session_id: String,
        producer: String,
        camera: SlamCameraInfo,
    },
    TrackingFrame(LiveTrackingFrame),
    PointCloudDelta(LivePointCloudDelta),
    SessionEnded {
        session_id: String,
        reason: String,
    },
}

#[derive(Debug)]
pub enum LiveSlamInputError {
    Io(io::Error),
    Decode(BoundaryDecodeError),
    Validation(BoundaryValidationError),
    ProducerAlreadyAccepted,
    DisconnectedBeforeSessionEnd,
}

impl fmt::Display for LiveSlamInputError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "live SLAM input I/O failed: {error}"),
            Self::Decode(error) => write!(formatter, "live SLAM input frame failed: {error}"),
            Self::Validation(error) => {
                write!(formatter, "live SLAM input validation failed: {error}")
            }
            Self::ProducerAlreadyAccepted => {
                write!(
                    formatter,
                    "live SLAM listener already accepted its producer"
                )
            }
            Self::DisconnectedBeforeSessionEnd => {
                write!(
                    formatter,
                    "live SLAM producer disconnected before session_end"
                )
            }
        }
    }
}

impl Error for LiveSlamInputError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Decode(error) => Some(error),
            Self::Validation(error) => Some(error),
            Self::ProducerAlreadyAccepted => None,
            Self::DisconnectedBeforeSessionEnd => None,
        }
    }
}

impl From<io::Error> for LiveSlamInputError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

pub struct LiveSlamListener {
    listener: UnixListener,
    path: PathBuf,
    socket_identity: (u64, u64),
    producer_accepted: bool,
}

impl LiveSlamListener {
    pub fn bind(path: impl AsRef<Path>) -> Result<Self, LiveSlamInputError> {
        let path = path.as_ref();
        let listener = UnixListener::bind(path)?;
        let metadata = fs::symlink_metadata(path)?;
        Ok(Self {
            listener,
            path: path.to_owned(),
            socket_identity: (metadata.dev(), metadata.ino()),
            producer_accepted: false,
        })
    }

    pub fn accept(&mut self) -> Result<LiveSlamConnection, LiveSlamInputError> {
        if self.producer_accepted {
            return Err(LiveSlamInputError::ProducerAlreadyAccepted);
        }
        let (stream, _) = self.listener.accept()?;
        self.producer_accepted = true;
        Ok(LiveSlamConnection {
            stream,
            validator: BoundarySessionValidator::default(),
            session_ended: false,
        })
    }

    pub fn local_path(&self) -> &Path {
        &self.path
    }
}

impl Drop for LiveSlamListener {
    fn drop(&mut self) {
        let Ok(metadata) = fs::symlink_metadata(&self.path) else {
            return;
        };
        if metadata.file_type().is_socket()
            && (metadata.dev(), metadata.ino()) == self.socket_identity
        {
            let _ = fs::remove_file(&self.path);
        }
    }
}

pub struct LiveSlamConnection {
    stream: UnixStream,
    validator: BoundarySessionValidator,
    session_ended: bool,
}

impl LiveSlamConnection {
    pub fn next_event(&mut self) -> Result<Option<LiveSlamEvent>, LiveSlamInputError> {
        let Some(message) = decode_frame(&mut self.stream).map_err(LiveSlamInputError::Decode)?
        else {
            return if self.session_ended {
                Ok(None)
            } else {
                Err(LiveSlamInputError::DisconnectedBeforeSessionEnd)
            };
        };

        self.validator
            .validate(&message)
            .map_err(LiveSlamInputError::Validation)?;
        let event = adapt_message(message);
        if matches!(event, LiveSlamEvent::SessionEnded { .. }) {
            self.session_ended = true;
        }
        Ok(Some(event))
    }
}

fn adapt_message(message: BoundaryMessage) -> LiveSlamEvent {
    match message {
        BoundaryMessage::Hello {
            session_id,
            producer,
            camera,
            ..
        } => LiveSlamEvent::SessionStarted {
            session_id,
            producer,
            camera: adapt_camera(camera),
        },
        BoundaryMessage::TrackingFrame {
            frame_id,
            timestamp_seconds,
            tracking_state,
            pose,
            ..
        } => {
            let tracking_state = tracking_state.into();
            LiveSlamEvent::TrackingFrame(LiveTrackingFrame {
                frame_id,
                timestamp_seconds,
                tracking_state,
                pose: pose.map(|pose| SlamPose {
                    frame_id,
                    timestamp_seconds,
                    translation: pose.translation,
                    orientation_xyzw: pose.orientation_xyzw,
                    tracking_state,
                }),
            })
        }
        BoundaryMessage::PointcloudDelta {
            frame_id,
            timestamp_seconds,
            add,
            update,
            remove,
            ..
        } => LiveSlamEvent::PointCloudDelta(LivePointCloudDelta {
            frame_id,
            timestamp_seconds,
            add: add.into_iter().map(SlamMapPoint::from).collect(),
            update: update.into_iter().map(SlamMapPoint::from).collect(),
            remove,
        }),
        BoundaryMessage::SessionEnd {
            session_id, reason, ..
        } => LiveSlamEvent::SessionEnded { session_id, reason },
    }
}

fn adapt_camera(camera: BoundaryCamera) -> SlamCameraInfo {
    SlamCameraInfo {
        camera_type: camera.camera_type,
        id: camera.id,
        width: camera.width,
        height: camera.height,
        fps: camera.fps,
    }
}

impl From<BoundaryTrackingState> for SlamTrackingState {
    fn from(state: BoundaryTrackingState) -> Self {
        match state {
            BoundaryTrackingState::Initializing => Self::Initializing,
            BoundaryTrackingState::Tracking => Self::Tracking,
            BoundaryTrackingState::Lost => Self::Lost,
            BoundaryTrackingState::Relocalizing => Self::Relocalizing,
        }
    }
}

impl From<BoundaryMapPoint> for SlamMapPoint {
    fn from(point: BoundaryMapPoint) -> Self {
        Self {
            id: point.id,
            position: point.position,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{
        io::Write,
        os::unix::net::UnixStream,
        path::PathBuf,
        sync::atomic::{AtomicU64, Ordering},
        thread,
    };

    use crate::slam_boundary::BoundaryMessage;

    use super::*;

    static NEXT_SOCKET_ID: AtomicU64 = AtomicU64::new(0);

    fn socket_path(test_name: &str) -> PathBuf {
        std::env::temp_dir().join(format!(
            "slam-remote-{test_name}-{}-{}.sock",
            std::process::id(),
            NEXT_SOCKET_ID.fetch_add(1, Ordering::Relaxed)
        ))
    }

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
        serde_json::from_str(json).expect("fixture should parse")
    }

    fn write_message(stream: &mut UnixStream, message: &BoundaryMessage) {
        let payload = serde_json::to_vec(message).expect("message should serialize");
        stream
            .write_all(&(payload.len() as u32).to_be_bytes())
            .expect("length should write");
        stream.write_all(&payload).expect("payload should write");
    }

    #[test]
    fn receives_and_adapts_a_complete_live_session() {
        let path = socket_path("complete");
        let mut listener = LiveSlamListener::bind(&path).expect("listener should bind");
        let producer_path = path.clone();
        let producer = thread::spawn(move || {
            let mut stream = UnixStream::connect(producer_path).expect("producer should connect");
            for name in ["hello", "tracking_frame", "pointcloud_delta", "session_end"] {
                write_message(&mut stream, &fixture(name));
            }
        });

        let mut connection = listener.accept().expect("consumer should accept");
        assert!(matches!(
            listener.accept(),
            Err(LiveSlamInputError::ProducerAlreadyAccepted)
        ));
        let started = connection
            .next_event()
            .expect("hello should be accepted")
            .expect("hello event should exist");
        let tracking = connection
            .next_event()
            .expect("tracking should be accepted")
            .expect("tracking event should exist");
        let points = connection
            .next_event()
            .expect("points should be accepted")
            .expect("points event should exist");
        let ended = connection
            .next_event()
            .expect("end should be accepted")
            .expect("end event should exist");
        assert_eq!(connection.next_event().unwrap(), None);

        assert!(matches!(
            started,
            LiveSlamEvent::SessionStarted {
                session_id,
                camera: SlamCameraInfo { width: 1280, .. },
                ..
            } if session_id == "fixture-session"
        ));
        assert!(matches!(
            tracking,
            LiveSlamEvent::TrackingFrame(LiveTrackingFrame {
                frame_id: 7,
                tracking_state: SlamTrackingState::Tracking,
                pose: Some(SlamPose {
                    translation: [1.0, 2.0, 3.0],
                    ..
                }),
                ..
            })
        ));
        assert!(matches!(
            points,
            LiveSlamEvent::PointCloudDelta(LivePointCloudDelta { frame_id: 7, add, .. })
                if add.len() == 2 && add[0].id == 1001
        ));
        assert!(matches!(
            ended,
            LiveSlamEvent::SessionEnded { reason, .. } if reason == "shutdown"
        ));

        producer.join().expect("producer should finish");
        drop(connection);
        drop(listener);
        assert!(!path.exists());
    }

    #[test]
    fn reports_disconnect_before_session_end() {
        let path = socket_path("disconnect");
        let mut listener = LiveSlamListener::bind(&path).expect("listener should bind");
        let producer_path = path.clone();
        let producer = thread::spawn(move || {
            let mut stream = UnixStream::connect(producer_path).expect("producer should connect");
            write_message(&mut stream, &fixture("hello"));
        });

        let mut connection = listener.accept().expect("consumer should accept");
        assert!(matches!(
            connection.next_event().unwrap(),
            Some(LiveSlamEvent::SessionStarted { .. })
        ));
        producer.join().expect("producer should finish");
        assert!(matches!(
            connection.next_event(),
            Err(LiveSlamInputError::DisconnectedBeforeSessionEnd)
        ));
    }

    #[test]
    fn reports_truncated_frames() {
        let path = socket_path("truncated");
        let mut listener = LiveSlamListener::bind(&path).expect("listener should bind");
        let producer_path = path.clone();
        let producer = thread::spawn(move || {
            let mut stream = UnixStream::connect(producer_path).expect("producer should connect");
            stream.write_all(&10_u32.to_be_bytes()).unwrap();
            stream.write_all(b"short").unwrap();
        });

        let mut connection = listener.accept().expect("consumer should accept");
        producer.join().expect("producer should finish");
        assert!(matches!(
            connection.next_event(),
            Err(LiveSlamInputError::Decode(BoundaryDecodeError::Io(_)))
        ));
    }

    #[test]
    fn reports_boundary_validation_errors() {
        let path = socket_path("invalid");
        let mut listener = LiveSlamListener::bind(&path).expect("listener should bind");
        let producer_path = path.clone();
        let producer = thread::spawn(move || {
            let mut stream = UnixStream::connect(producer_path).expect("producer should connect");
            write_message(&mut stream, &fixture("unsupported_version"));
        });

        let mut connection = listener.accept().expect("consumer should accept");
        producer.join().expect("producer should finish");
        assert!(matches!(
            connection.next_event(),
            Err(LiveSlamInputError::Validation(
                BoundaryValidationError::UnsupportedVersion(2)
            ))
        ));
    }

    #[test]
    fn refuses_to_replace_an_existing_path() {
        let path = socket_path("existing");
        fs::write(&path, b"owned by another process").expect("fixture file should exist");

        assert!(matches!(
            LiveSlamListener::bind(&path),
            Err(LiveSlamInputError::Io(error)) if error.kind() == io::ErrorKind::AddrInUse
        ));
        assert_eq!(fs::read(&path).unwrap(), b"owned by another process");
        fs::remove_file(path).unwrap();
    }
}
