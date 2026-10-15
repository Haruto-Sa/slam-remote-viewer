use std::{
    collections::BTreeMap,
    error::Error,
    fmt,
    time::{Duration, Instant},
};

use crate::{
    MapPoint, POINTCLOUD_TOPIC, POSE_TOPIC, PointCloudDeltaMessage, PoseMessage, PublishError,
    SETTINGS_TOPIC, SettingsMessage,
    live_slam_input::{LivePointCloudDelta, LiveSlamEvent},
    publish_json,
};

const SETTINGS_INTERVAL: Duration = Duration::from_secs(5);
const MAX_RETAINED_POINTS: usize = 100_000;
const MAX_SNAPSHOT_BYTES: usize = 16 * 1024 * 1024;

#[derive(Debug)]
pub enum LiveProtocolError {
    Publish(PublishError),
    TelemetryBeforeSettings,
    SessionAlreadyStarted,
    MapLimitExceeded,
    SnapshotTooLarge,
}

impl fmt::Display for LiveProtocolError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SessionAlreadyStarted => write!(
                formatter,
                "live publisher already has a session; create a new publisher for each producer"
            ),
            Self::MapLimitExceeded => write!(
                formatter,
                "live map exceeds {MAX_RETAINED_POINTS} retained points"
            ),
            Self::SnapshotTooLarge => write!(
                formatter,
                "complete map exceeds the {MAX_SNAPSHOT_BYTES}-byte protocol payload limit"
            ),
            Self::Publish(error) => error.fmt(formatter),
            Self::TelemetryBeforeSettings => {
                write!(formatter, "live telemetry arrived before session settings")
            }
        }
    }
}

impl Error for LiveProtocolError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Publish(error) => Some(error),
            _ => None,
        }
    }
}

impl From<PublishError> for LiveProtocolError {
    fn from(error: PublishError) -> Self {
        Self::Publish(error)
    }
}

#[derive(Debug, Default)]
pub struct LiveProtocolPublisher {
    settings: Option<SettingsMessage>,
    next_settings: Option<Instant>,
    poses_published: u64,
    pointclouds_published: u64,
    skipped_without_pose: u64,
    snapshots_enabled: bool,
    snapshots_published: u64,
    retained_points: BTreeMap<u64, [f64; 3]>,
    pointcloud_timestamp: f64,
}

impl LiveProtocolPublisher {
    pub fn with_snapshots() -> Self {
        Self {
            snapshots_enabled: true,
            ..Self::default()
        }
    }

    fn updated_map(
        &self,
        delta: &LivePointCloudDelta,
    ) -> Result<BTreeMap<u64, [f64; 3]>, LiveProtocolError> {
        let mut points = self.retained_points.clone();
        for id in &delta.remove {
            points.remove(id);
        }
        for point in delta.update.iter().chain(&delta.add) {
            if !points.contains_key(&point.id) && points.len() == MAX_RETAINED_POINTS {
                return Err(LiveProtocolError::MapLimitExceeded);
            }
            points.insert(point.id, point.position);
        }
        Ok(points)
    }

    fn publish_snapshot(&mut self, socket: &zmq::Socket) -> Result<(), LiveProtocolError> {
        let settings = self
            .settings
            .as_ref()
            .ok_or(LiveProtocolError::TelemetryBeforeSettings)?;
        let snapshot = PointCloudDeltaMessage {
            v: crate::PROTOCOL_VERSION,
            session: settings.session.clone(),
            seq: self.pointclouds_published,
            t: self.pointcloud_timestamp,
            snapshot: true,
            add: self
                .retained_points
                .iter()
                .map(|(&id, p)| MapPoint(id, p[0], p[1], p[2]))
                .collect(),
            update: Vec::new(),
            remove: Vec::new(),
        };
        let payload = serde_json::to_vec(&snapshot).map_err(PublishError::from)?;
        if payload.len() > MAX_SNAPSHOT_BYTES {
            return Err(LiveProtocolError::SnapshotTooLarge);
        }
        socket
            .send_multipart([POINTCLOUD_TOPIC.as_bytes(), payload.as_slice()], 0)
            .map_err(PublishError::from)?;
        self.pointclouds_published += 1;
        self.snapshots_published += 1;
        Ok(())
    }

    /// Repeats settings and, when enabled, the complete retained map. Call on
    /// the live event loop's idle timeout as well as before processing events.
    pub fn publish_due_settings(
        &mut self,
        socket: &zmq::Socket,
        now: Instant,
    ) -> Result<(), LiveProtocolError> {
        if self.next_settings.is_some_and(|deadline| now >= deadline) {
            let settings = self
                .settings
                .as_ref()
                .ok_or(LiveProtocolError::TelemetryBeforeSettings)?;
            publish_json(socket, SETTINGS_TOPIC, settings)?;
            if self.snapshots_enabled {
                self.publish_snapshot(socket)?;
            }
            self.next_settings = Some(now + SETTINGS_INTERVAL);
        }
        Ok(())
    }

    pub fn handle_event(
        &mut self,
        socket: &zmq::Socket,
        event: LiveSlamEvent,
        now: Instant,
    ) -> Result<bool, LiveProtocolError> {
        match event {
            LiveSlamEvent::SessionStarted {
                session_id,
                producer: _,
                camera,
            } => {
                if self.settings.is_some() {
                    return Err(LiveProtocolError::SessionAlreadyStarted);
                }
                let mut settings = SettingsMessage::live(
                    session_id,
                    camera.camera_type,
                    camera.id,
                    camera.width,
                    camera.height,
                    camera.fps,
                );
                if self.snapshots_enabled {
                    settings.pointcloud_mode = "delta_snapshot".into();
                }
                publish_json(socket, SETTINGS_TOPIC, &settings)?;
                self.settings = Some(settings);
                self.next_settings = Some(now + SETTINGS_INTERVAL);
            }
            LiveSlamEvent::TrackingFrame(frame) => {
                self.publish_due_settings(socket, now)?;
                let settings = self
                    .settings
                    .as_ref()
                    .ok_or(LiveProtocolError::TelemetryBeforeSettings)?;
                if let Some(pose) = frame.pose {
                    let message = PoseMessage::from_slam_pose(settings.session.as_str(), pose);
                    publish_json(socket, POSE_TOPIC, &message)?;
                    self.poses_published += 1;
                } else {
                    self.skipped_without_pose += 1;
                }
            }
            LiveSlamEvent::PointCloudDelta(delta) => {
                self.publish_due_settings(socket, now)?;
                let updated = if self.snapshots_enabled {
                    Some(self.updated_map(&delta)?)
                } else {
                    None
                };
                let timestamp = delta.timestamp_seconds;
                let settings = self
                    .settings
                    .as_ref()
                    .ok_or(LiveProtocolError::TelemetryBeforeSettings)?;
                let message = PointCloudDeltaMessage::from_live_delta(
                    settings.session.as_str(),
                    self.pointclouds_published,
                    delta,
                );
                publish_json(socket, POINTCLOUD_TOPIC, &message)?;
                if let Some(points) = updated {
                    self.retained_points = points;
                    self.pointcloud_timestamp = timestamp;
                }
                self.pointclouds_published += 1;
            }
            LiveSlamEvent::SessionEnded { .. } => return Ok(true),
        }
        Ok(false)
    }

    pub fn poses_published(&self) -> u64 {
        self.poses_published
    }

    pub fn skipped_without_pose(&self) -> u64 {
        self.skipped_without_pose
    }

    pub fn pointclouds_published(&self) -> u64 {
        self.pointclouds_published
    }

    pub fn snapshots_published(&self) -> u64 {
        self.snapshots_published
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        live_slam_input::{LivePointCloudDelta, LiveTrackingFrame, SlamCameraInfo, SlamMapPoint},
        pose_source::{SlamPose, SlamTrackingState},
    };

    fn session_started() -> LiveSlamEvent {
        LiveSlamEvent::SessionStarted {
            session_id: "live-session".to_owned(),
            producer: "fake-orbslam3".to_owned(),
            camera: SlamCameraInfo {
                camera_type: "monocular".to_owned(),
                id: "camera-1".to_owned(),
                width: 640,
                height: 480,
                fps: 30,
            },
        }
    }

    fn cloud(
        add: Vec<SlamMapPoint>,
        update: Vec<SlamMapPoint>,
        remove: Vec<u64>,
    ) -> LivePointCloudDelta {
        LivePointCloudDelta {
            frame_id: 1,
            timestamp_seconds: 2.5,
            add,
            update,
            remove,
        }
    }

    #[test]
    fn repeats_complete_sorted_maps_and_empty_maps_on_idle_ticks() {
        let context = zmq::Context::new();
        let publisher = context.socket(zmq::PAIR).unwrap();
        let subscriber = context.socket(zmq::PAIR).unwrap();
        publisher.bind("inproc://snapshot-unit").unwrap();
        subscriber.connect("inproc://snapshot-unit").unwrap();
        subscriber.set_rcvtimeo(1000).unwrap();
        let receive = || {
            let frames = subscriber.recv_multipart(0).unwrap();
            (
                String::from_utf8(frames[0].clone()).unwrap(),
                serde_json::from_slice::<serde_json::Value>(&frames[1]).unwrap(),
            )
        };
        let now = Instant::now();
        let mut adapter = LiveProtocolPublisher::with_snapshots();
        adapter
            .handle_event(&publisher, session_started(), now)
            .unwrap();
        assert_eq!(receive().1["pointcloud_mode"], "delta_snapshot");
        adapter
            .handle_event(
                &publisher,
                LiveSlamEvent::PointCloudDelta(cloud(
                    vec![
                        SlamMapPoint {
                            id: 9,
                            position: [9.0, 0.0, 0.0],
                        },
                        SlamMapPoint {
                            id: 2,
                            position: [2.0, 0.0, 0.0],
                        },
                    ],
                    vec![],
                    vec![],
                )),
                now,
            )
            .unwrap();
        assert_eq!(receive().1["seq"], 0);
        adapter
            .publish_due_settings(&publisher, now + SETTINGS_INTERVAL)
            .unwrap();
        assert_eq!(receive().0, SETTINGS_TOPIC);
        let snapshot = receive().1;
        assert_eq!(snapshot["snapshot"], true);
        assert_eq!(snapshot["seq"], 1);
        assert_eq!(snapshot["t"], 2.5);
        assert_eq!(
            snapshot["add"],
            serde_json::json!([[2, 2.0, 0.0, 0.0], [9, 9.0, 0.0, 0.0]])
        );
        adapter
            .handle_event(
                &publisher,
                LiveSlamEvent::PointCloudDelta(cloud(vec![], vec![], vec![2, 9])),
                now + SETTINGS_INTERVAL,
            )
            .unwrap();
        assert_eq!(receive().1["seq"], 2);
        adapter
            .publish_due_settings(&publisher, now + 2 * SETTINGS_INTERVAL)
            .unwrap();
        assert_eq!(receive().0, SETTINGS_TOPIC);
        let empty = receive().1;
        assert_eq!(empty["seq"], 3);
        assert_eq!(empty["add"], serde_json::json!([]));
        assert_eq!(empty["snapshot"], true);
        assert_eq!(adapter.snapshots_published(), 2);
        assert!(matches!(
            adapter.handle_event(&publisher, session_started(), now),
            Err(LiveProtocolError::SessionAlreadyStarted)
        ));
    }

    #[test]
    fn rejects_map_overflow_without_changing_retained_state() {
        let mut adapter = LiveProtocolPublisher::with_snapshots();
        adapter.retained_points = (0..MAX_RETAINED_POINTS as u64)
            .map(|id| (id, [0.0; 3]))
            .collect();
        let overflow = cloud(
            vec![SlamMapPoint {
                id: MAX_RETAINED_POINTS as u64,
                position: [1.0; 3],
            }],
            vec![],
            vec![],
        );
        assert!(matches!(
            adapter.updated_map(&overflow),
            Err(LiveProtocolError::MapLimitExceeded)
        ));
        assert_eq!(adapter.retained_points.len(), MAX_RETAINED_POINTS);
        assert!(
            !adapter
                .retained_points
                .contains_key(&(MAX_RETAINED_POINTS as u64))
        );
        let replacement = cloud(overflow.add, vec![], vec![0]);
        let next = adapter.updated_map(&replacement).unwrap();
        assert_eq!(next.len(), MAX_RETAINED_POINTS);
        assert!(!next.contains_key(&0));
        assert!(adapter.retained_points.contains_key(&0));
    }

    #[test]
    fn separate_publishers_keep_overlapping_point_ids_in_separate_sessions() {
        let context = zmq::Context::new();
        let socket = context.socket(zmq::PAIR).unwrap();
        let subscriber = context.socket(zmq::PAIR).unwrap();
        socket.bind("inproc://independent-camera-maps").unwrap();
        subscriber
            .connect("inproc://independent-camera-maps")
            .unwrap();
        subscriber.set_rcvtimeo(1000).unwrap();
        let now = Instant::now();
        let mut cameras = [
            LiveProtocolPublisher::with_snapshots(),
            LiveProtocolPublisher::with_snapshots(),
        ];
        for (index, camera) in cameras.iter_mut().enumerate() {
            let mut hello = session_started();
            if let LiveSlamEvent::SessionStarted {
                session_id, camera, ..
            } = &mut hello
            {
                *session_id = format!("session-{index}");
                camera.id = format!("camera-{index}");
            }
            camera.handle_event(&socket, hello, now).unwrap();
            subscriber.recv_multipart(0).unwrap();
            camera
                .handle_event(
                    &socket,
                    LiveSlamEvent::PointCloudDelta(cloud(
                        vec![SlamMapPoint {
                            id: 1,
                            position: [index as f64, 0.0, 0.0],
                        }],
                        vec![],
                        vec![],
                    )),
                    now,
                )
                .unwrap();
            subscriber.recv_multipart(0).unwrap();
        }
        for (index, camera) in cameras.iter_mut().enumerate() {
            camera
                .publish_due_settings(&socket, now + SETTINGS_INTERVAL)
                .unwrap();
            let settings = subscriber.recv_multipart(0).unwrap();
            let settings: serde_json::Value = serde_json::from_slice(&settings[1]).unwrap();
            assert_eq!(settings["camera"]["id"], format!("camera-{index}"));
            let frames = subscriber.recv_multipart(0).unwrap();
            let snapshot: serde_json::Value = serde_json::from_slice(&frames[1]).unwrap();
            assert_eq!(snapshot["session"], format!("session-{index}"));
            assert_eq!(
                snapshot["add"],
                serde_json::json!([[1, index as f64, 0.0, 0.0]])
            );
        }
    }

    #[test]
    fn refuses_oversized_snapshot_payload_without_publishing_partial_map() {
        let context = zmq::Context::new();
        let publisher = context.socket(zmq::PUB).unwrap();
        let mut adapter = LiveProtocolPublisher::with_snapshots();
        adapter.settings = Some(SettingsMessage::mock("s".repeat(MAX_SNAPSHOT_BYTES)));
        assert!(matches!(
            adapter.publish_snapshot(&publisher),
            Err(LiveProtocolError::SnapshotTooLarge)
        ));
        assert_eq!(adapter.pointclouds_published(), 0);
    }

    #[test]
    fn publishes_settings_before_fake_backend_pose() {
        let context = zmq::Context::new();
        let publisher = context.socket(zmq::PAIR).expect("publisher socket");
        let subscriber = context.socket(zmq::PAIR).expect("subscriber socket");
        publisher
            .bind("inproc://live-protocol-order")
            .expect("bind");
        subscriber
            .connect("inproc://live-protocol-order")
            .expect("connect");
        subscriber.set_rcvtimeo(1_000).expect("receive timeout");

        let now = Instant::now();
        let mut adapter = LiveProtocolPublisher::default();
        adapter
            .handle_event(
                &publisher,
                LiveSlamEvent::SessionStarted {
                    session_id: "live-session".to_owned(),
                    producer: "fake-orbslam3".to_owned(),
                    camera: SlamCameraInfo {
                        camera_type: "monocular".to_owned(),
                        id: "camera-1".to_owned(),
                        width: 640,
                        height: 480,
                        fps: 30,
                    },
                },
                now,
            )
            .expect("settings event");
        let pose = SlamPose {
            frame_id: 7,
            timestamp_seconds: 0.25,
            translation: [1.0, 2.0, 3.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            tracking_state: SlamTrackingState::Tracking,
        };
        adapter
            .handle_event(
                &publisher,
                LiveSlamEvent::TrackingFrame(LiveTrackingFrame {
                    frame_id: pose.frame_id,
                    timestamp_seconds: pose.timestamp_seconds,
                    tracking_state: pose.tracking_state,
                    pose: Some(pose),
                }),
                now,
            )
            .expect("pose event");

        let settings_frames = subscriber.recv_multipart(0).expect("settings packet");
        let pose_frames = subscriber.recv_multipart(0).expect("pose packet");
        assert_eq!(settings_frames[0], SETTINGS_TOPIC.as_bytes());
        assert_eq!(pose_frames[0], POSE_TOPIC.as_bytes());
        let settings: serde_json::Value =
            serde_json::from_slice(&settings_frames[1]).expect("settings JSON");
        let pose: serde_json::Value = serde_json::from_slice(&pose_frames[1]).expect("pose JSON");
        assert_eq!(settings["frame"], "slam_world");
        assert_eq!(settings["pose_convention"], "Twc");
        assert_eq!(pose["session"], "live-session");
        assert_eq!(pose["seq"], 7);
        assert_eq!(adapter.poses_published(), 1);
    }

    #[test]
    fn skips_pose_less_tracking_without_fabricating_coordinates() {
        let context = zmq::Context::new();
        let publisher = context.socket(zmq::PAIR).expect("publisher socket");
        let subscriber = context.socket(zmq::PAIR).expect("subscriber socket");
        publisher.bind("inproc://live-protocol-lost").expect("bind");
        subscriber
            .connect("inproc://live-protocol-lost")
            .expect("connect");
        subscriber.set_rcvtimeo(20).expect("receive timeout");
        let mut adapter = LiveProtocolPublisher::default();
        let now = Instant::now();
        adapter
            .handle_event(
                &publisher,
                LiveSlamEvent::SessionStarted {
                    session_id: "session".to_owned(),
                    producer: "fake".to_owned(),
                    camera: SlamCameraInfo {
                        camera_type: "monocular".to_owned(),
                        id: "camera".to_owned(),
                        width: 640,
                        height: 480,
                        fps: 30,
                    },
                },
                now,
            )
            .expect("settings event");
        subscriber.recv_multipart(0).expect("discard settings");
        adapter
            .handle_event(
                &publisher,
                LiveSlamEvent::TrackingFrame(LiveTrackingFrame {
                    frame_id: 8,
                    timestamp_seconds: 0.5,
                    tracking_state: SlamTrackingState::Lost,
                    pose: None,
                }),
                now,
            )
            .expect("lost event");
        assert_eq!(subscriber.recv_multipart(0), Err(zmq::Error::EAGAIN));
        assert_eq!(adapter.skipped_without_pose(), 1);
    }

    #[test]
    fn republishes_settings_without_a_tracking_event() {
        let context = zmq::Context::new();
        let publisher = context.socket(zmq::PAIR).expect("publisher socket");
        let subscriber = context.socket(zmq::PAIR).expect("subscriber socket");
        publisher.bind("inproc://live-protocol-tick").expect("bind");
        subscriber
            .connect("inproc://live-protocol-tick")
            .expect("connect");
        subscriber.set_rcvtimeo(1_000).expect("receive timeout");
        let mut adapter = LiveProtocolPublisher::default();
        let now = Instant::now();
        adapter
            .handle_event(
                &publisher,
                LiveSlamEvent::SessionStarted {
                    session_id: "session".to_owned(),
                    producer: "fake".to_owned(),
                    camera: SlamCameraInfo {
                        camera_type: "monocular".to_owned(),
                        id: "camera".to_owned(),
                        width: 640,
                        height: 480,
                        fps: 30,
                    },
                },
                now,
            )
            .expect("settings event");
        subscriber.recv_multipart(0).expect("startup settings");

        adapter
            .publish_due_settings(&publisher, now + SETTINGS_INTERVAL)
            .expect("periodic settings");
        let frames = subscriber.recv_multipart(0).expect("periodic packet");
        assert_eq!(frames[0], SETTINGS_TOPIC.as_bytes());
    }

    #[test]
    fn publishes_live_pointcloud_with_topic_sequence() {
        let context = zmq::Context::new();
        let publisher = context.socket(zmq::PAIR).expect("publisher socket");
        let subscriber = context.socket(zmq::PAIR).expect("subscriber socket");
        publisher
            .bind("inproc://live-protocol-pointcloud")
            .expect("bind");
        subscriber
            .connect("inproc://live-protocol-pointcloud")
            .expect("connect");
        subscriber.set_rcvtimeo(1_000).expect("receive timeout");

        let now = Instant::now();
        let mut adapter = LiveProtocolPublisher::default();
        adapter
            .handle_event(&publisher, session_started(), now)
            .expect("settings event");
        subscriber.recv_multipart(0).expect("startup settings");

        for frame_id in [42, 72] {
            adapter
                .handle_event(
                    &publisher,
                    LiveSlamEvent::PointCloudDelta(LivePointCloudDelta {
                        frame_id,
                        timestamp_seconds: frame_id as f64 / 30.0,
                        add: vec![SlamMapPoint {
                            id: 1001 + frame_id,
                            position: [0.1, 0.2, 1.4],
                        }],
                        update: Vec::new(),
                        remove: vec![1000 + frame_id],
                    }),
                    now,
                )
                .expect("point-cloud event");
        }

        for expected_seq in [0, 1] {
            let frames = subscriber.recv_multipart(0).expect("point-cloud packet");
            assert_eq!(frames[0], POINTCLOUD_TOPIC.as_bytes());
            let payload: serde_json::Value =
                serde_json::from_slice(&frames[1]).expect("point-cloud JSON");
            assert_eq!(payload["session"], "live-session");
            assert_eq!(payload["seq"], expected_seq);
            assert_eq!(payload["add"][0][1], 0.1);
        }
        assert_eq!(adapter.pointclouds_published(), 2);
    }

    #[test]
    fn rejects_pointcloud_before_settings() {
        let context = zmq::Context::new();
        let publisher = context.socket(zmq::PAIR).expect("publisher socket");
        let mut adapter = LiveProtocolPublisher::default();
        let error = adapter
            .handle_event(
                &publisher,
                LiveSlamEvent::PointCloudDelta(LivePointCloudDelta {
                    frame_id: 1,
                    timestamp_seconds: 0.1,
                    add: Vec::new(),
                    update: Vec::new(),
                    remove: Vec::new(),
                }),
                Instant::now(),
            )
            .expect_err("point-cloud before settings must fail");
        assert!(matches!(error, LiveProtocolError::TelemetryBeforeSettings));
        assert_eq!(adapter.pointclouds_published(), 0);
    }
}
