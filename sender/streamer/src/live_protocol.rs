use std::{
    error::Error,
    fmt,
    time::{Duration, Instant},
};

use crate::{
    POINTCLOUD_TOPIC, POSE_TOPIC, PointCloudDeltaMessage, PoseMessage, PublishError,
    SETTINGS_TOPIC, SettingsMessage, live_slam_input::LiveSlamEvent, publish_json,
};

const SETTINGS_INTERVAL: Duration = Duration::from_secs(5);

#[derive(Debug)]
pub enum LiveProtocolError {
    Publish(PublishError),
    TelemetryBeforeSettings,
}

impl fmt::Display for LiveProtocolError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
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
            Self::TelemetryBeforeSettings => None,
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
}

impl LiveProtocolPublisher {
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
                let settings = SettingsMessage::live(
                    session_id,
                    camera.camera_type,
                    camera.id,
                    camera.width,
                    camera.height,
                    camera.fps,
                );
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
