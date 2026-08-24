use std::{error::Error, fmt};

use serde::Serialize;

pub const PROTOCOL_VERSION: u32 = 1;
pub const SETTINGS_TOPIC: &str = "slam/v1/settings";
pub const POSE_TOPIC: &str = "slam/v1/pose";
pub const POINTCLOUD_TOPIC: &str = "slam/v1/pointcloud";

#[derive(Debug)]
pub enum PublishError {
    Serialization(serde_json::Error),
    Transport(zmq::Error),
}

impl fmt::Display for PublishError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Serialization(error) => {
                write!(formatter, "failed to serialize message: {error}")
            }
            Self::Transport(error) => {
                write!(formatter, "failed to publish message: {error}")
            }
        }
    }
}
impl Error for PublishError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Serialization(error) => Some(error),
            Self::Transport(error) => Some(error),
        }
    }
}

impl From<serde_json::Error> for PublishError {
    fn from(error: serde_json::Error) -> Self {
        Self::Serialization(error)
    }
}

impl From<zmq::Error> for PublishError {
    fn from(error: zmq::Error) -> Self {
        Self::Transport(error)
    }
}

pub fn publish_json<T>(socket: &zmq::Socket, topic: &str, message: &T) -> Result<(), PublishError>
where
    T: Serialize,
{
    let payload = serde_json::to_vec(message)?;

    socket.send_multipart([topic.as_bytes(), payload.as_slice()], 0)?;

    Ok(())
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CameraSettings {
    #[serde(rename = "type")]
    pub camera_type: String,
    pub id: String,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
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

impl SettingsMessage {
    pub fn mock(session: impl Into<String>) -> Self {
        Self {
            v: PROTOCOL_VERSION,
            session: session.into(),
            unit: "m".to_owned(),
            frame: "slam_world".to_owned(),
            pose_convention: "Twc".to_owned(),
            quaternion: "xyzw".to_owned(),
            camera: CameraSettings {
                camera_type: "mock".to_owned(),
                id: "mock_camera".to_owned(),
                width: 1280,
                height: 720,
                fps: 30,
            },
            pointcloud_mode: "delta".to_owned(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum TrackingState {
    Initializing,
    Tracking,
    Lost,
    Relocalizing,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct PoseMessage {
    pub v: u32,
    pub session: String,
    pub seq: u64,
    pub t: f64,
    pub p: [f64; 3],
    pub q: [f64; 4],
    pub state: TrackingState,
}

impl PoseMessage {
    pub fn circular(
        session: impl Into<String>,
        seq: u64,
        time_sec: f64,
        radius_m: f64,
        angular_speed_rad_per_sec: f64,
    ) -> Self {
        let angle = angular_speed_rad_per_sec * time_sec;
        let half_yaw = -0.5 * angle;

        let quaternion_y = if angle == 0.0 { 0.0 } else { half_yaw.sin() };

        Self {
            v: PROTOCOL_VERSION,
            session: session.into(),
            seq,
            t: time_sec,
            p: [radius_m * angle.cos(), 0.0, radius_m * angle.sin()],
            q: [0.0, quaternion_y, 0.0, half_yaw.cos()],
            state: TrackingState::Tracking,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MapPoint(pub u64, pub f64, pub f64, pub f64);

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct PointCloudDeltaMessage {
    pub v: u32,
    pub session: String,
    pub seq: u64,
    pub t: f64,
    pub add: Vec<MapPoint>,
    pub update: Vec<MapPoint>,
    pub remove: Vec<u64>,
}

impl PointCloudDeltaMessage {
    pub fn fixture(session: impl Into<String>, seq: u64, time_sec: f64) -> Self {
        Self {
            v: PROTOCOL_VERSION,
            session: session.into(),
            seq,
            t: time_sec,
            add: vec![MapPoint(1001, 0.1, 0.2, 1.4), MapPoint(1002, 0.2, 0.3, 1.5)],
            update: Vec::new(),
            remove: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn serializes_protocol_v1_settings() {
        let message = SettingsMessage::mock("test-session");

        let actual = serde_json::to_value(message).expect("settings must serialize");

        let expected = json!({
            "v": 1,
            "session": "test-session",
            "unit": "m",
            "frame": "slam_world",
            "pose_convention": "Twc",
            "quaternion": "xyzw",
            "camera": {
                "type": "mock",
                "id": "mock_camera",
                "width": 1280,
                "height": 720,
                "fps": 30
            },
            "pointcloud_mode": "delta"
        });

        assert_eq!(actual, expected);
    }

    fn assert_approx_eq(actual: f64, expected: f64) {
        const EPSILON: f64 = 1.0e-12;

        assert!(
            (actual - expected).abs() < EPSILON,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn uses_protocol_v1_settings_topic() {
        assert_eq!(SETTINGS_TOPIC, "slam/v1/settings");
    }

    #[test]
    fn uses_protocol_v1_pose_topic() {
        assert_eq!(POSE_TOPIC, "slam/v1/pose");
    }

    #[test]
    fn serializes_initial_circular_pose() {
        let message = PoseMessage::circular("test-session", 42, 0.0, 2.0, 1.0);

        let actual = serde_json::to_value(message).expect("pose must serialize");

        let expected = json!({
            "v": 1,
            "session": "test-session",
            "seq": 42,
            "t": 0.0,
            "p": [2.0, 0.0, 0.0],
            "q": [0.0, 0.0, 0.0, 1.0],
            "state": "tracking"
        });

        assert_eq!(actual, expected);
    }

    #[test]
    fn generate_quarter_turn_pose() {
        use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};

        let pose = PoseMessage::circular("test-session", 7, FRAC_PI_2, 2.0, 1.0);

        assert_eq!(pose.seq, 7);
        assert_eq!(pose.state, TrackingState::Tracking);

        assert_approx_eq(pose.p[0], 0.0);
        assert_approx_eq(pose.p[1], 0.0);
        assert_approx_eq(pose.p[2], 2.0);

        assert_approx_eq(pose.q[0], 0.0);
        assert_approx_eq(pose.q[1], -FRAC_1_SQRT_2);
        assert_approx_eq(pose.q[2], 0.0);
        assert_approx_eq(pose.q[3], FRAC_1_SQRT_2);

        let quaternion_length = pose.q.iter().map(|value| value * value).sum::<f64>().sqrt();

        assert_approx_eq(quaternion_length, 1.0);
    }

    #[test]
    fn serialize_pointcloud_delta_fixture() {
        let message = PointCloudDeltaMessage::fixture("test-session", 82, 123.5);

        let actual = serde_json::to_value(message).expect("pointcloud must serialize");

        let expected = json! ({
            "v": 1,
            "session": "test-session",
            "seq": 82,
            "t": 123.5,
            "add": [
                [1001, 0.1, 0.2, 1.4],
                [1002, 0.2, 0.3, 1.5]
            ],
            "update": [],
            "remove": []
        });

        assert_eq!(actual, expected);
    }

    #[test]
    fn uses_protocol_v1_pointcloud_topic() {
        assert_eq!(POINTCLOUD_TOPIC, "slam/v1/pointcloud");
    }

    #[test]
    fn publishes_topic_and_json_as_two_frames() {
        let context = zmq::Context::new();

        let receiver = context
            .socket(zmq::PAIR)
            .expect("receiver socket must be created");

        receiver
            .bind("inproc://publish-json-test")
            .expect("receiver must bind");

        let sender = context
            .socket(zmq::PAIR)
            .expect("sender socket must be created");

        sender
            .connect("inproc://publish-json-test")
            .expect("sender must connect");

        let message = SettingsMessage::mock("test-session");

        publish_json(&sender, SETTINGS_TOPIC, &message).expect("message must publish");

        let frames = receiver
            .recv_multipart(0)
            .expect("message must be received");

        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], SETTINGS_TOPIC.as_bytes());

        let payload: serde_json::Value =
            serde_json::from_slice(&frames[1]).expect("payload must be JSON");

        assert_eq!(payload["session"], "test-session");
        assert_eq!(payload["frame"], "slam_world");
    }
}
