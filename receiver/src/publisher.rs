use std::{error::Error, fmt, time::Instant};

use crate::protocol::{SETTINGS_TOPIC, TelemetryMessage};

#[derive(Debug)]
pub enum EncodeError {
    Serialization(serde_json::Error),
}

impl fmt::Display for EncodeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Serialization(error) => {
                write!(formatter, "failed to serialize telemetry: {error}")
            }
        }
    }
}

impl Error for EncodeError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Serialization(error) => Some(error),
        }
    }
}

impl From<serde_json::Error> for EncodeError {
    fn from(error: serde_json::Error) -> Self {
        Self::Serialization(error)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EncodedTelemetry {
    topic: &'static str,
    session: String,
    seq: Option<u64>,
    payload: Vec<u8>,
}

impl EncodedTelemetry {
    pub fn topic(&self) -> &'static str {
        self.topic
    }

    pub fn session(&self) -> &str {
        &self.session
    }

    pub fn seq(&self) -> Option<u64> {
        self.seq
    }

    pub fn payload(&self) -> &[u8] {
        &self.payload
    }
}

pub fn encode_telemetry(message: &TelemetryMessage) -> Result<EncodedTelemetry, EncodeError> {
    let payload = match message {
        TelemetryMessage::Settings(settings) => serde_json::to_vec(settings)?,
        TelemetryMessage::Pose(pose) => serde_json::to_vec(pose)?,
        TelemetryMessage::PointCloud(pointcloud) => serde_json::to_vec(pointcloud)?,
    };

    Ok(EncodedTelemetry {
        topic: message.topic(),
        session: message.session().to_owned(),
        seq: message.seq(),
        payload,
    })
}

pub fn publish_encoded(
    socket: &zmq::Socket,
    telemetry: &EncodedTelemetry,
) -> Result<(), zmq::Error> {
    socket.send_multipart([telemetry.topic().as_bytes(), telemetry.payload()], 0)
}

#[derive(Debug)]
pub struct SettingsRepeater {
    interval: std::time::Duration,
    latest: Option<EncodedTelemetry>,
    next_repeat: Option<Instant>,
}

impl SettingsRepeater {
    pub fn new(interval: std::time::Duration) -> Self {
        Self {
            interval,
            latest: None,
            next_repeat: None,
        }
    }

    pub fn remember(&mut self, telemetry: &EncodedTelemetry, now: Instant) {
        if telemetry.topic() != SETTINGS_TOPIC {
            return;
        }

        self.latest = Some(telemetry.clone());
        self.next_repeat = Some(now + self.interval);
    }

    pub fn take_due(&mut self, now: Instant) -> Option<EncodedTelemetry> {
        if now < self.next_repeat? {
            return None;
        }

        self.next_repeat = Some(now + self.interval);
        self.latest.clone()
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use crate::{
        coordinates::telemetry_to_unity,
        protocol::{POINTCLOUD_TOPIC, POSE_TOPIC, parse_telemetry},
    };

    const SETTINGS_EXAMPLE: &str = include_str!("../../protocol/settings.example.json");
    const POSE_EXAMPLE: &str = include_str!("../../protocol/pose.example.json");
    const POINTCLOUD_EXAMPLE: &str = include_str!("../../protocol/pointcloud.example.json");

    #[test]
    fn encodes_concrete_message_without_enum_wrapper() {
        let mut message = parse_telemetry(SETTINGS_TOPIC, SETTINGS_EXAMPLE)
            .expect("settings example should parse");
        telemetry_to_unity(&mut message);

        let encoded = encode_telemetry(&message).expect("settings should encode");
        let value: serde_json::Value =
            serde_json::from_slice(encoded.payload()).expect("payload should be JSON");

        assert_eq!(encoded.topic(), SETTINGS_TOPIC);
        assert_eq!(encoded.session(), "001");
        assert_eq!(encoded.seq(), None);
        assert_eq!(value["frame"], "unity_world");
        assert_eq!(value["session"], "001");
        assert!(value.get("Settings").is_none());
    }

    #[test]
    fn encodes_all_protocol_topics() {
        let cases = [
            (SETTINGS_TOPIC, SETTINGS_EXAMPLE),
            (POSE_TOPIC, POSE_EXAMPLE),
            (POINTCLOUD_TOPIC, POINTCLOUD_EXAMPLE),
        ];

        for (topic, payload) in cases {
            let message = parse_telemetry(topic, payload).expect("example should parse");
            let encoded = encode_telemetry(&message).expect("message should encode");

            assert_eq!(encoded.topic(), topic);
            assert!(!encoded.payload().is_empty());
        }
    }

    #[test]
    fn publishes_two_multipart_frames() {
        let context = zmq::Context::new();
        let sender = context
            .socket(zmq::PAIR)
            .expect("sender socket should open");
        let receiver = context
            .socket(zmq::PAIR)
            .expect("receiver socket should open");
        let endpoint = "inproc://receiver-publisher-test";
        sender.bind(endpoint).expect("sender should bind");
        receiver.connect(endpoint).expect("receiver should connect");

        let message = parse_telemetry(POSE_TOPIC, POSE_EXAMPLE).expect("pose example should parse");
        let encoded = encode_telemetry(&message).expect("pose should encode");

        publish_encoded(&sender, &encoded).expect("telemetry should publish");
        let frames = receiver
            .recv_multipart(0)
            .expect("telemetry should be received");

        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], POSE_TOPIC.as_bytes());
        assert_eq!(frames[1], encoded.payload());
    }

    #[test]
    fn repeats_only_latest_settings_after_interval() {
        let start = Instant::now();
        let interval = Duration::from_secs(5);
        let mut repeater = SettingsRepeater::new(interval);

        let first = parse_telemetry(SETTINGS_TOPIC, SETTINGS_EXAMPLE)
            .expect("settings example should parse");
        let first = encode_telemetry(&first).expect("settings should encode");
        repeater.remember(&first, start);

        assert!(
            repeater
                .take_due(start + interval - Duration::from_millis(1))
                .is_none()
        );
        assert_eq!(repeater.take_due(start + interval), Some(first.clone()));
        assert!(repeater.take_due(start + interval).is_none());

        let mut second = parse_telemetry(SETTINGS_TOPIC, SETTINGS_EXAMPLE)
            .expect("settings example should parse");
        let TelemetryMessage::Settings(settings) = &mut second else {
            panic!("message should be settings");
        };
        settings.session = "session-2".to_owned();
        let second = encode_telemetry(&second).expect("settings should encode");
        repeater.remember(&second, start + interval);

        assert_eq!(repeater.take_due(start + interval * 2), Some(second));
    }

    #[test]
    fn ignores_non_settings_for_repetition() {
        let start = Instant::now();
        let interval = Duration::from_secs(5);
        let mut repeater = SettingsRepeater::new(interval);
        let pose = parse_telemetry(POSE_TOPIC, POSE_EXAMPLE).expect("pose example should parse");
        let pose = encode_telemetry(&pose).expect("pose should encode");

        repeater.remember(&pose, start);

        assert!(repeater.take_due(start + interval).is_none());
    }
}
