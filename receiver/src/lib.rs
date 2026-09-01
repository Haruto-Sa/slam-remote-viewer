pub mod clip;
pub mod control;
pub mod coordinates;
pub mod playback;
pub mod protocol;
pub mod publisher;
pub mod quaternion;
pub mod recording;

use std::fmt;

pub const PROTOCOL_V1_TOPIC_PREFIX: &str = "slam/v1/";
pub const MAX_PAYLOAD_BYTES: usize = 16 * 1024 * 1024;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TelemetryPacket {
    pub topic: String,
    pub payload: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecodeError {
    InvalidFrameCount { actual: usize },
    TopicNotUtf8,
    PayloadNotUtf8,
    PayloadTooLarge { actual: usize, max: usize },
}

impl fmt::Display for DecodeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidFrameCount { actual } => {
                write!(formatter, "expected 2 frames, received {actual}")
            }
            Self::TopicNotUtf8 => write!(formatter, "topic frame is not valid UTF-8"),
            Self::PayloadNotUtf8 => write!(formatter, "payload frame is not valid UTF-8"),
            Self::PayloadTooLarge { actual, max } => {
                write!(
                    formatter,
                    "payload contains {actual} bytes, maximum is {max}"
                )
            }
        }
    }
}

impl std::error::Error for DecodeError {}

pub fn decode_multipart(frames: &[Vec<u8>]) -> Result<TelemetryPacket, DecodeError> {
    if frames.len() != 2 {
        return Err(DecodeError::InvalidFrameCount {
            actual: frames.len(),
        });
    }

    if frames[1].len() > MAX_PAYLOAD_BYTES {
        return Err(DecodeError::PayloadTooLarge {
            actual: frames[1].len(),
            max: MAX_PAYLOAD_BYTES,
        });
    }

    let topic = std::str::from_utf8(&frames[0]).map_err(|_| DecodeError::TopicNotUtf8)?;
    let payload = std::str::from_utf8(&frames[1]).map_err(|_| DecodeError::PayloadNotUtf8)?;

    Ok(TelemetryPacket {
        topic: topic.to_owned(),
        payload: payload.to_owned(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_protocol_v1_multipart_message() {
        let frames = vec![
            b"slam/v1/pose".to_vec(),
            br#"{"v":1,"session":"test","seq":0}"#.to_vec(),
        ];

        let packet = decode_multipart(&frames).expect("message should be valid");

        assert_eq!(packet.topic, "slam/v1/pose");
        assert_eq!(packet.payload, r#"{"v":1,"session":"test","seq":0}"#);
    }

    #[test]
    fn rejects_missing_payload_frame() {
        let frames = vec![b"slam/v1/pose".to_vec()];

        let error = decode_multipart(&frames).expect_err("frame count should be rejected");

        assert_eq!(error, DecodeError::InvalidFrameCount { actual: 1 });
    }

    #[test]
    fn rejects_extra_frame() {
        let frames = vec![
            b"slam/v1/pose".to_vec(),
            br#"{"v":1}"#.to_vec(),
            b"unexpected".to_vec(),
        ];

        let error = decode_multipart(&frames).expect_err("extra frame should be rejected");

        assert_eq!(error, DecodeError::InvalidFrameCount { actual: 3 });
    }

    #[test]
    fn rejects_non_utf8_topic() {
        let frames = vec![vec![0xff], br#"{"v":1}"#.to_vec()];

        let error = decode_multipart(&frames).expect_err("invalid topic should be rejected");

        assert_eq!(error, DecodeError::TopicNotUtf8);
    }

    #[test]
    fn rejects_non_utf8_payload() {
        let frames = vec![b"slam/v1/pose".to_vec(), vec![0xff]];

        let error = decode_multipart(&frames).expect_err("invalid payload should be rejected");

        assert_eq!(error, DecodeError::PayloadNotUtf8);
    }

    #[test]
    fn leaves_json_validation_to_protocol_parser() {
        let frames = vec![b"slam/v1/pose".to_vec(), b"{not-json}".to_vec()];

        let packet =
            decode_multipart(&frames).expect("transport decoder should preserve the UTF-8 payload");

        assert_eq!(packet.topic, "slam/v1/pose");
        assert_eq!(packet.payload, "{not-json}");
    }

    #[test]
    fn rejects_payload_larger_than_protocol_limit() {
        let oversized_length = MAX_PAYLOAD_BYTES + 1;
        let frames = vec![b"slam/v1/pose".to_vec(), vec![b' '; oversized_length]];

        let error = decode_multipart(&frames).expect_err("oversized payload should be rejected");

        assert_eq!(
            error,
            DecodeError::PayloadTooLarge {
                actual: oversized_length,
                max: MAX_PAYLOAD_BYTES,
            }
        );
    }
}
