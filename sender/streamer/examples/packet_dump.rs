use std::{collections::HashSet, env, error::Error, process};

use slam_mock_sender::{POINTCLOUD_TOPIC, POSE_TOPIC, SETTINGS_TOPIC};

const DEFAULT_ENDPOINT: &str = "tcp://127.0.0.1:5555";
const RECEIVE_TIMEOUT_MS: i32 = 30_000;

fn main() {
    if let Err(error) = run() {
        eprintln!("packet dump failed: {error}");
        process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let endpoint = env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_ENDPOINT.to_owned());

    let context = zmq::Context::new();
    let subscriber = context.socket(zmq::SUB)?;

    subscriber.set_subscribe(b"slam/v1/")?;
    subscriber.set_rcvtimeo(RECEIVE_TIMEOUT_MS)?;
    subscriber.connect(&endpoint)?;

    println!("packet dump connected to {endpoint}");
    println!("waiting for Protocol v1 topics");

    let expected_topics = [SETTINGS_TOPIC, POSE_TOPIC, POINTCLOUD_TOPIC];

    let mut received_topics = HashSet::new();

    while received_topics.len() < expected_topics.len() {
        let frames = match subscriber.recv_multipart(0) {
            Ok(frames) => frames,
            Err(zmq::Error::EAGAIN) => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "no telemetry received within 30 seconds; start the Mock Sender",
                )
                .into());
            }
            Err(error) => return Err(error.into()),
        };

        if frames.len() != 2 {
            eprintln!("ignored message with {} frames", frames.len());
            continue;
        }

        let topic = std::str::from_utf8(&frames[0])?;
        let payload: serde_json::Value = serde_json::from_slice(&frames[1])?;

        let sequence = payload
            .get("seq")
            .map_or_else(|| "-".to_owned(), ToString::to_string);

        println!("received topic={topic} seq={sequence}");

        if expected_topics.contains(&topic) {
            received_topics.insert(topic.to_owned());
        }
    }

    println!("received all Protocol v1 topics");

    Ok(())
}
