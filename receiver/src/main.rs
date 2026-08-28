use std::{
    error::Error,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

use clap::Parser;
use slam_receiver::{
    PROTOCOL_V1_TOPIC_PREFIX,
    coordinates::prepare_telemetry_for_unity,
    decode_multipart,
    protocol::{TelemetryMessage, parse_telemetry},
    publisher::{EncodedTelemetry, SettingsRepeater, encode_telemetry, publish_encoded},
    quaternion::QuaternionContinuity,
};

const DEFAULT_INPUT_ENDPOINT: &str = "tcp://127.0.0.1:5555";
const DEFAULT_OUTPUT_ENDPOINT: &str = "tcp://127.0.0.1:5556";
const RECEIVE_TIMEOUT_MS: i32 = 250;
const SETTINGS_REPEAT_INTERVAL: Duration = Duration::from_secs(5);

#[derive(Debug, Parser)]
#[command(
    name = "slam-receiver",
    version,
    about = "Receive Protocol v1 telemetry over ZeroMQ"
)]
struct Cli {
    /// ZeroMQ PUB endpoint to connect to
    #[arg(long, default_value = DEFAULT_INPUT_ENDPOINT)]
    endpoint: String,

    /// Local ZeroMQ PUB endpoint to bind for Unity
    #[arg(long, default_value = DEFAULT_OUTPUT_ENDPOINT)]
    output_endpoint: String,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("receiver failed: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let cli = Cli::parse();

    let context = zmq::Context::new();
    let subscriber = context.socket(zmq::SUB)?;

    subscriber.set_linger(0)?;
    subscriber.set_subscribe(PROTOCOL_V1_TOPIC_PREFIX.as_bytes())?;
    subscriber.set_rcvtimeo(RECEIVE_TIMEOUT_MS)?;
    subscriber.connect(&cli.endpoint)?;

    let publisher = context.socket(zmq::PUB)?;
    publisher.set_linger(0)?;
    publisher.bind(&cli.output_endpoint)?;

    let running = Arc::new(AtomicBool::new(true));
    let running_for_handler = Arc::clone(&running);

    ctrlc::set_handler(move || {
        running_for_handler.store(false, Ordering::SeqCst);
    })?;

    println!("receiver connected to {}", cli.endpoint);
    println!("subscribed to {PROTOCOL_V1_TOPIC_PREFIX}");
    println!("publishing Unity telemetry to {}", cli.output_endpoint);
    println!("press Ctrl-C to stop");

    let mut received_count = 0_u64;
    let mut rejected_count = 0_u64;
    let mut published_count = 0_u64;
    let mut publication_failed_count = 0_u64;
    let mut quaternion_continuity = QuaternionContinuity::new();
    let mut settings_repeater = SettingsRepeater::new(SETTINGS_REPEAT_INTERVAL);

    while running.load(Ordering::SeqCst) {
        if let Some(settings) = settings_repeater.take_due(Instant::now()) {
            publish(
                &publisher,
                &settings,
                &mut published_count,
                &mut publication_failed_count,
            );
        }

        let frames = match subscriber.recv_multipart(0) {
            Ok(frames) => frames,
            Err(zmq::Error::EAGAIN) => continue,
            Err(zmq::Error::EINTR) => continue,
            Err(error) => return Err(error.into()),
        };

        match decode_multipart(&frames) {
            Ok(packet) => match parse_telemetry(&packet.topic, &packet.payload) {
                Ok(mut message) => {
                    if let Err(error) =
                        prepare_telemetry_for_unity(&mut message, &mut quaternion_continuity)
                    {
                        let TelemetryMessage::Pose(pose) = &message else {
                            unreachable!("only pose quaternion preparation can fail");
                        };
                        rejected_count += 1;
                        eprintln!(
                            "rejected telemetry: topic=slam/v1/pose session={} seq={} reason={error}",
                            pose.session, pose.seq
                        );
                        continue;
                    }

                    received_count += 1;
                    log_received(&message);

                    match encode_telemetry(&message) {
                        Ok(telemetry) => {
                            settings_repeater.remember(&telemetry, Instant::now());
                            publish(
                                &publisher,
                                &telemetry,
                                &mut published_count,
                                &mut publication_failed_count,
                            );
                        }
                        Err(error) => {
                            publication_failed_count += 1;
                            eprintln!(
                                "failed to publish telemetry: topic={} session={} seq={} reason={error}",
                                message.topic(),
                                message.session(),
                                format_seq(message.seq())
                            );
                        }
                    }
                }
                Err(error) => {
                    rejected_count += 1;
                    eprintln!("rejected telemetry: {error}");
                }
            },
            Err(error) => {
                rejected_count += 1;
                eprintln!("rejected multipart message: {error}");
            }
        }
    }

    println!(
        "receiver stopped: received={received_count}, rejected={rejected_count}, \
         published={published_count}, publication_failed={publication_failed_count}"
    );

    Ok(())
}

fn publish(
    publisher: &zmq::Socket,
    telemetry: &EncodedTelemetry,
    published_count: &mut u64,
    publication_failed_count: &mut u64,
) {
    match publish_encoded(publisher, telemetry) {
        Ok(()) => {
            *published_count += 1;
        }
        Err(error) => {
            *publication_failed_count += 1;
            eprintln!(
                "failed to publish telemetry: topic={} session={} seq={} reason={error}",
                telemetry.topic(),
                telemetry.session(),
                format_seq(telemetry.seq())
            );
        }
    }
}

fn format_seq(seq: Option<u64>) -> String {
    seq.map_or_else(|| "-".to_owned(), |seq| seq.to_string())
}

fn log_received(message: &TelemetryMessage) {
    match message.seq() {
        Some(seq) => println!(
            "received topic={} session={} seq={seq}",
            message.topic(),
            message.session()
        ),
        None => println!(
            "received topic={} session={} seq=-",
            message.topic(),
            message.session()
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uses_default_endpoint() {
        let cli = Cli::parse_from(["slam-receiver"]);
        assert_eq!(cli.endpoint, DEFAULT_INPUT_ENDPOINT);
        assert_eq!(cli.output_endpoint, DEFAULT_OUTPUT_ENDPOINT);
    }

    #[test]
    fn parses_custom_endpoint() {
        let cli = Cli::parse_from(["slam-receiver", "--endpoint", "tcp://192.168.1.10:5555"]);

        assert_eq!(cli.endpoint, "tcp://192.168.1.10:5555");
    }

    #[test]
    fn parses_custom_output_endpoint() {
        let cli = Cli::parse_from(["slam-receiver", "--output-endpoint", "tcp://127.0.0.1:6000"]);

        assert_eq!(cli.output_endpoint, "tcp://127.0.0.1:6000");
    }
}
