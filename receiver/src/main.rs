use std::{
    error::Error,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

use clap::Parser;
use slam_receiver::{
    PROTOCOL_V1_TOPIC_PREFIX, decode_multipart,
    protocol::{TelemetryMessage, parse_telemetry},
};

const DEFAULT_ENDPOINT: &str = "tcp://127.0.0.1:5555";
const RECEIVE_TIMEOUT_MS: i32 = 250;

#[derive(Debug, Parser)]
#[command(
    name = "slam-receiver",
    version,
    about = "Receive Protocol v1 telemetry over ZeroMQ"
)]
struct Cli {
    /// ZeroMQ PUB endpoint to connect to
    #[arg(long, default_value = DEFAULT_ENDPOINT)]
    endpoint: String,
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

    let running = Arc::new(AtomicBool::new(true));
    let running_for_handler = Arc::clone(&running);

    ctrlc::set_handler(move || {
        running_for_handler.store(false, Ordering::SeqCst);
    })?;

    println!("receiver connected to {}", cli.endpoint);
    println!("subscribed to {PROTOCOL_V1_TOPIC_PREFIX}");
    println!("press Ctrl-C to stop");

    let mut received_count = 0_u64;
    let mut rejected_count = 0_u64;

    while running.load(Ordering::SeqCst) {
        let frames = match subscriber.recv_multipart(0) {
            Ok(frames) => frames,
            Err(zmq::Error::EAGAIN) => continue,
            Err(zmq::Error::EINTR) => continue,
            Err(error) => return Err(error.into()),
        };

        match decode_multipart(&frames) {
            Ok(packet) => match parse_telemetry(&packet.topic, &packet.payload) {
                Ok(message) => {
                    received_count += 1;
                    log_received(&message);
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

    println!("receiver stopped: received={received_count}, rejected={rejected_count}");

    Ok(())
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
        assert_eq!(cli.endpoint, DEFAULT_ENDPOINT);
    }

    #[test]
    fn parses_custom_endpoint() {
        let cli = Cli::parse_from(["slam-receiver", "--endpoint", "tcp://192.168.1.10:5555"]);

        assert_eq!(cli.endpoint, "tcp://192.168.1.10:5555");
    }
}
