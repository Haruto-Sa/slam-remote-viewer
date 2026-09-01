use std::{
    error::Error,
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

use clap::Parser;
use slam_receiver::{
    PROTOCOL_V1_TOPIC_PREFIX,
    clip::ClipRecorder,
    control::handle_control_request,
    coordinates::prepare_telemetry_for_unity,
    decode_multipart,
    protocol::{TelemetryMessage, parse_telemetry},
    publisher::{EncodedTelemetry, SettingsRepeater, encode_telemetry, publish_encoded},
    quaternion::QuaternionContinuity,
    recording::{SessionGate, TelemetryRecorder, recover_incomplete_recordings},
    retention::{RetentionManager, RetentionPolicy},
};

const DEFAULT_INPUT_ENDPOINT: &str = "tcp://127.0.0.1:5555";
const DEFAULT_OUTPUT_ENDPOINT: &str = "tcp://127.0.0.1:5556";
const DEFAULT_CONTROL_ENDPOINT: &str = "tcp://127.0.0.1:5557";
const DEFAULT_RECORD_DIRECTORY: &str = "recordings";
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

    /// Local ZeroMQ REP endpoint to bind for Unity clip controls
    #[arg(long, default_value = DEFAULT_CONTROL_ENDPOINT)]
    control_endpoint: String,

    /// Directory in which accepted telemetry sessions and clips are recorded
    #[arg(long, default_value = DEFAULT_RECORD_DIRECTORY)]
    record_dir: PathBuf,

    /// Maximum combined size in bytes for finalized recordings and clips
    #[arg(long, value_parser = parse_positive_u64)]
    retention_max_bytes: Option<u64>,

    /// Maximum age in whole days for finalized recordings and clips
    #[arg(long, value_parser = parse_retention_days)]
    retention_max_age_days: Option<u64>,

    /// Report retention candidates without deleting them
    #[arg(long)]
    retention_dry_run: bool,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("receiver failed: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let cli = Cli::parse();

    let recovery = recover_incomplete_recordings(&cli.record_dir)?;
    for summary in &recovery.recovered {
        println!(
            "recording recovered: session={} messages={} points={} discarded_trailing_bytes={} \
             directory={}",
            summary.session,
            summary.message_count,
            summary.point_count,
            summary.discarded_trailing_bytes,
            summary.directory.display()
        );
    }
    for directory in &recovery.already_finalized {
        println!(
            "removed stale recording checkpoint: directory={}",
            directory.display()
        );
    }
    for failure in &recovery.failures {
        eprintln!(
            "recording recovery failed: directory={} reason={}",
            failure.directory.display(),
            failure.reason
        );
    }

    let retention = RetentionManager::new(
        cli.record_dir.clone(),
        RetentionPolicy {
            max_total_bytes: cli.retention_max_bytes,
            max_age: cli
                .retention_max_age_days
                .map(|days| Duration::from_secs(days * 24 * 60 * 60)),
            dry_run: cli.retention_dry_run,
        },
    );
    retention.apply_and_log("startup");

    let context = zmq::Context::new();
    let subscriber = context.socket(zmq::SUB)?;

    subscriber.set_linger(0)?;
    subscriber.set_subscribe(PROTOCOL_V1_TOPIC_PREFIX.as_bytes())?;
    subscriber.connect(&cli.endpoint)?;

    let publisher = context.socket(zmq::PUB)?;
    publisher.set_linger(0)?;
    publisher.bind(&cli.output_endpoint)?;

    let control = context.socket(zmq::REP)?;
    control.set_linger(0)?;
    control.bind(&cli.control_endpoint)?;

    let running = Arc::new(AtomicBool::new(true));
    let running_for_handler = Arc::clone(&running);

    ctrlc::set_handler(move || {
        running_for_handler.store(false, Ordering::SeqCst);
    })?;

    println!("receiver connected to {}", cli.endpoint);
    println!("subscribed to {PROTOCOL_V1_TOPIC_PREFIX}");
    println!("publishing Unity telemetry to {}", cli.output_endpoint);
    println!("serving Unity clip controls on {}", cli.control_endpoint);
    println!(
        "recording telemetry sessions and clips under {}",
        cli.record_dir.display()
    );
    println!("press Ctrl-C to stop");

    let mut received_count = 0_u64;
    let mut rejected_count = 0_u64;
    let mut published_count = 0_u64;
    let mut publication_failed_count = 0_u64;
    let mut quaternion_continuity = QuaternionContinuity::new();
    let mut settings_repeater = SettingsRepeater::new(SETTINGS_REPEAT_INTERVAL);
    let mut session_gate = SessionGate::new();
    let recorder =
        TelemetryRecorder::start_with_retention(cli.record_dir.clone(), retention.clone());
    let clip_recorder = ClipRecorder::start_with_retention(cli.record_dir, retention);

    while running.load(Ordering::SeqCst) {
        if let Some(settings) = settings_repeater.take_due(Instant::now()) {
            publish(
                &publisher,
                &settings,
                &mut published_count,
                &mut publication_failed_count,
            );
        }

        let mut poll_items = [
            subscriber.as_poll_item(zmq::POLLIN),
            control.as_poll_item(zmq::POLLIN),
        ];
        match zmq::poll(&mut poll_items, i64::from(RECEIVE_TIMEOUT_MS)) {
            Ok(_) => {}
            Err(zmq::Error::EINTR) => continue,
            Err(error) => return Err(error.into()),
        }

        if poll_items[1].is_readable() {
            let request = control.recv_bytes(0)?;
            let response = handle_control_request(&request, &clip_recorder);
            control.send(serde_json::to_vec(&response)?, 0)?;
        }

        if !poll_items[0].is_readable() {
            continue;
        }
        let frames = subscriber.recv_multipart(0)?;

        match decode_multipart(&frames) {
            Ok(packet) => match parse_telemetry(&packet.topic, &packet.payload) {
                Ok(mut message) => {
                    if let Err(error) = session_gate.accept(&message) {
                        rejected_count += 1;
                        eprintln!(
                            "rejected telemetry: topic={} session={} seq={} reason={error}",
                            message.topic(),
                            message.session(),
                            format_seq(message.seq())
                        );
                        continue;
                    }

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

                    recorder.record(&message)?;
                    clip_recorder.observe(&message)?;

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

    for summary in clip_recorder.finish()? {
        println!(
            "clip finalized: session={} messages={} interval_messages={} points={} directory={}",
            summary.session,
            summary.message_count,
            summary.interval_message_count,
            summary.point_count,
            summary.directory.display()
        );
    }
    for summary in recorder.finish()? {
        println!(
            "recording finalized: session={} messages={} poses={} pointcloud_messages={} \
             points={} directory={}",
            summary.session,
            summary.message_count,
            summary.pose_count,
            summary.pointcloud_message_count,
            summary.point_count,
            summary.directory.display()
        );
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

fn parse_positive_u64(value: &str) -> Result<u64, String> {
    let parsed = value
        .parse::<u64>()
        .map_err(|error| format!("invalid positive integer {value:?}: {error}"))?;
    if parsed == 0 {
        Err("value must be greater than zero".to_owned())
    } else {
        Ok(parsed)
    }
}

fn parse_retention_days(value: &str) -> Result<u64, String> {
    let days = parse_positive_u64(value)?;
    if days > u64::MAX / (24 * 60 * 60) {
        Err("retention age is too large".to_owned())
    } else {
        Ok(days)
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
        assert_eq!(cli.control_endpoint, DEFAULT_CONTROL_ENDPOINT);
        assert_eq!(cli.record_dir, PathBuf::from(DEFAULT_RECORD_DIRECTORY));
        assert_eq!(cli.retention_max_bytes, None);
        assert_eq!(cli.retention_max_age_days, None);
        assert!(!cli.retention_dry_run);
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

    #[test]
    fn parses_recording_directory() {
        let cli = Cli::parse_from(["slam-receiver", "--record-dir", "recordings/demo"]);

        assert_eq!(cli.record_dir, PathBuf::from("recordings/demo"));
    }

    #[test]
    fn parses_control_endpoint() {
        let cli = Cli::parse_from([
            "slam-receiver",
            "--control-endpoint",
            "tcp://127.0.0.1:6001",
        ]);

        assert_eq!(cli.control_endpoint, "tcp://127.0.0.1:6001");
    }

    #[test]
    fn parses_retention_options() {
        let cli = Cli::parse_from([
            "slam-receiver",
            "--retention-max-bytes",
            "1048576",
            "--retention-max-age-days",
            "30",
            "--retention-dry-run",
        ]);

        assert_eq!(cli.retention_max_bytes, Some(1_048_576));
        assert_eq!(cli.retention_max_age_days, Some(30));
        assert!(cli.retention_dry_run);
    }

    #[test]
    fn rejects_zero_retention_limits() {
        assert!(Cli::try_parse_from(["slam-receiver", "--retention-max-bytes", "0"]).is_err());
        assert!(Cli::try_parse_from(["slam-receiver", "--retention-max-age-days", "0"]).is_err());
    }
}
