use std::{
    error::Error,
    process,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant},
};

use slam_mock_sender::{
    POINTCLOUD_TOPIC, POSE_TOPIC, PointCloudDeltaMessage, PoseMessage, SETTINGS_TOPIC,
    SettingsMessage,
    pose_source::{MockPoseSource, PoseSource},
    publish_json,
};

use clap::Parser;

const DEFAULT_ENDPOINT: &str = "tcp://*:5555";
const DEFAULT_SESSION: &str = "mock-session-001";

const POSE_RATE_HZ: f64 = 30.0;
const SETTINGS_INTERVAL: Duration = Duration::from_secs(5);
const POINTCLOUD_INTERVAL: Duration = Duration::from_secs(5);

const TRAJECTORY_RADIUS_M: f64 = 2.0;
const ANGULAR_SPEED_RAD_PER_SEC: f64 = 0.5;

fn main() {
    if let Err(error) = run() {
        eprintln!("mock sender failed: {error}");
        process::exit(1);
    }
}

#[derive(Debug, Parser)]
#[command(
    version,
    about = "Publish deterministic Protocol v1 telemetry over ZeroMQ"
)]
struct Cli {
    /// ZeroMQ PUB endpoint to bind
    #[arg(long, default_value = DEFAULT_ENDPOINT)]
    endpoint: String,
    /// Protocol session identifier.
    #[arg(
        long,
        default_value = DEFAULT_SESSION,
        value_parser = parse_non_empty_string
    )]
    session: String,

    /// Number of poses published per second.
    #[arg(
        long,
        default_value_t = POSE_RATE_HZ,
        value_parser = parse_positive_f64
    )]
    pose_rate_hz: f64,

    /// Circular trajectory radius in metres.
    #[arg(
        long,
        default_value_t = TRAJECTORY_RADIUS_M,
        value_parser = parse_positive_f64
    )]
    radius_m: f64,
    /// Angular speed in radians per second.
    #[arg(
        long,
        default_value_t = ANGULAR_SPEED_RAD_PER_SEC,
        value_parser = parse_non_negative_f64
    )]
    angular_speed_rad_per_sec: f64,
    /// Stop after generating samples within this duration.
    #[arg(
        long,
        value_parser = parse_positive_f64
    )]
    duration_sec: Option<f64>,
}

fn parse_positive_f64(value: &str) -> Result<f64, String> {
    let parsed = value
        .parse::<f64>()
        .map_err(|error| format!("invalid number '{value}': {error}"))?;

    if parsed.is_finite() && parsed > 0.0 {
        Ok(parsed)
    } else {
        Err(format!(
            "value must be finite and greater than zero: {value}"
        ))
    }
}

fn parse_non_negative_f64(value: &str) -> Result<f64, String> {
    let parsed = value
        .parse::<f64>()
        .map_err(|error| format!("invalid number '{value}': {error}"))?;

    if parsed.is_finite() && parsed >= 0.0 {
        Ok(parsed)
    } else {
        Err(format!("value must be finite and non-negative: {value}"))
    }
}

fn parse_non_empty_string(value: &str) -> Result<String, String> {
    let trimmed = value.trim();

    if trimmed.is_empty() {
        Err("value must not be empty".to_owned())
    } else {
        Ok(trimmed.to_owned())
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let Cli {
        endpoint,
        session,
        pose_rate_hz,
        radius_m,
        angular_speed_rad_per_sec,
        duration_sec,
    } = Cli::parse();
    let mut pose_source = MockPoseSource::new(
        pose_rate_hz,
        radius_m,
        angular_speed_rad_per_sec,
        duration_sec,
    )?;
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);

    ctrlc::set_handler(move || {
        signal_running.store(false, Ordering::SeqCst);
    })?;

    let context = zmq::Context::new();
    let publisher = context.socket(zmq::PUB)?;

    publisher.set_linger(0)?;
    publisher.bind(&endpoint)?;

    println!("mock sender publishing to {endpoint}");
    println!("press Ctrl-C to stop");

    // Subscriberとの接続確立を少し待つ。
    thread::sleep(Duration::from_millis(250));

    let pose_interval = Duration::from_secs_f64(1.0 / pose_rate_hz);

    let mut next_settings = Instant::now();
    let mut next_pose = Instant::now();
    let mut next_pointcloud = Instant::now();

    let mut pose_seq = 0_u64;
    let mut pointcloud_seq = 0_u64;

    while running.load(Ordering::SeqCst) {
        let now = Instant::now();

        if now >= next_settings {
            let settings = SettingsMessage::mock(session.as_str());
            publish_json(&publisher, SETTINGS_TOPIC, &settings)?;

            println!("published {SETTINGS_TOPIC}");
            next_settings += SETTINGS_INTERVAL;
        }

        if now >= next_pointcloud {
            let pointcloud_time = pointcloud_seq as f64 * POINTCLOUD_INTERVAL.as_secs_f64();

            let pointcloud =
                PointCloudDeltaMessage::fixture(session.as_str(), pointcloud_seq, pointcloud_time);
            publish_json(&publisher, POINTCLOUD_TOPIC, &pointcloud)?;
            println!("published {POINTCLOUD_TOPIC} seq={pointcloud_seq}");
            pointcloud_seq += 1;
            next_pointcloud += POINTCLOUD_INTERVAL;
        }

        if now >= next_pose {
            let Some(slam_pose) = pose_source.next_pose()? else {
                break;
            };
            let pose = PoseMessage::from_slam_pose(session.as_str(), slam_pose);
            publish_json(&publisher, POSE_TOPIC, &pose)?;

            pose_seq += 1;
            next_pose += pose_interval;
        }

        thread::sleep(Duration::from_millis(1));
    }

    println!("mock sender stopped: poses={pose_seq}, pointclouds={pointcloud_seq}");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uses_default_cli_values() {
        let cli = Cli::try_parse_from(["slam-mock-sender"]).expect("default arguments must parse");

        assert_eq!(cli.endpoint, DEFAULT_ENDPOINT);
        assert_eq!(cli.session, DEFAULT_SESSION);
        assert_eq!(cli.pose_rate_hz, POSE_RATE_HZ);
        assert_eq!(cli.radius_m, TRAJECTORY_RADIUS_M);
        assert_eq!(cli.angular_speed_rad_per_sec, ANGULAR_SPEED_RAD_PER_SEC);
        assert_eq!(cli.duration_sec, None);
    }

    #[test]
    fn parses_custom_endpoint() {
        let cli = Cli::try_parse_from(["slam-mock-sender", "--endpoint", "tcp://127.0.0.1:6000"])
            .expect("custom endpoint must parse");

        assert_eq!(cli.endpoint, "tcp://127.0.0.1:6000");
    }

    #[test]
    fn parses_motion_and_duration_options() {
        let cli = Cli::try_parse_from([
            "slam-mock-sender",
            "--session",
            "integration-test",
            "--pose-rate-hz",
            "10",
            "--radius-m",
            "3.5",
            "--angular-speed-rad-per-sec",
            "0.25",
            "--duration-sec",
            "2",
        ])
        .expect("custom motion arguments must parse");

        assert_eq!(cli.session, "integration-test");
        assert_eq!(cli.pose_rate_hz, 10.0);
        assert_eq!(cli.radius_m, 3.5);
        assert_eq!(cli.angular_speed_rad_per_sec, 0.25);
        assert_eq!(cli.duration_sec, Some(2.0));
    }

    #[test]
    fn rejects_zero_pose_rate() {
        let result = Cli::try_parse_from(["slam-mock-sender", "--pose-rate-hz", "0"]);

        assert!(result.is_err());
    }
}
