use std::{
    error::Error,
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant},
};

use clap::Parser;
use slam_receiver::playback::{load_session, playback_schedule};

const DEFAULT_ENDPOINT: &str = "tcp://127.0.0.1:5556";
const DEFAULT_STARTUP_DELAY_MS: u64 = 500;
const WAIT_SLICE: Duration = Duration::from_millis(20);

#[derive(Debug, Parser)]
#[command(
    name = "slam-session-player",
    version,
    about = "Replay a recorded Protocol v1 telemetry session into Unity"
)]
struct Cli {
    /// Session directory containing metadata.json and telemetry.ndjson
    #[arg(long)]
    session_dir: PathBuf,

    /// ZeroMQ PUB endpoint to bind for Unity
    #[arg(long, default_value = DEFAULT_ENDPOINT)]
    endpoint: String,

    /// Playback speed multiplier
    #[arg(long, default_value = "1.0", value_parser = parse_speed)]
    speed: f64,

    /// Delay after binding the PUB socket so Unity can subscribe
    #[arg(long, default_value_t = DEFAULT_STARTUP_DELAY_MS)]
    startup_delay_ms: u64,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("session player failed: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let cli = Cli::parse();
    let session = load_session(&cli.session_dir)?;
    let schedule = playback_schedule(session.messages(), cli.speed)?;

    let context = zmq::Context::new();
    let publisher = context.socket(zmq::PUB)?;
    publisher.set_linger(0)?;
    publisher.bind(&cli.endpoint)?;

    let running = Arc::new(AtomicBool::new(true));
    let running_for_handler = Arc::clone(&running);
    ctrlc::set_handler(move || {
        running_for_handler.store(false, Ordering::SeqCst);
    })?;

    println!(
        "loaded session={} messages={} points={} directory={}",
        session.metadata().session,
        session.messages().len(),
        session.metadata().point_count,
        session.directory().display()
    );
    println!("publishing recorded telemetry to {}", cli.endpoint);
    println!("playback speed={}x", cli.speed);
    println!("press Ctrl-C to stop");

    if !wait_for(Duration::from_millis(cli.startup_delay_ms), &running) {
        println!("session player stopped before playback: published=0");
        return Ok(());
    }

    let playback_start = Instant::now();
    let mut published_count = 0_u64;
    for (message, offset) in session.messages().iter().zip(schedule) {
        if !wait_until(playback_start + offset, &running) {
            break;
        }
        publisher.send_multipart([message.topic().as_bytes(), message.payload()], 0)?;
        published_count += 1;
        println!(
            "published topic={} line={}",
            message.topic(),
            message.line()
        );
    }

    let cancelled = !running.load(Ordering::SeqCst);
    println!(
        "session player stopped: published={published_count} total={} cancelled={cancelled}",
        session.messages().len()
    );
    Ok(())
}

fn wait_for(duration: Duration, running: &AtomicBool) -> bool {
    wait_until(Instant::now() + duration, running)
}

fn wait_until(deadline: Instant, running: &AtomicBool) -> bool {
    while running.load(Ordering::SeqCst) {
        let now = Instant::now();
        if now >= deadline {
            return true;
        }
        thread::sleep((deadline - now).min(WAIT_SLICE));
    }
    false
}

fn parse_speed(value: &str) -> Result<f64, String> {
    let speed: f64 = value
        .parse()
        .map_err(|error| format!("invalid playback speed {value:?}: {error}"))?;
    if !speed.is_finite() || speed <= 0.0 {
        return Err(format!(
            "playback speed must be positive and finite, received {value:?}"
        ));
    }
    Ok(speed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_required_session_directory_and_defaults() {
        let cli = Cli::parse_from(["slam-session-player", "--session-dir", "recordings/demo"]);

        assert_eq!(cli.session_dir, PathBuf::from("recordings/demo"));
        assert_eq!(cli.endpoint, DEFAULT_ENDPOINT);
        assert_eq!(cli.speed, 1.0);
        assert_eq!(cli.startup_delay_ms, DEFAULT_STARTUP_DELAY_MS);
    }

    #[test]
    fn parses_custom_playback_options() {
        let cli = Cli::parse_from([
            "slam-session-player",
            "--session-dir",
            "recordings/demo",
            "--endpoint",
            "tcp://127.0.0.1:6000",
            "--speed",
            "2.5",
            "--startup-delay-ms",
            "1000",
        ]);

        assert_eq!(cli.endpoint, "tcp://127.0.0.1:6000");
        assert_eq!(cli.speed, 2.5);
        assert_eq!(cli.startup_delay_ms, 1000);
    }

    #[test]
    fn rejects_invalid_playback_speeds() {
        for value in ["0", "-1", "NaN", "inf", "not-a-number"] {
            assert!(parse_speed(value).is_err());
        }
    }

    #[test]
    fn zero_wait_completes_unless_cancelled() {
        let running = AtomicBool::new(true);
        assert!(wait_for(Duration::ZERO, &running));
        running.store(false, Ordering::SeqCst);
        assert!(!wait_for(Duration::ZERO, &running));
    }
}
