use std::{
    error::Error,
    fs,
    os::unix::fs::FileTypeExt,
    path::{Path, PathBuf},
    process::{self, Child, Command, ExitStatus, Stdio},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant},
};

use clap::Parser;

const DEFAULT_SOCKET: &str = "/private/tmp/slam-live.sock";
const DEFAULT_ENDPOINT: &str = "tcp://*:5555";
const POLL_INTERVAL: Duration = Duration::from_millis(25);
const SHUTDOWN_GRACE_PERIOD: Duration = Duration::from_secs(5);

#[derive(Debug, Parser)]
#[command(
    version,
    about = "Start and supervise the macOS live SLAM producer and Rust streamer"
)]
struct Cli {
    /// Existing slam-mock-sender executable.
    #[arg(long)]
    streamer: PathBuf,
    /// Existing orbslam3_macos_camera_sender executable.
    #[arg(long)]
    producer: PathBuf,
    /// ORB-SLAM3 vocabulary file.
    #[arg(long)]
    vocabulary: PathBuf,
    /// ORB-SLAM3 camera settings file.
    #[arg(long)]
    settings: PathBuf,
    /// AVFoundation camera unique ID.
    #[arg(long)]
    device_id: String,
    #[arg(long, default_value_t = 640)]
    width: u32,
    #[arg(long, default_value_t = 480)]
    height: u32,
    #[arg(long, default_value_t = 30)]
    fps: u32,
    #[arg(long, default_value = DEFAULT_SOCKET)]
    slam_socket: PathBuf,
    #[arg(long, default_value = DEFAULT_ENDPOINT)]
    endpoint: String,
    #[arg(long, default_value = "live-session")]
    session: String,
    #[arg(long, default_value = "mac-camera")]
    camera_id: String,
    #[arg(long, default_value_t = 30)]
    pointcloud_period: u32,
    /// Seconds to wait for the streamer socket.
    #[arg(long, default_value_t = 10)]
    startup_timeout_sec: u64,
    /// Do not open the producer diagnostics window.
    #[arg(long)]
    headless: bool,
}

#[derive(Debug)]
struct LaunchCommands {
    streamer_program: PathBuf,
    streamer_args: Vec<String>,
    producer_program: PathBuf,
    producer_args: Vec<String>,
}

impl Cli {
    fn validate(&self) -> Result<(), String> {
        for (value, name) in [
            (self.width, "width"),
            (self.height, "height"),
            (self.fps, "fps"),
            (self.pointcloud_period, "pointcloud-period"),
        ] {
            if value == 0 {
                return Err(format!("--{name} must be greater than zero"));
            }
        }
        if self.startup_timeout_sec == 0 {
            return Err("--startup-timeout-sec must be greater than zero".into());
        }
        for (value, name) in [
            (&self.device_id, "device-id"),
            (&self.endpoint, "endpoint"),
            (&self.session, "session"),
            (&self.camera_id, "camera-id"),
        ] {
            if value.trim().is_empty() {
                return Err(format!("--{name} must not be empty"));
            }
        }
        Ok(())
    }

    fn commands(&self) -> LaunchCommands {
        let mut producer_args = vec![
            self.vocabulary.display().to_string(),
            self.settings.display().to_string(),
            self.device_id.clone(),
            self.width.to_string(),
            self.height.to_string(),
            self.fps.to_string(),
            self.slam_socket.display().to_string(),
            self.session.clone(),
            self.camera_id.clone(),
            "0".into(),
            self.pointcloud_period.to_string(),
        ];
        if !self.headless {
            producer_args.push("--diagnostics".into());
        }
        LaunchCommands {
            streamer_program: self.streamer.clone(),
            streamer_args: vec![
                "--source".into(),
                "live".into(),
                "--slam-socket".into(),
                self.slam_socket.display().to_string(),
                "--endpoint".into(),
                self.endpoint.clone(),
            ],
            producer_program: self.producer.clone(),
            producer_args,
        }
    }
}

fn main() {
    if let Err(error) = run(Cli::parse()) {
        eprintln!("live sender launcher failed: {error}");
        process::exit(1);
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    cli.validate()?;
    require_file(&cli.streamer, "streamer executable")?;
    require_file(&cli.producer, "producer executable")?;
    require_file(&cli.vocabulary, "vocabulary")?;
    require_file(&cli.settings, "settings")?;
    if cli.slam_socket.exists() {
        return Err(format!(
            "SLAM socket already exists: {}; remove a stale socket only after confirming no Sender owns it",
            cli.slam_socket.display()
        )
        .into());
    }

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))?;

    let commands = cli.commands();
    println!("starting streamer: {}", commands.streamer_program.display());
    let mut streamer = spawn(&commands.streamer_program, &commands.streamer_args)?;
    if let Err(error) = wait_for_socket(
        &cli.slam_socket,
        Duration::from_secs(cli.startup_timeout_sec),
        &running,
        &mut streamer,
    ) {
        stop_and_reap(&mut streamer);
        return Err(error);
    }

    println!("streamer ready: {}", cli.slam_socket.display());
    println!("starting producer: {}", commands.producer_program.display());
    let mut producer = match spawn(&commands.producer_program, &commands.producer_args) {
        Ok(child) => child,
        Err(error) => {
            stop_and_reap(&mut streamer);
            return Err(error);
        }
    };
    println!("live sender running; press Ctrl-C or use the diagnostics Stop button");

    let result = supervise(&running, &mut streamer, &mut producer);
    stop_and_reap(&mut producer);
    stop_and_reap(&mut streamer);
    result
}

fn require_file(path: &Path, name: &str) -> Result<(), String> {
    if path.is_file() {
        Ok(())
    } else {
        Err(format!("{name} does not exist: {}", path.display()))
    }
}

fn spawn(program: &Path, args: &[String]) -> Result<Child, Box<dyn Error>> {
    Ok(Command::new(program)
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .spawn()?)
}

fn wait_for_socket(
    socket: &Path,
    timeout: Duration,
    running: &AtomicBool,
    streamer: &mut Child,
) -> Result<(), Box<dyn Error>> {
    let deadline = Instant::now() + timeout;
    loop {
        if !running.load(Ordering::SeqCst) {
            return Err("interrupted while waiting for the streamer".into());
        }
        if let Some(status) = streamer.try_wait()? {
            return Err(format!("streamer exited before becoming ready: {status}").into());
        }
        match fs::symlink_metadata(socket) {
            Ok(metadata) if metadata.file_type().is_socket() => return Ok(()),
            Ok(_) => {
                return Err(
                    format!("streamer path is not a Unix socket: {}", socket.display()).into(),
                );
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(error.into()),
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "streamer did not create {} within {:.1} seconds",
                socket.display(),
                timeout.as_secs_f64()
            )
            .into());
        }
        thread::sleep(POLL_INTERVAL);
    }
}

fn supervise(
    running: &AtomicBool,
    streamer: &mut Child,
    producer: &mut Child,
) -> Result<(), Box<dyn Error>> {
    loop {
        if !running.load(Ordering::SeqCst) {
            signal_interrupt(producer);
            signal_interrupt(streamer);
            return Ok(());
        }
        if let Some(status) = producer.try_wait()? {
            return finish_after_producer(status, streamer);
        }
        if let Some(status) = streamer.try_wait()? {
            return finish_after_streamer(status, producer);
        }
        thread::sleep(POLL_INTERVAL);
    }
}

fn finish_after_streamer(
    streamer_status: ExitStatus,
    producer: &mut Child,
) -> Result<(), Box<dyn Error>> {
    if !streamer_status.success() {
        signal_interrupt(producer);
        return Err(
            format!("streamer exited while producer was running: {streamer_status}").into(),
        );
    }
    let deadline = Instant::now() + SHUTDOWN_GRACE_PERIOD;
    loop {
        if let Some(producer_status) = producer.try_wait()? {
            if producer_status.success() {
                println!("live sender stopped cleanly");
                return Ok(());
            }
            return Err(format!(
                "child failure: producer={producer_status}, streamer={streamer_status}"
            )
            .into());
        }
        if Instant::now() >= deadline {
            signal_interrupt(producer);
            return Err(format!(
                "producer did not exit within 5 seconds after streamer {streamer_status}"
            )
            .into());
        }
        thread::sleep(POLL_INTERVAL);
    }
}

fn finish_after_producer(
    producer_status: ExitStatus,
    streamer: &mut Child,
) -> Result<(), Box<dyn Error>> {
    let deadline = Instant::now() + SHUTDOWN_GRACE_PERIOD;
    loop {
        if let Some(streamer_status) = streamer.try_wait()? {
            if producer_status.success() && streamer_status.success() {
                println!("live sender stopped cleanly");
                return Ok(());
            }
            return Err(format!(
                "child failure: producer={producer_status}, streamer={streamer_status}"
            )
            .into());
        }
        if Instant::now() >= deadline {
            signal_interrupt(streamer);
            return Err(format!(
                "streamer did not exit within 5 seconds after producer {producer_status}"
            )
            .into());
        }
        thread::sleep(POLL_INTERVAL);
    }
}

fn signal_interrupt(child: &mut Child) {
    if child.try_wait().ok().flatten().is_none() {
        // SAFETY: kill is called with a child PID returned by std::process and
        // SIGINT has no pointer or lifetime requirements.
        unsafe {
            libc::kill(child.id() as libc::pid_t, libc::SIGINT);
        }
    }
}

fn stop_and_reap(child: &mut Child) {
    if child.try_wait().ok().flatten().is_some() {
        return;
    }
    signal_interrupt(child);
    let deadline = Instant::now() + SHUTDOWN_GRACE_PERIOD;
    while Instant::now() < deadline {
        if child.try_wait().ok().flatten().is_some() {
            return;
        }
        thread::sleep(POLL_INTERVAL);
    }
    let _ = child.kill();
    let _ = child.wait();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        os::unix::net::UnixListener,
        sync::atomic::AtomicBool,
        time::{SystemTime, UNIX_EPOCH},
    };

    fn cli() -> Cli {
        Cli::try_parse_from([
            "macos-live-sender",
            "--streamer",
            "/tmp/streamer",
            "--producer",
            "/tmp/producer",
            "--vocabulary",
            "/tmp/vocabulary",
            "--settings",
            "/tmp/settings",
            "--device-id",
            "camera-device",
        ])
        .expect("valid launcher arguments")
    }

    #[test]
    fn builds_argument_arrays_with_diagnostics_by_default() {
        let commands = cli().commands();
        assert_eq!(commands.streamer_program, PathBuf::from("/tmp/streamer"));
        assert_eq!(
            commands.streamer_args,
            [
                "--source",
                "live",
                "--slam-socket",
                DEFAULT_SOCKET,
                "--endpoint",
                DEFAULT_ENDPOINT
            ]
        );
        assert_eq!(commands.producer_args.last().unwrap(), "--diagnostics");
        assert!(
            commands
                .producer_args
                .windows(2)
                .any(|items| items == ["mac-camera", "0"])
        );
    }

    #[test]
    fn headless_omits_diagnostics() {
        let mut cli = cli();
        cli.headless = true;
        assert!(
            !cli.commands()
                .producer_args
                .contains(&"--diagnostics".into())
        );
    }

    #[test]
    fn rejects_zero_runtime_values() {
        let mut cli = cli();
        cli.pointcloud_period = 0;
        assert_eq!(
            cli.validate().unwrap_err(),
            "--pointcloud-period must be greater than zero"
        );
    }

    #[test]
    fn observes_a_unix_socket_without_slam_dependencies() {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let socket =
            std::env::temp_dir().join(format!("slam-live-launcher-{}-{nonce}.sock", process::id()));
        let listener = UnixListener::bind(&socket).unwrap();
        let running = AtomicBool::new(true);
        let mut child = Command::new("/bin/sleep").arg("1").spawn().unwrap();
        let result = wait_for_socket(&socket, Duration::from_secs(1), &running, &mut child);
        child.kill().unwrap();
        child.wait().unwrap();
        drop(listener);
        fs::remove_file(&socket).unwrap();
        assert!(result.is_ok());
    }

    #[test]
    fn accepts_clean_child_completion() {
        let producer_status = Command::new("/usr/bin/true")
            .spawn()
            .unwrap()
            .wait()
            .unwrap();
        let mut streamer = Command::new("/usr/bin/true").spawn().unwrap();
        assert!(finish_after_producer(producer_status, &mut streamer).is_ok());
    }

    #[test]
    fn accepts_streamer_exiting_just_before_producer() {
        let streamer_status = Command::new("/usr/bin/true")
            .spawn()
            .unwrap()
            .wait()
            .unwrap();
        let mut producer = Command::new("/bin/sleep").arg("0.01").spawn().unwrap();
        assert!(finish_after_streamer(streamer_status, &mut producer).is_ok());
    }

    #[test]
    fn stop_reaps_a_running_child() {
        let mut child = Command::new("/bin/sleep").arg("30").spawn().unwrap();
        stop_and_reap(&mut child);
        assert!(child.try_wait().unwrap().is_some());
    }
}
