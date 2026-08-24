use std::{
    env,
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
    SettingsMessage, publish_json,
};

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

fn run() -> Result<(), Box<dyn Error>> {
    let endpoint = env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_ENDPOINT.to_owned());
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

    let pose_interval = Duration::from_secs_f64(1.0 / POSE_RATE_HZ);

    let mut next_settings = Instant::now();
    let mut next_pose = Instant::now();
    let mut next_pointcloud = Instant::now();

    let mut pose_seq = 0_u64;
    let mut pointcloud_seq = 0_u64;

    while running.load(Ordering::SeqCst) {
        let now = Instant::now();

        if now >= next_settings {
            let settings = SettingsMessage::mock(DEFAULT_SESSION);
            publish_json(&publisher, SETTINGS_TOPIC, &settings)?;

            println!("published {SETTINGS_TOPIC}");
            next_settings += SETTINGS_INTERVAL;
        }

        if now >= next_pointcloud {
            let pointcloud_time = pointcloud_seq as f64 * POINTCLOUD_INTERVAL.as_secs_f64();

            let pointcloud =
                PointCloudDeltaMessage::fixture(DEFAULT_SESSION, pointcloud_seq, pointcloud_time);
            publish_json(&publisher, POINTCLOUD_TOPIC, &pointcloud)?;
            println!("published {POINTCLOUD_TOPIC} seq={pointcloud_seq}");
            pointcloud_seq += 1;
            next_pointcloud += POINTCLOUD_INTERVAL;
        }

        if now >= next_pose {
            let pose_time = pose_seq as f64 / POSE_RATE_HZ;
            let pose = PoseMessage::circular(
                DEFAULT_SESSION,
                pose_seq,
                pose_time,
                TRAJECTORY_RADIUS_M,
                ANGULAR_SPEED_RAD_PER_SEC,
            );
            publish_json(&publisher, POSE_TOPIC, &pose)?;

            pose_seq += 1;
            next_pose += pose_interval;
        }

        thread::sleep(Duration::from_millis(1));
    }

    println!("mock sender stopped: poses={pose_seq}, pointclouds={pointcloud_seq}");

    Ok(())
}
