use std::path::PathBuf;

use clap::Parser;
use slam_mock_sender::live_slam_input::{LiveSlamEvent, LiveSlamListener};

#[derive(Debug, Parser)]
#[command(about = "Validate and summarize one live SLAM boundary session")]
struct Args {
    #[arg(long)]
    socket: PathBuf,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let mut listener = LiveSlamListener::bind(&args.socket)?;
    println!("listening socket={}", listener.local_path().display());
    let mut connection = listener.accept()?;
    let mut tracking_frames = 0_u64;
    let mut tracked_poses = 0_u64;
    loop {
        match connection.next_event()? {
            Some(LiveSlamEvent::SessionStarted {
                session_id,
                producer,
                camera,
            }) => println!(
                "session_started session={} producer={} camera={} size={}x{} fps={}",
                session_id, producer, camera.id, camera.width, camera.height, camera.fps
            ),
            Some(LiveSlamEvent::TrackingFrame(frame)) => {
                tracking_frames += 1;
                if frame.pose.is_some() {
                    tracked_poses += 1;
                }
            }
            Some(LiveSlamEvent::PointCloudDelta(_)) => {
                return Err("unexpected point-cloud delta in pose-only session".into());
            }
            Some(LiveSlamEvent::SessionEnded { session_id, reason }) => println!(
                "session_ended session={} reason={} frames={} poses={}",
                session_id, reason, tracking_frames, tracked_poses
            ),
            None => break,
        }
    }
    Ok(())
}
