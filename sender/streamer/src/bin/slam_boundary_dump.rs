use std::path::PathBuf;

use clap::Parser;
use slam_mock_sender::live_slam_input::{LiveSlamEvent, LiveSlamListener};

#[derive(Debug, Parser)]
#[command(about = "Validate and summarize one live SLAM boundary session")]
struct Args {
    #[arg(long)]
    socket: PathBuf,

    /// Accept and summarize point-cloud deltas instead of treating them as unexpected.
    #[arg(long)]
    allow_pointcloud: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let mut listener = LiveSlamListener::bind(&args.socket)?;
    println!("listening socket={}", listener.local_path().display());
    let mut connection = listener.accept()?;
    let mut tracking_frames = 0_u64;
    let mut tracked_poses = 0_u64;
    let mut pointcloud_deltas = 0_u64;
    let mut pointcloud_operations = 0_usize;
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
            Some(LiveSlamEvent::PointCloudDelta(delta)) => {
                if !args.allow_pointcloud {
                    return Err(
                        "unexpected point-cloud delta; pass --allow-pointcloud to accept it".into(),
                    );
                }
                pointcloud_deltas += 1;
                pointcloud_operations += delta.add.len() + delta.update.len() + delta.remove.len();
            }
            Some(LiveSlamEvent::SessionEnded { session_id, reason }) => println!(
                "session_ended session={} reason={} frames={} poses={} pointcloud_deltas={} pointcloud_operations={}",
                session_id,
                reason,
                tracking_frames,
                tracked_poses,
                pointcloud_deltas,
                pointcloud_operations
            ),
            None => break,
        }
    }
    Ok(())
}
