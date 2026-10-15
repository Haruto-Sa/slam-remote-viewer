use std::{
    collections::BTreeMap,
    fs,
    io::Write,
    os::unix::net::UnixStream,
    path::PathBuf,
    process::{Child, Command, Stdio},
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use serde_json::{Value, json};
use slam_receiver::{
    coordinates::prepare_telemetry_for_unity,
    decode_multipart,
    playback::load_session,
    protocol::{TelemetryMessage, parse_telemetry},
    quaternion::QuaternionContinuity,
    recording::{SessionGate, TelemetryRecorder},
};

struct SenderProcess {
    child: Child,
    root: PathBuf,
}
impl Drop for SenderProcess {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        let _ = fs::remove_dir_all(&self.root);
    }
}

fn send(stream: &mut UnixStream, message: Value) {
    let payload = serde_json::to_vec(&message).unwrap();
    stream
        .write_all(&(payload.len() as u32).to_be_bytes())
        .unwrap();
    stream.write_all(&payload).unwrap();
}

fn delta(frame: u64, add: Value, update: Value, remove: Value) -> Value {
    json!({"type":"pointcloud_delta", "boundary_version":1, "session_id":"resync-test",
        "frame_id":frame, "timestamp_seconds":frame as f64, "add":add, "update":update, "remove":remove})
}

fn subscribe(context: &zmq::Context, endpoint: &str) -> zmq::Socket {
    let socket = context.socket(zmq::SUB).unwrap();
    socket.set_linger(0).unwrap();
    socket.set_subscribe(b"slam/v1/").unwrap();
    socket.set_rcvtimeo(500).unwrap();
    socket.connect(endpoint).unwrap();
    socket
}

struct Viewer {
    gate: SessionGate,
    continuity: QuaternionContinuity,
    points: BTreeMap<u64, [f64; 3]>,
    recorder: TelemetryRecorder,
}
impl Viewer {
    fn new(root: &std::path::Path) -> Self {
        Self {
            gate: SessionGate::new(),
            continuity: QuaternionContinuity::new(),
            points: BTreeMap::new(),
            recorder: TelemetryRecorder::start(root),
        }
    }

    fn receive_until(
        &mut self,
        socket: &zmq::Socket,
        predicate: impl Fn(&TelemetryMessage) -> bool,
    ) {
        let deadline = Instant::now() + Duration::from_secs(15);
        loop {
            assert!(
                Instant::now() < deadline,
                "timed out waiting for sender resynchronization"
            );
            let frames = match socket.recv_multipart(0) {
                Ok(frames) => frames,
                Err(zmq::Error::EAGAIN) => continue,
                Err(error) => panic!("{error}"),
            };
            let packet = decode_multipart(&frames).unwrap();
            let mut message = parse_telemetry(&packet.topic, &packet.payload).unwrap();
            // A late subscriber may see deltas before the next settings repeat.
            if self.gate.accept(&message).is_err() {
                continue;
            }
            prepare_telemetry_for_unity(&mut message, &mut self.continuity).unwrap();
            self.recorder.record(&message).unwrap();
            if let TelemetryMessage::PointCloud(cloud) = &message {
                if cloud.snapshot {
                    self.points.clear();
                }
                for id in &cloud.remove {
                    self.points.remove(id);
                }
                for &(id, x, y, z) in cloud.update.iter().chain(&cloud.add) {
                    self.points.insert(id, [x, y, z]);
                }
            }
            if predicate(&message) {
                return;
            }
        }
    }
}

#[test]
fn live_process_recovers_late_and_reconnected_viewers_while_producer_is_idle() {
    let unique = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let root = PathBuf::from(format!(
        "/private/tmp/slam-resync-{}-{unique}",
        std::process::id()
    ));
    fs::create_dir(&root).unwrap();
    let boundary = root.join("input.sock");
    let endpoint = format!("ipc://{}", root.join("telemetry.sock").display());
    let child = Command::new(env!("CARGO_BIN_EXE_slam-mock-sender"))
        .args([
            "--source",
            "live",
            "--pointcloud-snapshots",
            "--slam-socket",
        ])
        .arg(&boundary)
        .args(["--endpoint", &endpoint])
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()
        .unwrap();
    let mut sender = SenderProcess { child, root };
    let deadline = Instant::now() + Duration::from_secs(10);
    while !boundary.exists() {
        assert!(Instant::now() < deadline && sender.child.try_wait().unwrap().is_none());
        thread::sleep(Duration::from_millis(10));
    }
    let context = zmq::Context::new();
    let early_socket = subscribe(&context, &endpoint);
    let mut early = Viewer::new(&sender.root.join("early"));
    let mut stream = UnixStream::connect(&boundary).unwrap();
    send(
        &mut stream,
        json!({"type":"hello","boundary_version":1,"session_id":"resync-test",
        "producer":"integration-camera", "camera":{"camera_type":"monocular","id":"camera-a","width":640,"height":480,"fps":30}}),
    );
    early.receive_until(&early_socket, |message| {
        matches!(message, TelemetryMessage::Settings(_))
    });
    send(
        &mut stream,
        delta(
            1,
            json!([{"id":1,"position":[1.0,2.0,3.0]},{"id":2,"position":[2.0,3.0,4.0]}]),
            json!([]),
            json!([]),
        ),
    );
    early.receive_until(
        &early_socket,
        |m| matches!(m, TelemetryMessage::PointCloud(p) if !p.snapshot && p.t == 1.0),
    );
    assert_eq!(early.points.len(), 2);
    send(
        &mut stream,
        delta(
            2,
            json!([{"id":3,"position":[3.0,4.0,5.0]}]),
            json!([{"id":2,"position":[20.0,30.0,40.0]}]),
            json!([1]),
        ),
    );
    early.receive_until(
        &early_socket,
        |m| matches!(m, TelemetryMessage::PointCloud(p) if p.t == 2.0),
    );

    let mut late = Viewer::new(&sender.root.join("late"));
    let late_socket = subscribe(&context, &endpoint);
    let at_snapshot = |m: &TelemetryMessage| matches!(m, TelemetryMessage::PointCloud(p) if p.snapshot && p.t == 2.0);
    // No more producer messages: the sender's actual idle loop must repeat state.
    late.receive_until(&late_socket, at_snapshot);
    early.receive_until(&early_socket, at_snapshot);
    assert_eq!(early.points, late.points);
    assert_eq!(
        late.points,
        BTreeMap::from([(2, [20.0, -30.0, 40.0]), (3, [3.0, -4.0, 5.0])])
    );

    drop(early_socket);
    send(&mut stream, delta(3, json!([]), json!([]), json!([2, 3])));
    late.receive_until(
        &late_socket,
        |m| matches!(m, TelemetryMessage::PointCloud(p) if p.t == 3.0),
    );
    assert!(late.points.is_empty());
    assert_eq!(early.points.len(), 2); // Simulates the stale map kept by a disconnected viewer.
    let reconnected = subscribe(&context, &endpoint);
    early.receive_until(
        &reconnected,
        |m| matches!(m, TelemetryMessage::PointCloud(p) if p.snapshot && p.t == 3.0),
    );
    assert!(early.points.is_empty());

    send(
        &mut stream,
        json!({"type":"session_end","boundary_version":1,"session_id":"resync-test","reason":"shutdown"}),
    );
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if let Some(status) = sender.child.try_wait().unwrap() {
            assert!(status.success());
            break;
        }
        assert!(Instant::now() < deadline, "sender did not shut down");
        thread::sleep(Duration::from_millis(10));
    }
    for viewer in [early, late] {
        let summaries = viewer.recorder.finish().unwrap();
        assert_eq!(summaries.len(), 1);
        assert_eq!(summaries[0].point_count, 0);
        load_session(&summaries[0].directory).unwrap();
    }
}
