use std::{thread, time::Duration};

use slam_receiver::{
    coordinates::prepare_telemetry_for_unity,
    decode_multipart,
    protocol::{SETTINGS_TOPIC, SettingsMessage, parse_telemetry},
    publisher::{encode_telemetry, publish_encoded},
    quaternion::QuaternionContinuity,
};

const SETTINGS_EXAMPLE: &str = include_str!("../../protocol/settings.example.json");

#[test]
fn republishes_converted_settings_over_pub_sub() {
    let context = zmq::Context::new();
    let publisher = context.socket(zmq::PUB).expect("publisher should open");
    let subscriber = context.socket(zmq::SUB).expect("subscriber should open");
    let endpoint = "inproc://unity-republisher-integration";

    publisher.bind(endpoint).expect("publisher should bind");
    subscriber
        .set_subscribe(b"slam/v1/")
        .expect("subscriber should subscribe");
    subscriber
        .set_rcvtimeo(1_000)
        .expect("receive timeout should be configured");
    subscriber
        .connect(endpoint)
        .expect("subscriber should connect");

    // PUB/SUB drops messages until the subscription handshake completes.
    thread::sleep(Duration::from_millis(50));

    let mut message =
        parse_telemetry(SETTINGS_TOPIC, SETTINGS_EXAMPLE).expect("settings example should parse");
    let mut continuity = QuaternionContinuity::new();
    prepare_telemetry_for_unity(&mut message, &mut continuity)
        .expect("settings should prepare for Unity");
    let encoded = encode_telemetry(&message).expect("settings should encode");

    publish_encoded(&publisher, &encoded).expect("settings should publish");

    let frames = subscriber
        .recv_multipart(0)
        .expect("subscriber should receive settings");
    let packet = decode_multipart(&frames).expect("multipart message should decode");
    let republished = parse_republished_settings(&packet.payload);

    assert_eq!(packet.topic, SETTINGS_TOPIC);
    assert_eq!(republished.frame, "unity_world");
    assert_eq!(republished.session, "001");
}

fn parse_republished_settings(payload: &str) -> SettingsMessage {
    serde_json::from_str(payload).expect("settings payload should deserialize")
}
