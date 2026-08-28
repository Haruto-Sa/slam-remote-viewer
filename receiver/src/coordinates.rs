use crate::protocol::{PointEntry, TelemetryMessage};

pub const UNITY_WORLD_FRAME: &str = "unity_world";

pub fn position_to_unity(position: [f64; 3]) -> [f64; 3] {
    let [x, y, z] = position;

    [x, -y, z]
}

pub fn point_entry_to_unity(point: PointEntry) -> PointEntry {
    let (id, x, y, z) = point;

    (id, x, -y, z)
}

pub fn quaternion_to_unity(quaternion: [f64; 4]) -> [f64; 4] {
    let [x, y, z, w] = quaternion;

    [-x, y, -z, w]
}

pub fn telemetry_to_unity(message: &mut TelemetryMessage) {
    match message {
        TelemetryMessage::Settings(settings) => {
            settings.frame = UNITY_WORLD_FRAME.to_owned();
        }
        TelemetryMessage::Pose(pose) => {
            pose.p = position_to_unity(pose.p);
            pose.q = quaternion_to_unity(pose.q);
        }
        TelemetryMessage::PointCloud(pointcloud) => {
            pointcloud
                .add
                .iter_mut()
                .for_each(|point| *point = point_entry_to_unity(*point));
            pointcloud
                .update
                .iter_mut()
                .for_each(|point| *point = point_entry_to_unity(*point));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{
        CameraSettings, PointCloudMessage, PoseMessage, PoseState, SettingsMessage,
    };
    use crate::quaternion::normalize;

    const TOLERANCE: f64 = 1.0e-12;

    fn assert_quaternion_close(actual: [f64; 4], expected: [f64; 4]) {
        for (index, (actual, expected)) in actual.into_iter().zip(expected).enumerate() {
            assert!(
                (actual - expected).abs() <= TOLERANCE,
                "component {index}: expected {expected}, received {actual}"
            );
        }
    }

    #[test]
    fn converts_documented_position_fixture() {
        assert_eq!(position_to_unity([1.0, 2.0, 3.0]), [1.0, -2.0, 3.0]);
    }

    #[test]
    fn converts_positive_slam_axes_to_unity_axes() {
        assert_eq!(position_to_unity([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0]);

        assert_eq!(position_to_unity([0.0, 1.0, 0.0]), [0.0, -1.0, 0.0]);

        assert_eq!(position_to_unity([0.0, 0.0, 1.0]), [0.0, 0.0, 1.0]);
    }

    #[test]
    fn converts_point_and_preserves_id() {
        assert_eq!(
            point_entry_to_unity((1001, -4.0, 5.0, 6.0)),
            (1001, -4.0, -5.0, 6.0)
        );
    }

    #[test]
    fn preserves_identity_quaternion() {
        assert_eq!(
            quaternion_to_unity([0.0, 0.0, 0.0, 1.0]),
            [0.0, 0.0, 0.0, 1.0]
        );
    }

    #[test]
    fn converts_documented_quaternion_fixture() {
        let slam =
            normalize([0.1, 0.2, 0.3, 0.92736185]).expect("fixture quaternion should normalize");

        let unity = quaternion_to_unity(slam);

        let expected =
            normalize([-0.1, 0.2, -0.3, 0.92736185]).expect("expected quaternion should normalize");

        assert_quaternion_close(unity, expected);
    }

    #[test]
    fn converts_quarter_turns_around_each_axis() {
        let half_angle = std::f64::consts::FRAC_1_SQRT_2;

        let cases = [
            (
                [half_angle, 0.0, 0.0, half_angle],
                [-half_angle, 0.0, 0.0, half_angle],
            ),
            (
                [0.0, half_angle, 0.0, half_angle],
                [0.0, half_angle, 0.0, half_angle],
            ),
            (
                [0.0, 0.0, half_angle, half_angle],
                [0.0, 0.0, -half_angle, half_angle],
            ),
        ];

        for (slam, expected_unity) in cases {
            assert_quaternion_close(quaternion_to_unity(slam), expected_unity);
        }
    }

    #[test]
    fn rewrites_settings_frame_and_preserves_other_fields() {
        let mut message = TelemetryMessage::Settings(SettingsMessage {
            v: 1,
            session: "session-1".to_owned(),
            unit: "m".to_owned(),
            frame: "slam_world".to_owned(),
            pose_convention: "Twc".to_owned(),
            quaternion: "xyzw".to_owned(),
            camera: CameraSettings {
                camera_type: "pc".to_owned(),
                id: "builtin_0".to_owned(),
                width: 1280,
                height: 720,
                fps: 30,
            },
            pointcloud_mode: "delta".to_owned(),
        });

        telemetry_to_unity(&mut message);

        let TelemetryMessage::Settings(settings) = message else {
            panic!("settings message should remain settings");
        };
        assert_eq!(settings.frame, UNITY_WORLD_FRAME);
        assert_eq!(settings.session, "session-1");
        assert_eq!(settings.camera.id, "builtin_0");
    }

    #[test]
    fn converts_pose_and_preserves_metadata() {
        let mut message = TelemetryMessage::Pose(PoseMessage {
            v: 1,
            session: "session-1".to_owned(),
            seq: 42,
            t: 1.5,
            p: [1.0, 2.0, 3.0],
            q: [0.1, 0.2, 0.3, 0.4],
            state: PoseState::Tracking,
        });

        telemetry_to_unity(&mut message);

        let TelemetryMessage::Pose(pose) = message else {
            panic!("pose message should remain a pose");
        };
        assert_eq!(pose.p, [1.0, -2.0, 3.0]);
        assert_eq!(pose.q, [-0.1, 0.2, -0.3, 0.4]);
        assert_eq!(pose.seq, 42);
        assert_eq!(pose.state, PoseState::Tracking);
    }

    #[test]
    fn converts_pointcloud_add_and_update_but_preserves_ids_and_removals() {
        let mut message = TelemetryMessage::PointCloud(PointCloudMessage {
            v: 1,
            session: "session-1".to_owned(),
            seq: 7,
            t: 2.5,
            add: vec![(1001, 1.0, 2.0, 3.0)],
            update: vec![(1002, -4.0, 5.0, 6.0)],
            remove: vec![1003],
        });

        telemetry_to_unity(&mut message);

        let TelemetryMessage::PointCloud(pointcloud) = message else {
            panic!("point-cloud message should remain a point cloud");
        };
        assert_eq!(pointcloud.add, vec![(1001, 1.0, -2.0, 3.0)]);
        assert_eq!(pointcloud.update, vec![(1002, -4.0, -5.0, 6.0)]);
        assert_eq!(pointcloud.remove, vec![1003]);
        assert_eq!(pointcloud.seq, 7);
    }
}
