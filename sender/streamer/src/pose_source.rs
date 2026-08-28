use std::{convert::Infallible, error::Error, fmt};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlamTrackingState {
    Initializing,
    Tracking,
    Lost,
    Relocalizing,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SlamPose {
    /// Monotonically increasing source frame ID, mapped to Protocol v1 `seq`.
    pub frame_id: u64,
    /// Finite, non-negative seconds since the current SLAM session started.
    pub timestamp_seconds: f64,
    /// `Twc` camera position in canonical `slam_world`, measured in metres.
    pub translation: [f64; 3],
    /// `Twc` orientation in canonical `slam_world`, ordered `[x, y, z, w]`.
    pub orientation_xyzw: [f64; 4],
    pub tracking_state: SlamTrackingState,
}

/// Supplies SLAM poses without exposing backend-specific types to the Sender.
pub trait PoseSource {
    type Error: Error;

    fn next_pose(&mut self) -> Result<Option<SlamPose>, Self::Error>;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MockPoseSourceError {
    InvalidPoseRate,
    InvalidRadius,
    InvalidAngularSpeed,
    InvalidDuration,
}

impl fmt::Display for MockPoseSourceError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPoseRate => write!(formatter, "pose rate must be finite and positive"),
            Self::InvalidRadius => write!(formatter, "radius must be finite and positive"),
            Self::InvalidAngularSpeed => {
                write!(formatter, "angular speed must be finite and non-negative")
            }
            Self::InvalidDuration => write!(formatter, "duration must be finite and positive"),
        }
    }
}

impl Error for MockPoseSourceError {}

#[derive(Debug)]
pub struct MockPoseSource {
    pose_rate_hz: f64,
    radius_m: f64,
    angular_speed_rad_per_sec: f64,
    sample_limit: Option<u64>,
    next_frame_id: u64,
}

impl MockPoseSource {
    pub fn new(
        pose_rate_hz: f64,
        radius_m: f64,
        angular_speed_rad_per_sec: f64,
        duration_seconds: Option<f64>,
    ) -> Result<Self, MockPoseSourceError> {
        if !pose_rate_hz.is_finite() || pose_rate_hz <= 0.0 {
            return Err(MockPoseSourceError::InvalidPoseRate);
        }
        if !radius_m.is_finite() || radius_m <= 0.0 {
            return Err(MockPoseSourceError::InvalidRadius);
        }
        if !angular_speed_rad_per_sec.is_finite() || angular_speed_rad_per_sec < 0.0 {
            return Err(MockPoseSourceError::InvalidAngularSpeed);
        }
        if duration_seconds.is_some_and(|duration| !duration.is_finite() || duration <= 0.0) {
            return Err(MockPoseSourceError::InvalidDuration);
        }

        let sample_limit = duration_seconds
            .map(|duration| (duration * pose_rate_hz).ceil().min(u64::MAX as f64) as u64);

        Ok(Self {
            pose_rate_hz,
            radius_m,
            angular_speed_rad_per_sec,
            sample_limit,
            next_frame_id: 0,
        })
    }

    fn pose_for_frame(&self, frame_id: u64) -> SlamPose {
        let timestamp_seconds = frame_id as f64 / self.pose_rate_hz;
        let angle = self.angular_speed_rad_per_sec * timestamp_seconds;
        let half_yaw = -0.5 * angle;
        let quaternion_y = if angle == 0.0 { 0.0 } else { half_yaw.sin() };

        SlamPose {
            frame_id,
            timestamp_seconds,
            translation: [
                self.radius_m * angle.cos(),
                0.0,
                self.radius_m * angle.sin(),
            ],
            orientation_xyzw: [0.0, quaternion_y, 0.0, half_yaw.cos()],
            tracking_state: SlamTrackingState::Tracking,
        }
    }
}

impl PoseSource for MockPoseSource {
    type Error = Infallible;

    fn next_pose(&mut self) -> Result<Option<SlamPose>, Self::Error> {
        if self
            .sample_limit
            .is_some_and(|limit| self.next_frame_id >= limit)
        {
            return Ok(None);
        }

        let pose = self.pose_for_frame(self.next_frame_id);
        self.next_frame_id += 1;

        Ok(Some(pose))
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};

    use super::*;

    const TOLERANCE: f64 = 1.0e-12;

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() <= TOLERANCE,
            "expected {expected}, received {actual}"
        );
    }

    #[test]
    fn generates_deterministic_slam_poses() {
        let mut source = MockPoseSource::new(1.0, 2.0, 1.0, None)
            .expect("mock source configuration should be valid");

        let first = source
            .next_pose()
            .expect("mock source should not fail")
            .expect("first pose should exist");
        let second = source
            .next_pose()
            .expect("mock source should not fail")
            .expect("second pose should exist");

        assert_eq!(first.frame_id, 0);
        assert_eq!(first.timestamp_seconds, 0.0);
        assert_eq!(first.translation, [2.0, 0.0, 0.0]);
        assert_eq!(first.orientation_xyzw, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(first.tracking_state, SlamTrackingState::Tracking);
        assert_eq!(second.frame_id, 1);
        assert_eq!(second.timestamp_seconds, 1.0);
    }

    #[test]
    fn generates_quarter_turn_in_canonical_slam_frame() {
        let pose_rate_hz = 1.0 / FRAC_PI_2;
        let mut source = MockPoseSource::new(pose_rate_hz, 2.0, 1.0, None)
            .expect("mock source configuration should be valid");

        source.next_pose().expect("source should not fail");
        let pose = source
            .next_pose()
            .expect("source should not fail")
            .expect("second pose should exist");

        assert_close(pose.timestamp_seconds, FRAC_PI_2);
        assert_close(pose.translation[0], 0.0);
        assert_close(pose.translation[1], 0.0);
        assert_close(pose.translation[2], 2.0);
        assert_close(pose.orientation_xyzw[0], 0.0);
        assert_close(pose.orientation_xyzw[1], -FRAC_1_SQRT_2);
        assert_close(pose.orientation_xyzw[2], 0.0);
        assert_close(pose.orientation_xyzw[3], FRAC_1_SQRT_2);
    }

    #[test]
    fn stops_after_configured_duration() {
        let mut source = MockPoseSource::new(10.0, 2.0, 0.5, Some(0.25))
            .expect("mock source configuration should be valid");

        let poses = std::iter::from_fn(|| source.next_pose().expect("source should not fail"))
            .collect::<Vec<_>>();

        assert_eq!(poses.len(), 3);
        assert_eq!(poses.last().map(|pose| pose.frame_id), Some(2));
    }

    #[test]
    fn rejects_invalid_configuration() {
        let cases = [
            MockPoseSource::new(0.0, 2.0, 0.5, None).unwrap_err(),
            MockPoseSource::new(10.0, 0.0, 0.5, None).unwrap_err(),
            MockPoseSource::new(10.0, 2.0, -0.5, None).unwrap_err(),
            MockPoseSource::new(10.0, 2.0, 0.5, Some(0.0)).unwrap_err(),
        ];

        assert_eq!(
            cases,
            [
                MockPoseSourceError::InvalidPoseRate,
                MockPoseSourceError::InvalidRadius,
                MockPoseSourceError::InvalidAngularSpeed,
                MockPoseSourceError::InvalidDuration,
            ]
        );
    }
}
