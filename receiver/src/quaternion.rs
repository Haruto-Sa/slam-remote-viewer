use std::fmt;

use crate::protocol::PoseMessage;

pub const MIN_QUATERNION_NORM: f64 = 1.0e-12;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QuaternionError {
    NonFiniteComponent,
    NormTooSmall,
}

impl fmt::Display for QuaternionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFiniteComponent => {
                write!(formatter, "quaternion contains a non-finite component")
            }
            Self::NormTooSmall => {
                write!(
                    formatter,
                    "quaternion norm must be at least {MIN_QUATERNION_NORM}"
                )
            }
        }
    }
}

impl std::error::Error for QuaternionError {}

pub fn normalize(quaternion: [f64; 4]) -> Result<[f64; 4], QuaternionError> {
    if quaternion.iter().any(|component| !component.is_finite()) {
        return Err(QuaternionError::NonFiniteComponent);
    }

    let scale = quaternion
        .iter()
        .map(|component| component.abs())
        .fold(0.0_f64, f64::max);

    if scale == 0.0 {
        return Err(QuaternionError::NormTooSmall);
    }

    let scaled = quaternion.map(|component| component / scale);

    let scaled_norm = scaled
        .iter()
        .map(|component| component * component)
        .sum::<f64>()
        .sqrt();

    if scale < MIN_QUATERNION_NORM / scaled_norm {
        return Err(QuaternionError::NormTooSmall);
    }

    Ok(scaled.map(|component| component / scaled_norm))
}

#[derive(Debug, Default)]
pub struct QuaternionContinuity {
    session: Option<String>,
    previous: Option<[f64; 4]>,
}

impl QuaternionContinuity {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn normalize_for_session(
        &mut self,
        session: &str,
        quaternion: [f64; 4],
    ) -> Result<[f64; 4], QuaternionError> {
        let mut normalized = normalize(quaternion)?;

        if self.session.as_deref() != Some(session) {
            self.session = Some(session.to_owned());
            self.previous = None;
        }

        if let Some(previous) = self.previous
            && dot(previous, normalized) < 0.0
        {
            normalized = normalized.map(|component| -component);
        }

        self.previous = Some(normalized);

        Ok(normalized)
    }

    pub fn normalize_pose(&mut self, pose: &mut PoseMessage) -> Result<(), QuaternionError> {
        let normalized = self.normalize_for_session(&pose.session, pose.q)?;
        pose.q = normalized;

        Ok(())
    }
}

fn dot(left: [f64; 4], right: [f64; 4]) -> f64 {
    left.into_iter()
        .zip(right)
        .map(|(left, right)| left * right)
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1.0e-12;
    const POSE_EXAMPLE: &str = include_str!("../../protocol/pose.example.json");

    fn assert_quaternion_close(actual: [f64; 4], expected: [f64; 4]) {
        for (index, (actual, expected)) in actual.into_iter().zip(expected).enumerate() {
            assert!(
                (actual - expected).abs() <= TOLERANCE,
                "component {index}: expected {expected}, received {actual}"
            );
        }
    }

    #[test]
    fn normalizes_quaternion_to_unit_length() {
        let normalized = normalize([0.0, 0.0, 0.0, 2.0]).expect("quaternion should normalize");

        assert_quaternion_close(normalized, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn preserves_xyzw_component_order() {
        let normalized = normalize([1.0, 2.0, 3.0, 4.0]).expect("quaternion should normalize");

        let norm = 30.0_f64.sqrt();

        assert_quaternion_close(normalized, [1.0 / norm, 2.0 / norm, 3.0 / norm, 4.0 / norm]);
    }

    #[test]
    fn rejects_zero_and_near_zero_quaternions() {
        assert_eq!(
            normalize([0.0, 0.0, 0.0, 0.0]),
            Err(QuaternionError::NormTooSmall)
        );

        assert_eq!(
            normalize([1.0e-15, 0.0, 0.0, 0.0]),
            Err(QuaternionError::NormTooSmall)
        );
    }

    #[test]
    fn rejects_non_finite_components() {
        assert_eq!(
            normalize([0.0, f64::NAN, 0.0, 1.0]),
            Err(QuaternionError::NonFiniteComponent)
        );

        assert_eq!(
            normalize([0.0, 0.0, f64::INFINITY, 1.0]),
            Err(QuaternionError::NonFiniteComponent)
        );
    }

    #[test]
    fn normalizes_large_finite_components_without_overflow() {
        let normalized =
            normalize([1.0e308, 0.0, 0.0, 1.0e308]).expect("large quaternion should normalize");

        assert_quaternion_close(
            normalized,
            [
                std::f64::consts::FRAC_1_SQRT_2,
                0.0,
                0.0,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
        );
    }

    #[test]
    fn continuity_tracker_normalizes_first_quaternion() {
        let mut tracker = QuaternionContinuity::new();

        let normalized = tracker
            .normalize_for_session("session-a", [0.0, 0.0, 0.0, 2.0])
            .expect("quaternion should normalize");

        assert_quaternion_close(normalized, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn negates_equivalent_quaternion_to_preserve_continuity() {
        let mut tracker = QuaternionContinuity::new();

        let previous = tracker
            .normalize_for_session("session-a", [0.0, 0.0, 0.0, 2.0])
            .expect("first quaternion should normalize");

        let current = tracker
            .normalize_for_session("session-a", [0.0, 0.0, 0.0, -3.0])
            .expect("second quaternion should normalize");

        assert_quaternion_close(current, [0.0, 0.0, 0.0, 1.0]);

        assert!(dot(previous, current) >= 0.0);
    }

    #[test]
    fn resets_continuity_reference_when_session_changes() {
        let mut tracker = QuaternionContinuity::new();

        tracker
            .normalize_for_session("session-a", [0.0, 0.0, 0.0, 1.0])
            .expect("first quaternion should normalize");

        let first_in_new_session = tracker
            .normalize_for_session("session-b", [0.0, 0.0, 0.0, -2.0])
            .expect("new-session quaternion should normalize");

        assert_quaternion_close(first_in_new_session, [0.0, 0.0, 0.0, -1.0]);
    }

    #[test]
    fn invalid_quaternion_does_not_replace_previous_value() {
        let mut tracker = QuaternionContinuity::new();

        tracker
            .normalize_for_session("session-a", [0.0, 0.0, 0.0, 1.0])
            .expect("first quaternion should normalize");

        assert_eq!(
            tracker.normalize_for_session("session-a", [0.0, 0.0, 0.0, 0.0]),
            Err(QuaternionError::NormTooSmall)
        );

        let recovered = tracker
            .normalize_for_session("session-a", [0.0, 0.0, 0.0, -1.0])
            .expect("valid quaternion should normalize");

        assert_quaternion_close(recovered, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn normalizes_pose_message_in_place() {
        let mut tracker = QuaternionContinuity::new();
        let mut pose: PoseMessage =
            serde_json::from_str(POSE_EXAMPLE).expect("pose example should deserialize");
        pose.q = [0.0, 0.0, 0.0, 2.0];

        tracker
            .normalize_pose(&mut pose)
            .expect("pose quaternion should normalize");

        assert_quaternion_close(pose.q, [0.0, 0.0, 0.0, 1.0]);
    }
}
