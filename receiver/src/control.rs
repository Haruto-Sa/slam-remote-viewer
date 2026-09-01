use serde::{Deserialize, Serialize};

use crate::clip::{ClipRecorder, ClipStatus};

#[derive(Debug, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case", deny_unknown_fields)]
enum ClipControlCommand {
    StartClip,
    StopClip,
    Status,
}

#[derive(Debug, Serialize)]
pub struct ClipControlResponse {
    pub ok: bool,
    #[serde(flatten)]
    pub status: ClipStatus,
}

pub fn handle_control_request(payload: &[u8], recorder: &ClipRecorder) -> ClipControlResponse {
    let command: ClipControlCommand = match serde_json::from_slice(payload) {
        Ok(command) => command,
        Err(error) => return failure_response(recorder.status(), error.to_string()),
    };

    let result = match command {
        ClipControlCommand::StartClip => recorder.start_clip(),
        ClipControlCommand::StopClip => recorder.stop_clip(),
        ClipControlCommand::Status => return success_response(recorder.status()),
    };

    match result {
        Ok(()) => success_response(recorder.status()),
        Err(error) => failure_response(recorder.status(), error.to_string()),
    }
}

fn success_response(status: ClipStatus) -> ClipControlResponse {
    ClipControlResponse { ok: true, status }
}

fn failure_response(mut status: ClipStatus, error: String) -> ClipControlResponse {
    status.error = Some(error);
    ClipControlResponse { ok: false, status }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;
    use crate::clip::ClipState;

    #[test]
    fn returns_current_status_and_rejects_invalid_commands() {
        let recorder = ClipRecorder::start(PathBuf::from("unused-control-test-output"));

        let status = handle_control_request(br#"{"command":"status"}"#, &recorder);
        assert!(status.ok);
        assert_eq!(status.status.state, ClipState::Idle);

        let invalid = handle_control_request(br#"{"command":"unknown"}"#, &recorder);
        assert!(!invalid.ok);
        assert!(invalid.status.error.unwrap().contains("unknown variant"));

        recorder.finish().expect("worker should finish");
    }
}
