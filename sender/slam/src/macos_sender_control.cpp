#include "slam_remote/launcher/macos_sender_control.hpp"

#include <array>
#include <utility>

namespace slam_remote::launcher {
namespace {

bool Empty(const std::string& value) { return value.find_first_not_of(" \t\r\n") == std::string::npos; }

}  // namespace

std::string ValidateLaunchConfig(const SenderLaunchConfig& config) {
    const std::array<std::pair<const std::string*, const char*>, 10> required{{
        {&config.launcher_path, "launcher path"},
        {&config.streamer_path, "streamer path"},
        {&config.producer_path, "producer path"},
        {&config.vocabulary_path, "vocabulary path"},
        {&config.settings_path, "settings path"},
        {&config.device_id, "device ID"},
        {&config.socket_path, "socket path"},
        {&config.endpoint, "endpoint"},
        {&config.session, "session"},
        {&config.camera_id, "camera ID"},
    }};
    for (const auto& [value, name] : required) {
        if (Empty(*value)) return std::string(name) + " must not be empty";
    }
    const std::array<std::pair<const std::string*, const char*>, 5> file_paths{{
        {&config.launcher_path, "launcher path"},
        {&config.streamer_path, "streamer path"},
        {&config.producer_path, "producer path"},
        {&config.vocabulary_path, "vocabulary path"},
        {&config.settings_path, "settings path"},
    }};
    for (const auto& [value, name] : file_paths) {
        if (value->front() != '/') return std::string(name) + " must be an absolute path";
    }
    if (config.width == 0 || config.height == 0 || config.fps == 0 ||
        config.pointcloud_period == 0) {
        return "width, height, FPS, and point-cloud period must be positive";
    }
    return {};
}

std::vector<std::string> BuildLauncherArguments(const SenderLaunchConfig& config) {
    return {"--streamer",
            config.streamer_path,
            "--producer",
            config.producer_path,
            "--vocabulary",
            config.vocabulary_path,
            "--settings",
            config.settings_path,
            "--device-id",
            config.device_id,
            "--width",
            std::to_string(config.width),
            "--height",
            std::to_string(config.height),
            "--fps",
            std::to_string(config.fps),
            "--slam-socket",
            config.socket_path,
            "--endpoint",
            config.endpoint,
            "--session",
            config.session,
            "--camera-id",
            config.camera_id,
            "--pointcloud-period",
            std::to_string(config.pointcloud_period)};
}

const char* ControlStateName(ControlState state) {
    switch (state) {
        case ControlState::kIdle:
            return "Idle";
        case ControlState::kStarting:
            return "Starting";
        case ControlState::kRunning:
            return "Running";
        case ControlState::kStopping:
            return "Stopping";
        case ControlState::kSucceeded:
            return "Stopped cleanly";
        case ControlState::kFailed:
            return "Failed";
    }
    return "Unknown";
}

bool SenderControlModel::CanStart() const noexcept {
    return state_ == ControlState::kIdle || state_ == ControlState::kSucceeded ||
           state_ == ControlState::kFailed;
}

bool SenderControlModel::CanStop() const noexcept {
    return state_ == ControlState::kStarting || state_ == ControlState::kRunning;
}

bool SenderControlModel::RequestStart(const SenderLaunchConfig& config, std::string& error) {
    if (!CanStart()) {
        error = "a live sender session is already active";
        return false;
    }
    error = ValidateLaunchConfig(config);
    if (!error.empty()) return false;
    state_ = ControlState::kStarting;
    return true;
}

void SenderControlModel::MarkRunning() noexcept {
    if (state_ == ControlState::kStarting) state_ = ControlState::kRunning;
}

void SenderControlModel::RequestStop() noexcept {
    if (CanStop()) state_ = ControlState::kStopping;
}

void SenderControlModel::ProcessExited(int exit_code) noexcept {
    state_ = exit_code == 0 ? ControlState::kSucceeded : ControlState::kFailed;
}

void SenderControlModel::LaunchFailed() noexcept { state_ = ControlState::kFailed; }

}  // namespace slam_remote::launcher
