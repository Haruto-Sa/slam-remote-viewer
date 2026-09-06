#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace slam_remote::launcher {

struct SenderLaunchConfig final {
    std::string launcher_path;
    std::string streamer_path;
    std::string producer_path;
    std::string vocabulary_path;
    std::string settings_path;
    std::string device_id;
    std::uint32_t width{640};
    std::uint32_t height{480};
    std::uint32_t fps{30};
    std::string socket_path{"/private/tmp/slam-live.sock"};
    std::string endpoint{"tcp://*:5555"};
    std::string session{"live-session"};
    std::string camera_id{"mac-camera"};
    std::uint32_t pointcloud_period{30};
};

enum class ControlState { kIdle, kStarting, kRunning, kStopping, kSucceeded, kFailed };

std::vector<std::string> BuildLauncherArguments(const SenderLaunchConfig& config);
std::string ValidateLaunchConfig(const SenderLaunchConfig& config);
using LaunchPathProbe = std::function<bool(const std::string&, bool)>;
std::string ValidateLaunchPaths(const SenderLaunchConfig& config, const LaunchPathProbe& probe);
std::optional<std::size_t> FindSavedCameraDevice(
    const std::vector<std::string>& discovered_device_ids, const std::string& saved_device_id);
const char* ControlStateName(ControlState state);

class SenderControlModel final {
   public:
    [[nodiscard]] ControlState state() const noexcept { return state_; }
    [[nodiscard]] bool CanStart() const noexcept;
    [[nodiscard]] bool CanStop() const noexcept;
    bool RequestStart(const SenderLaunchConfig& config, std::string& error);
    void MarkRunning() noexcept;
    void RequestStop() noexcept;
    void ProcessExited(int exit_code) noexcept;
    void LaunchFailed() noexcept;

   private:
    ControlState state_{ControlState::kIdle};
};

}  // namespace slam_remote::launcher
