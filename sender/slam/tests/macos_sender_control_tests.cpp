#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "slam_remote/launcher/macos_sender_control.hpp"

namespace {

using slam_remote::launcher::BuildLauncherArguments;
using slam_remote::launcher::ControlState;
using slam_remote::launcher::FindSavedCameraDevice;
using slam_remote::launcher::SenderControlModel;
using slam_remote::launcher::SenderLaunchConfig;

void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

SenderLaunchConfig Config() {
    return {"/app/launcher", "/app/streamer", "/app/producer", "/data/vocab", "/data/camera.yaml",
            "device", 640, 480, 30, "/private/tmp/test.sock", "tcp://*:5555", "session",
            "camera", 30};
}

void TestArguments() {
    const auto arguments = BuildLauncherArguments(Config());
    Check(arguments.front() == "--streamer" && arguments[1] == "/app/streamer",
          "streamer path must remain a separate argument");
    Check(arguments[arguments.size() - 2] == "--pointcloud-period" && arguments.back() == "30",
          "point-cloud period must be forwarded");
    Check(arguments.size() == 26, "launcher command must contain every configured option");
}

void TestStateTransitions() {
    SenderControlModel model;
    std::string error;
    Check(model.CanStart() && !model.CanStop(), "idle controls must allow only Start");
    Check(model.RequestStart(Config(), error), "valid configuration must start");
    Check(!model.CanStart() && model.CanStop(), "duplicate Start must be disabled");
    model.MarkRunning();
    Check(model.state() == ControlState::kRunning, "launch must enter running state");
    model.RequestStop();
    Check(model.state() == ControlState::kStopping && !model.CanStop(),
          "Stop must be idempotent while shutdown is pending");
    model.ProcessExited(0);
    Check(model.state() == ControlState::kSucceeded && model.CanStart(),
          "clean exit must allow another session");
    Check(model.RequestStart(Config(), error), "restart must be allowed");
    model.ProcessExited(1);
    Check(model.state() == ControlState::kFailed && model.CanStart(),
          "failure must be visible and recoverable");
}

void TestValidation() {
    auto config = Config();
    config.device_id.clear();
    SenderControlModel model;
    std::string error;
    Check(!model.RequestStart(config, error) && error == "device ID must not be empty",
          "invalid input must not start a process");
    Check(model.state() == ControlState::kIdle, "validation failure must remain idle");

    config = Config();
    config.launcher_path = "sender/streamer/target/debug/macos_live_sender";
    Check(!model.RequestStart(config, error) && error == "launcher path must be an absolute path",
          "Finder launches must reject working-directory-dependent paths");
}

void TestSavedCameraLookup() {
    const std::vector<std::string> devices{"built-in", "continuity-camera"};
    Check(FindSavedCameraDevice(devices, "continuity-camera") == 1,
          "a discovered saved camera must retain its stable ID");
    Check(!FindSavedCameraDevice(devices, "disconnected-camera").has_value(),
          "a stale saved camera ID must be detectable");
    Check(!FindSavedCameraDevice({}, "built-in").has_value(),
          "an empty discovery result must not select a camera");
}

}  // namespace

int main() {
    try {
        TestArguments();
        TestStateTransitions();
        TestValidation();
        TestSavedCameraLookup();
    } catch (const std::exception& error) {
        std::cerr << "macOS sender control test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "macOS sender control tests passed\n";
    return EXIT_SUCCESS;
}
