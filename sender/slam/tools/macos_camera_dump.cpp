#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "slam_remote/camera/macos_camera_source.hpp"

namespace {

volatile std::sig_atomic_t running = 1;

void HandleSignal(int) { running = 0; }

std::uint32_t ParsePositive(const char* value, const char* name) {
    try {
        std::size_t consumed = 0;
        const std::string text(value);
        const auto parsed = std::stoul(text, &consumed, 10);
        if (consumed != text.size() || parsed == 0 ||
            parsed > std::numeric_limits<std::uint32_t>::max()) {
            throw std::out_of_range("range");
        }
        return static_cast<std::uint32_t>(parsed);
    } catch (const std::exception&) {
        std::cerr << name << " must be a positive 32-bit integer\n";
        std::exit(EXIT_FAILURE);
    }
}

const char* AuthorizationName(slam_remote::camera::CameraAuthorization status) {
    using slam_remote::camera::CameraAuthorization;
    switch (status) {
        case CameraAuthorization::kAuthorized:
            return "authorized";
        case CameraAuthorization::kDenied:
            return "denied";
        case CameraAuthorization::kRestricted:
            return "restricted";
        case CameraAuthorization::kNotDetermined:
            return "not-determined";
    }
    return "unknown";
}

}  // namespace

int main(int argc, char** argv) {
    using namespace std::chrono_literals;
    using namespace slam_remote::camera;

    if (argc == 2 && std::string(argv[1]) == "--list") {
        std::cout << "authorization=" << AuthorizationName(GetCameraAuthorization()) << '\n';
        for (const auto& device : ListMacosCameraDevices()) {
            std::cout << "device_id=" << device.unique_id << " name=" << device.name << '\n';
        }
        return EXIT_SUCCESS;
    }
    if (argc == 2 && std::string(argv[1]) == "--request-permission") {
        const bool granted = RequestCameraAuthorization();
        std::cout << "authorization=" << AuthorizationName(GetCameraAuthorization()) << '\n';
        return granted ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    if (argc != 6) {
        std::cerr << "usage:\n"
                  << "  macos_camera_dump --list\n"
                  << "  macos_camera_dump --request-permission\n"
                  << "  macos_camera_dump DEVICE_ID WIDTH HEIGHT FPS FRAME_COUNT\n";
        return EXIT_FAILURE;
    }

    const std::string device_id(argv[1]);
    const auto width = ParsePositive(argv[2], "WIDTH");
    const auto height = ParsePositive(argv[3], "HEIGHT");
    const auto fps = ParsePositive(argv[4], "FPS");
    const auto frame_limit = ParsePositive(argv[5], "FRAME_COUNT");
    CameraCalibration placeholder{device_id,
                                  width,
                                  height,
                                  CameraModel::kPinhole,
                                  static_cast<double>(width),
                                  static_cast<double>(height),
                                  static_cast<double>(width) / 2.0,
                                  static_cast<double>(height) / 2.0,
                                  {0.0, 0.0, 0.0, 0.0}};
    MacosCameraSource source({device_id, width, height, fps, 4, std::move(placeholder)});
    const auto started = source.Start();
    if (!started.started) {
        std::cerr << "camera failed to start: " << started.error << '\n';
        return EXIT_FAILURE;
    }

    const auto& info = source.capture_info();
    std::cout << "started device_id=" << info.device_id << " name=" << info.device_name
              << " mode=" << info.width << 'x' << info.height << '@' << info.fps << '\n';
    std::signal(SIGINT, HandleSignal);
    std::uint32_t received = 0;
    while (running != 0 && received < frame_limit) {
        const auto result = source.NextFrame(1s);
        if (result.status == FrameStatus::kTimeout) {
            std::cerr << "camera frame timeout\n";
            continue;
        }
        if (!result.IsValid() || !result.frame.has_value()) {
            std::cerr << "camera stopped: " << result.error << '\n';
            break;
        }
        const auto& frame = *result.frame;
        std::cout << "frame=" << frame.frame_id() << " timestamp_ns=" << frame.timestamp().count()
                  << " size=" << frame.width() << 'x' << frame.height()
                  << " dropped=" << source.dropped_frames() << '\n';
        ++received;
    }
    source.RequestStop();
    source.Stop();
    return received == frame_limit ? EXIT_SUCCESS : EXIT_FAILURE;
}
