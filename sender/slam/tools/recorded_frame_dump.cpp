#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "slam_remote/camera/recorded_frame_source.hpp"

namespace {

std::uint32_t ParsePositive(const char* value, const char* name) {
    try {
        std::size_t consumed = 0;
        const auto parsed = std::stoul(value, &consumed, 10);
        if (consumed != std::string(value).size() || parsed == 0 ||
            parsed > std::numeric_limits<std::uint32_t>::max()) {
            throw std::out_of_range("range");
        }
        return static_cast<std::uint32_t>(parsed);
    } catch (const std::exception&) {
        std::cerr << name << " must be a positive 32-bit integer\n";
        std::exit(EXIT_FAILURE);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "usage: recorded_frame_dump WIDTH HEIGHT FPS FRAME.pgm [FRAME.pgm ...]\n";
        return EXIT_FAILURE;
    }

    const auto width = ParsePositive(argv[1], "WIDTH");
    const auto height = ParsePositive(argv[2], "HEIGHT");
    const auto fps = ParsePositive(argv[3], "FPS");
    std::vector<std::filesystem::path> paths;
    for (int index = 4; index < argc; ++index) {
        paths.emplace_back(argv[index]);
    }

    slam_remote::camera::CameraCalibration calibration{
        "recorded-sequence", width, height, slam_remote::camera::CameraModel::kPinhole,
        static_cast<double>(width), static_cast<double>(height),
        static_cast<double>(width) / 2.0, static_cast<double>(height) / 2.0,
        {0.0, 0.0, 0.0, 0.0}};
    slam_remote::camera::RecordedFrameSource source(
        {std::move(calibration), std::move(paths), std::chrono::nanoseconds(1'000'000'000 / fps)});

    const auto start = source.Start();
    if (!start.started) {
        std::cerr << "recorded source failed to start: " << start.error << '\n';
        return EXIT_FAILURE;
    }

    while (true) {
        const auto result = source.NextFrame(std::chrono::milliseconds(1));
        if (result.status == slam_remote::camera::FrameStatus::kEndOfStream) {
            break;
        }
        if (!result.IsValid() || !result.frame.has_value()) {
            std::cerr << "frame rejected: " << result.error << '\n';
            continue;
        }
        const auto& frame = *result.frame;
        std::cout << "frame=" << frame.frame_id() << " timestamp_ns=" << frame.timestamp().count()
                  << " size=" << frame.width() << 'x' << frame.height()
                  << " bytes=" << frame.pixels().size() << '\n';
    }
    source.Stop();
    return EXIT_SUCCESS;
}
