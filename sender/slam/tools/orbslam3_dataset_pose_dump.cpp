#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "slam_remote/camera/image_frame.hpp"
#include "slam_remote/orbslam3/monocular_tracker.hpp"

namespace {

struct DatasetFrame final {
    double timestamp_seconds;
    std::string relative_path;
};

std::vector<DatasetFrame> LoadTumIndex(const std::string& sequence_path) {
    std::ifstream input(sequence_path + "/rgb.txt");
    if (!input) {
        throw std::runtime_error("cannot open TUM rgb.txt");
    }
    std::vector<DatasetFrame> frames;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#') {
            continue;
        }
        std::istringstream fields(line);
        DatasetFrame frame{};
        if (!(fields >> frame.timestamp_seconds >> frame.relative_path) ||
            !std::isfinite(frame.timestamp_seconds) || frame.timestamp_seconds < 0.0) {
            throw std::runtime_error("invalid TUM rgb.txt entry");
        }
        frames.push_back(std::move(frame));
    }
    if (frames.empty()) {
        throw std::runtime_error("TUM rgb.txt contains no frames");
    }
    return frames;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: orbslam3_dataset_pose_dump VOCABULARY SETTINGS TUM_SEQUENCE\n";
        return 2;
    }
    try {
        const std::string sequence_path = argv[3];
        const auto frames = LoadTumIndex(sequence_path);
        slam_remote::orbslam3::MonocularTracker tracker({argv[1], argv[2], false});
        std::size_t tracked_frames = 0;
        std::size_t lost_frames = 0;
        for (std::size_t index = 0; index < frames.size(); ++index) {
            const auto& dataset_frame = frames[index];
            cv::Mat image = cv::imread(sequence_path + "/" + dataset_frame.relative_path,
                                       cv::IMREAD_GRAYSCALE);
            if (image.empty() || !image.isContinuous()) {
                throw std::runtime_error("cannot load contiguous dataset image");
            }
            std::vector<std::uint8_t> pixels(image.datastart, image.dataend);
            const auto timestamp = std::chrono::nanoseconds(
                std::llround(dataset_frame.timestamp_seconds * 1'000'000'000.0));
            slam_remote::camera::ImageFrame frame(
                index, timestamp, static_cast<std::uint32_t>(image.cols),
                static_cast<std::uint32_t>(image.rows),
                slam_remote::camera::PixelFormat::kGray8, std::move(pixels));
            const auto result = tracker.Track(frame);
            if (result.frame_id != index || result.timestamp != timestamp) {
                throw std::runtime_error("adapter did not preserve frame metadata");
            }
            if (result.pose.has_value()) {
                ++tracked_frames;
            } else if (result.state == slam_remote::slam::TrackingState::kLost) {
                ++lost_frames;
            }
        }
        tracker.Shutdown();
        if (tracked_frames == 0) {
            throw std::runtime_error("dataset never reached valid tracking");
        }
        std::cout << "ORB-SLAM3 pose adapter replay passed: frames=" << frames.size()
                  << " tracked=" << tracked_frames << " lost=" << lost_frames << '\n';
    } catch (const std::exception& error) {
        std::cerr << "ORB-SLAM3 pose adapter replay failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
