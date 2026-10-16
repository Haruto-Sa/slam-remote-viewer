#include "slam_remote/rtabmap/rgbd_tracker.hpp"
#include "slam_remote/boundary/publisher.hpp"

#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
double Number(const char* value) {
    std::size_t used = 0;
    const std::string text(value);
    const double result = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(result)) {
        throw std::invalid_argument("expected a finite numeric argument");
    }
    return result;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc != 12) {
        std::cerr << "Usage: rtabmap_rgbd_sender ASSOCIATIONS SOCKET SESSION CAMERA_ID "
                     "FX FY CX CY DEPTH_UNITS_PER_METRE FPS MAX_FRAMES\n"
                     "Associations: rgb_timestamp rgb_path depth_timestamp depth_path\n"
                     "Paths are relative to the association file. Inputs must be "
                     "registered, rectified BGR and uint16 depth (0 means missing).\n"
                     "Start the Rust sender with --source live --slam-socket SOCKET first.\n";
        return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 2;
    }
    try {
        const std::filesystem::path manifest(argv[1]);
        std::ifstream input(manifest);
        if (!input) throw std::runtime_error("cannot open association file");
        const double fx = Number(argv[5]), fy = Number(argv[6]);
        const double cx = Number(argv[7]), cy = Number(argv[8]);
        const double depth_scale = Number(argv[9]), fps = Number(argv[10]);
        const double limit = Number(argv[11]);
        if (depth_scale <= 0 || fps < 1 || fps > 240 || std::floor(fps) != fps ||
            limit < 1 || limit > 1'000'000 || std::floor(limit) != limit) {
            throw std::invalid_argument("invalid depth scale, FPS or frame limit");
        }
        std::unique_ptr<slam_remote::rtabmap_backend::RgbdTracker> tracker;
        std::unique_ptr<slam_remote::boundary::Publisher> publisher;
        slam_remote::slam::PointCloudDeltaReducer reducer;
        std::uint64_t frame = 0, tracked = 0;
        std::chrono::nanoseconds last_timestamp{};
        double origin = 0, previous_rgb = -1, previous_depth = -1;
        std::string line;
        while (frame < static_cast<std::uint64_t>(limit) && std::getline(input, line)) {
            const auto start = line.find_first_not_of(" \t\r");
            if (start == std::string::npos || line[start] == '#') continue;
            double rgb_time = 0, depth_time = 0;
            std::string rgb_path, depth_path, extra;
            std::istringstream row(line);
            if (!(row >> rgb_time >> rgb_path >> depth_time >> depth_path) || (row >> extra) ||
                !std::isfinite(rgb_time) || !std::isfinite(depth_time) ||
                rgb_time <= previous_rgb || depth_time <= previous_depth ||
                std::abs(rgb_time - depth_time) > 0.02) {
                throw std::runtime_error("invalid, unordered or unsynchronized association row");
            }
            previous_rgb = rgb_time;
            previous_depth = depth_time;
            const cv::Mat color = cv::imread((manifest.parent_path() / rgb_path).string(), cv::IMREAD_COLOR);
            const cv::Mat raw_depth = cv::imread((manifest.parent_path() / depth_path).string(), cv::IMREAD_UNCHANGED);
            if (color.empty() || raw_depth.empty() || raw_depth.type() != CV_16UC1 ||
                color.size() != raw_depth.size()) {
                throw std::runtime_error("missing or incompatible RGB-D image pair");
            }
            cv::Mat depth;
            raw_depth.convertTo(depth, CV_32FC1, 1.0 / depth_scale);
            if (!tracker) {
                origin = rgb_time;
                tracker = std::make_unique<slam_remote::rtabmap_backend::RgbdTracker>(
                    slam_remote::rtabmap_backend::RgbdTrackerConfig{
                        color.cols, color.rows, fx, fy, cx, cy});
                publisher = std::make_unique<slam_remote::boundary::Publisher>(
                    slam_remote::boundary::PublisherConfig{
                        argv[2], argv[3], "rtabmap-rgbd",
                        {argv[4], static_cast<std::uint32_t>(color.cols),
                         static_cast<std::uint32_t>(color.rows), static_cast<std::uint32_t>(fps), "rgbd"},
                        std::chrono::nanoseconds(0)});
                if (!publisher->Connect()) throw std::runtime_error(publisher->last_error());
            }
            const double elapsed = rgb_time - origin;
            if (elapsed > 86400) throw std::runtime_error("session duration exceeds one day");
            const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(elapsed));
            last_timestamp = timestamp;
            const auto result = tracker->Track(frame, timestamp, color, depth);
            if (!publisher->PublishTracking(result)) throw std::runtime_error(publisher->last_error());
            if (result.pose) ++tracked;
            // Export at approximately 1 Hz. All current graph nodes participate;
            // loop-closure corrections become updates to stable observation IDs.
            if (frame % static_cast<std::uint64_t>(fps) == 0) {
                const auto delta = reducer.Reduce(tracker->MapPoints());
                if (delta.operation_count() && !publisher->PublishPointCloud(frame, timestamp, delta)) {
                    throw std::runtime_error(publisher->last_error());
                }
            }
            ++frame;
        }
        if (!publisher || !frame) throw std::runtime_error("association file contains no frames");
        if (input.bad()) throw std::runtime_error("association read failed");
        const auto final_delta = reducer.Reduce(tracker->MapPoints());
        if (final_delta.operation_count() &&
            !publisher->PublishPointCloud(frame - 1, last_timestamp, final_delta)) {
            throw std::runtime_error(publisher->last_error());
        }
        if (!publisher->EndSession()) throw std::runtime_error(publisher->last_error());
        std::cout << "frames=" << frame << " tracked=" << tracked
                  << " exported_points=" << reducer.retained_points() << '\n';
        return tracked ? 0 : 1;
    } catch (const std::exception& error) {
        // Closing without session_end reports an abnormal session to Rust.
        std::cerr << "RTAB-Map sender failed: " << error.what() << '\n';
        return 1;
    }
}
