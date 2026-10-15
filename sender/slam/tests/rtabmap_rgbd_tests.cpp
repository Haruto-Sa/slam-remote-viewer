#include "slam_remote/rtabmap/rgbd_tracker.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

using slam_remote::rtabmap_backend::RgbdTracker;
using slam_remote::rtabmap_backend::RgbdTrackerConfig;

template <typename F> void Reject(F function) {
    try { function(); }
    catch (const std::invalid_argument&) { return; }
    throw std::runtime_error("invalid input was accepted");
}

int main() {
    try {
        const RgbdTrackerConfig config{640, 480, 525, 525, 319.5, 239.5};
        auto invalid = config;
        invalid.fx = 0;
        Reject([&] { RgbdTracker bad(invalid); });
        invalid = config;
        invalid.cx = std::numeric_limits<double>::quiet_NaN();
        Reject([&] { RgbdTracker bad(invalid); });
        RgbdTracker tracker(config);
        const auto time = std::chrono::nanoseconds(0);
        cv::Mat color(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat depth(480, 640, CV_32FC1, cv::Scalar(0));
        Reject([&] { tracker.Track(0, time, cv::Mat(), depth); });
        Reject([&] { tracker.Track(0, time, color, color); });
        depth.at<float>(0, 0) = std::numeric_limits<float>::infinity();
        Reject([&] { tracker.Track(0, time, color, depth); });
        depth.at<float>(0, 0) = -1;
        Reject([&] { tracker.Track(0, time, color, depth); });
        depth.at<float>(0, 0) = 0;
        const auto result = tracker.Track(0, time, color, depth);
        if (result.pose || result.state != slam_remote::slam::TrackingState::kInitializing) {
            throw std::runtime_error("blank RGB-D frame fabricated a tracked pose");
        }
        Reject([&] { tracker.Track(0, time, color, depth); });
        if (!tracker.MapPoints().empty()) throw std::runtime_error("empty input created map points");
        std::cout << "RTAB-Map RGB-D validation tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
