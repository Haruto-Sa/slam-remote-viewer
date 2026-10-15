#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "slam_remote/boundary/publisher.hpp"
#include "slam_remote/camera/streamed_frame_reader.hpp"
#include "slam_remote/orbslam3/monocular_tracker.hpp"
#include "slam_remote/slam/point_cloud_delta_reducer.hpp"

namespace {
std::uint32_t Setting(const cv::FileStorage& settings, const char* name,
                      std::uint32_t maximum) {
    const auto node = settings[name];
    if (node.empty() || (!node.isInt() && !node.isReal())) {
        throw std::runtime_error(std::string("missing numeric setting: ") + name);
    }
    const double value = node.real();
    if (!std::isfinite(value) || value < 1 || value > maximum || std::floor(value) != value) {
        throw std::runtime_error(std::string("invalid integer setting: ") + name);
    }
    return static_cast<std::uint32_t>(value);
}
}  // namespace

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "usage: orbslam3_stream_camera_sender VOCABULARY SETTINGS"
                     " SOCKET SESSION CAMERA_ID < camera.slf\n";
        return 2;
    }
    try {
        cv::FileStorage settings(argv[2], cv::FileStorage::READ);
        if (!settings.isOpened()) throw std::runtime_error("cannot open camera settings");
        const auto width = Setting(settings, "Camera.width", 1920);
        const auto height = Setting(settings, "Camera.height", 1920);
        const auto fps = Setting(settings, "Camera.fps", 240);
        if (!settings["Camera.newWidth"].empty() || !settings["Camera.newHeight"].empty()) {
            throw std::runtime_error("stream calibration must use the original capture dimensions");
        }
        slam_remote::camera::StreamedFrameReader input(std::cin, width, height);
        auto frame = input.Next();
        if (!frame) throw std::runtime_error("camera stream ended before its first frame");
        slam_remote::boundary::Publisher publisher({
            argv[3], argv[4], "orbslam3-monocular-stream", {argv[5], width, height, fps},
            frame->timestamp(), std::chrono::milliseconds(250)});
        if (!publisher.Connect()) throw std::runtime_error(publisher.last_error());
        slam_remote::orbslam3::MonocularTracker tracker({argv[1], argv[2], false});
        slam_remote::slam::PointCloudDeltaReducer reducer;
        std::uint64_t frames = 0, poses = 0, deltas = 0;
        while (frame) {
            const auto result = tracker.Track(*frame);
            if (!publisher.PublishTracking(result)) throw std::runtime_error(publisher.last_error());
            if (result.pose) ++poses;
            if (frames % fps == 0) {
                const auto map = tracker.ActiveMap();
                const auto delta = reducer.Reduce(map.points);
                if (delta.operation_count() > 0) {
                    if (!publisher.PublishPointCloud(result.frame_id, result.timestamp, delta)) {
                        throw std::runtime_error(publisher.last_error());
                    }
                    ++deltas;
                }
            }
            ++frames;
            frame = input.Next();
        }
        tracker.Shutdown();
        if (!publisher.EndSession("camera_stream_eof")) {
            throw std::runtime_error(publisher.last_error());
        }
        std::cout << "stream camera ended: frames=" << frames << " poses=" << poses
                  << " pointcloud_deltas=" << deltas << '\n';
    } catch (const std::exception& error) {
        std::cerr << "stream camera failed: " << error.what() << '\n';
        return 1;
    }
}
