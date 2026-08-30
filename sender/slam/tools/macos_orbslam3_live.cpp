#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

#include "slam_remote/camera/calibration_io.hpp"
#include "slam_remote/camera/macos_camera_source.hpp"
#include "slam_remote/orbslam3/monocular_tracker.hpp"
#include "slam_remote/telemetry/point_cloud_delta_reducer.hpp"

namespace {

volatile std::sig_atomic_t running = 1;

void HandleSignal(int) { running = 0; }

const char* TrackingStateName(slam_remote::orbslam3::TrackingState state) noexcept {
    using slam_remote::orbslam3::TrackingState;
    switch (state) {
        case TrackingState::kInitializing:
            return "initializing";
        case TrackingState::kTracking:
            return "tracking";
        case TrackingState::kLost:
            return "lost";
        case TrackingState::kRelocalizing:
            return "relocalizing";
    }
    return "unknown";
}

}  // namespace

int main(int argc, char** argv) {
    using namespace std::chrono_literals;
    using slam_remote::camera::FrameStatus;

    if (argc != 4) {
        std::cerr << "usage: macos_orbslam3_live CALIBRATION ORB_SETTINGS ORB_VOCABULARY\n";
        return EXIT_FAILURE;
    }

    try {
        const auto calibration = slam_remote::camera::LoadCalibrationDocument(argv[1]);
        slam_remote::camera::MacosCameraSource camera(
            {calibration.camera.device_id, calibration.camera.width,
             calibration.camera.height, calibration.fps, 2, calibration.camera});
        slam_remote::orbslam3::MonocularTracker tracker({argv[3], argv[2], true});
        slam_remote::telemetry::PointCloudDeltaReducer reducer;

        const auto started = camera.Start();
        if (!started.started) {
            std::cerr << "camera failed to start: " << started.error << '\n';
            return EXIT_FAILURE;
        }

        const auto& info = camera.capture_info();
        std::cout << "live SLAM started device_id=" << info.device_id << " name=" << info.device_name
                  << " mode=" << info.width << 'x' << info.height << '@' << info.fps << '\n';
        std::signal(SIGINT, HandleSignal);

        auto next_cloud_update = std::chrono::steady_clock::now();
        auto next_status = next_cloud_update;
        std::uint64_t processed_frames = 0;
        while (running != 0) {
            const auto frame_result = camera.NextFrame(1s);
            if (frame_result.status == FrameStatus::kTimeout) {
                std::cerr << "camera frame timeout\n";
                continue;
            }
            if (!frame_result.IsValid() || !frame_result.frame.has_value()) {
                std::cerr << "camera stopped: " << frame_result.error << '\n';
                break;
            }

            const auto tracked = tracker.Track(*frame_result.frame);
            ++processed_frames;
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_cloud_update) {
                const auto reduced = reducer.Reduce(tracked.tracked_points);
                std::cout << "cloud frame=" << tracked.frame_id
                          << " retained=" << reduced.stats.selected_points
                          << " add=" << reduced.delta.add.size()
                          << " update=" << reduced.delta.update.size()
                          << " remove=" << reduced.delta.remove.size()
                          << " voxel_filtered=" << reduced.stats.voxel_filtered_points << '\n';
                next_cloud_update = now + 200ms;
            }
            if (now >= next_status) {
                std::cout << "status frame=" << tracked.frame_id
                          << " state=" << TrackingStateName(tracked.state)
                          << " pose=" << (tracked.pose.has_value() ? "available" : "unavailable")
                          << " camera_dropped=" << camera.dropped_frames() << '\n';
                next_status = now + 1s;
            }
        }

        camera.RequestStop();
        camera.Stop();
        tracker.Shutdown();
        std::cout << "live SLAM stopped frames=" << processed_frames
                  << " camera_dropped=" << camera.dropped_frames() << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "live SLAM failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
