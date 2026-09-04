#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include "slam_remote/boundary/publisher.hpp"
#include "slam_remote/camera/macos_camera_source.hpp"
#include "slam_remote/orbslam3/monocular_tracker.hpp"
#include "slam_remote/slam/point_cloud_delta_reducer.hpp"

namespace {
volatile std::sig_atomic_t running = 1;
void HandleSignal(int) { running = 0; }
constexpr auto kInterruptGracePeriod = std::chrono::seconds(5);

class InterruptWatchdog final {
   public:
    InterruptWatchdog() : worker_([this] { Run(); }) {}
    ~InterruptWatchdog() { Complete(); }
    InterruptWatchdog(const InterruptWatchdog&) = delete;
    InterruptWatchdog& operator=(const InterruptWatchdog&) = delete;

    void Complete() noexcept {
        complete_.store(true);
        if (worker_.joinable()) worker_.join();
    }

   private:
    void Run() {
        while (running != 0 && !complete_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (running != 0 || complete_.load()) return;
        const auto deadline = std::chrono::steady_clock::now() + kInterruptGracePeriod;
        while (!complete_.load() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!complete_.load()) {
            std::cerr << "forced exit: ORB-SLAM3 did not return within 5 seconds of Ctrl-C\n";
            std::cerr.flush();
            std::_Exit(130);
        }
    }

    std::atomic<bool> complete_{false};
    std::thread worker_;
};

std::uint32_t Positive(const char* value, const char* name) {
    try {
        std::size_t consumed = 0;
        const auto parsed = std::stoul(value, &consumed, 10);
        if (consumed != std::string(value).size() || parsed == 0 ||
            parsed > std::numeric_limits<std::uint32_t>::max()) {
            throw std::out_of_range("range");
        }
        return static_cast<std::uint32_t>(parsed);
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string(name) + " must be a positive uint32");
    }
}
}  // namespace

int main(int argc, char** argv) {
    using namespace std::chrono_literals;
    if (argc != 11 && argc != 12) {
        std::cerr << "usage: orbslam3_macos_camera_sender VOCABULARY SETTINGS DEVICE_ID WIDTH "
                     "HEIGHT FPS SOCKET SESSION CAMERA_ID FRAME_LIMIT "
                     "[POINTCLOUD_PERIOD_FRAMES]\n";
        return EXIT_FAILURE;
    }
    try {
        const auto width = Positive(argv[4], "WIDTH");
        const auto height = Positive(argv[5], "HEIGHT");
        const auto fps = Positive(argv[6], "FPS");
        const auto frame_limit = Positive(argv[10], "FRAME_LIMIT");
        const auto pointcloud_period =
            argc == 12 ? Positive(argv[11], "POINTCLOUD_PERIOD_FRAMES") : 30;
        slam_remote::camera::CameraCalibration calibration{
            argv[3], width, height, slam_remote::camera::CameraModel::kPinhole,
            static_cast<double>(width), static_cast<double>(height),
            static_cast<double>(width) / 2.0, static_cast<double>(height) / 2.0,
            {0.0, 0.0, 0.0, 0.0}};
        slam_remote::orbslam3::MonocularTracker tracker({argv[1], argv[2], false});
        slam_remote::camera::MacosCameraSource source(
            {argv[3], width, height, fps, 4, std::move(calibration)});
        const auto started = source.Start();
        if (!started.started) throw std::runtime_error(started.error);

        const auto first = source.NextFrame(2s);
        if (!first.IsValid() || !first.frame) {
            source.Stop();
            throw std::runtime_error("camera did not provide the first frame: " + first.error);
        }
        const auto& capture = source.capture_info();
        slam_remote::boundary::Publisher publisher({
            argv[7], argv[8], "orbslam3-macos-monocular",
            {argv[9], capture.width, capture.height, capture.fps}, first.frame->timestamp(), 250ms});
        if (!publisher.Connect()) {
            source.Stop();
            throw std::runtime_error(publisher.last_error());
        }
        std::signal(SIGINT, HandleSignal);
        InterruptWatchdog interrupt_watchdog;
        std::uint64_t frames = 0, poses = 0, state_changes = 0;
        std::uint64_t pointcloud_deltas = 0;
        slam_remote::slam::PointCloudDeltaReducer pointcloud_reducer;
        double tracking_seconds = 0.0;
        auto frame = std::move(*first.frame);
        const auto first_timestamp = frame.timestamp();
        auto last_timestamp = first_timestamp;
        auto previous_state = slam_remote::slam::TrackingState::kInitializing;
        const auto began = std::chrono::steady_clock::now();
        while (running != 0 && frames < frame_limit) {
            const auto tracking_started = std::chrono::steady_clock::now();
            const auto result = tracker.Track(frame);
            tracking_seconds += std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() - tracking_started)
                                    .count();
            last_timestamp = frame.timestamp();
            if (result.state != previous_state) ++state_changes;
            previous_state = result.state;
            if (result.pose) ++poses;
            if (!publisher.PublishTracking(result)) {
                throw std::runtime_error(publisher.last_error());
            }
            if (frames % pointcloud_period == 0) {
                const auto delta = pointcloud_reducer.Reduce(tracker.ActiveMapPoints());
                if (delta.operation_count() > 0) {
                    if (!publisher.PublishPointCloud(result.frame_id, result.timestamp, delta)) {
                        throw std::runtime_error(publisher.last_error());
                    }
                    ++pointcloud_deltas;
                }
            }
            ++frames;
            if (frames >= frame_limit) break;
            const auto next = source.NextFrame(2s);
            if (next.status == slam_remote::camera::FrameStatus::kTimeout) continue;
            if (!next.IsValid() || !next.frame) break;
            frame = std::move(*next.frame);
        }
        source.RequestStop();
        source.Stop();
        tracker.Shutdown();
        if (!publisher.EndSession(running == 0 ? "interrupted" : "shutdown")) {
            throw std::runtime_error(publisher.last_error());
        }
        interrupt_watchdog.Complete();
        const auto seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
                                 .count();
        const auto input_seconds =
            std::chrono::duration<double>(last_timestamp - first_timestamp).count();
        std::cout << "live SLAM session passed: frames=" << frames << " poses=" << poses
                  << " dropped=" << source.dropped_frames() << " state_changes=" << state_changes
                  << " pointcloud_deltas=" << pointcloud_deltas
                  << " input_fps=" << (input_seconds > 0.0 ? (frames - 1) / input_seconds : 0.0)
                  << " processed_fps=" << (seconds > 0.0 ? frames / seconds : 0.0)
                  << " mean_track_ms="
                  << (frames > 0 ? tracking_seconds * 1000.0 / frames : 0.0) << '\n';
        return frames > 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << "live SLAM session failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
