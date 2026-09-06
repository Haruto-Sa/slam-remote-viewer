#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "slam_remote/camera/image_frame.hpp"
#include "slam_remote/slam/point_cloud_delta_reducer.hpp"
#include "slam_remote/slam/pose.hpp"

namespace slam_remote::diagnostics {

struct PreviewImage final {
    std::uint32_t width;
    std::uint32_t height;
    std::vector<std::uint8_t> rgb_pixels;
};

struct LiveDiagnosticsStats final {
    slam::TrackingState tracking_state{slam::TrackingState::kInitializing};
    std::uint64_t frames{0};
    std::uint64_t poses{0};
    std::uint64_t pointcloud_deltas{0};
    std::uint64_t dropped_frames{0};
    double input_fps{0.0};
    double processed_fps{0.0};
    double mean_tracking_ms{0.0};
};

struct LiveDiagnosticsSnapshot final {
    std::shared_ptr<const PreviewImage> image;
    std::shared_ptr<const std::vector<slam::MapPoint>> points;
    LiveDiagnosticsStats stats;
    bool finished{false};
    std::string error;
};

PreviewImage MakePreviewImage(const camera::ImageFrame& frame);

/// Thread-safe latest-only state shared by one producer and one local viewer.
class LiveDiagnosticsStore final {
   public:
    LiveDiagnosticsStore();

    void UpdateFrame(const camera::ImageFrame& frame, LiveDiagnosticsStats stats);
    void UpdatePointCloud(std::vector<slam::MapPoint> points);
    void MarkFinished(std::string error = {});
    [[nodiscard]] LiveDiagnosticsSnapshot Snapshot() const;

   private:
    mutable std::mutex mutex_;
    LiveDiagnosticsSnapshot snapshot_;
};

}  // namespace slam_remote::diagnostics
