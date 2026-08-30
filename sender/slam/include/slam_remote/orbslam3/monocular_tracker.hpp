#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "slam_remote/camera/image_frame.hpp"
#include "slam_remote/telemetry/point_cloud_delta_reducer.hpp"

namespace ORB_SLAM3 {
class System;
}  // namespace ORB_SLAM3

namespace slam_remote::orbslam3 {

enum class TrackingState { kInitializing, kTracking, kLost, kRelocalizing };

struct CameraPose final {
    std::array<double, 3> position;
    std::array<double, 4> orientation_xyzw;
};

struct MonocularTrackingResult final {
    std::uint64_t frame_id;
    double timestamp_seconds;
    TrackingState state;
    std::optional<CameraPose> pose;
    std::vector<telemetry::MapPoint> tracked_points;
};

struct MonocularTrackerConfig final {
    std::string vocabulary_path;
    std::string settings_path;
    bool enable_pangolin_viewer{true};
};

/// Single-threaded bridge from the shared camera frame contract to ORB-SLAM3.
/// Lost results retain the last valid pose when one exists, allowing the UI to
/// display tracking loss without snapping the camera back to the origin.
class MonocularTracker final {
   public:
    explicit MonocularTracker(MonocularTrackerConfig config);
    ~MonocularTracker();
    MonocularTracker(const MonocularTracker&) = delete;
    MonocularTracker& operator=(const MonocularTracker&) = delete;

    MonocularTrackingResult Track(const camera::ImageFrame& frame);
    void Reset();
    void Shutdown();

   private:
    std::unique_ptr<ORB_SLAM3::System> system_;
    std::optional<CameraPose> last_valid_pose_;
    bool shut_down_{false};
};

}  // namespace slam_remote::orbslam3
