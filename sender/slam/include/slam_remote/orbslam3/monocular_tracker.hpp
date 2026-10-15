#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "slam_remote/camera/image_frame.hpp"
#include "slam_remote/slam/pose.hpp"
#include "slam_remote/slam/point_cloud_delta_reducer.hpp"

namespace slam_remote::orbslam3 {

struct MonocularTrackerConfig final {
    std::string vocabulary_path;
    std::string settings_path;
    bool enable_viewer{false};
};

struct ActiveMapSnapshot final {
    std::uint64_t map_id;
    std::vector<slam::MapPoint> points;
};

/// Owns one ORB-SLAM3 monocular system. No ORB-SLAM3 type crosses this header.
class MonocularTracker final {
   public:
    explicit MonocularTracker(MonocularTrackerConfig config);
    ~MonocularTracker();
    MonocularTracker(const MonocularTracker&) = delete;
    MonocularTracker& operator=(const MonocularTracker&) = delete;

    slam::TrackingResult Track(const camera::ImageFrame& frame);
    ActiveMapSnapshot ActiveMap();
    void Reset();
    void Shutdown() noexcept;

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace slam_remote::orbslam3
