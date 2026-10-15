#pragma once

#include <memory>
#include <opencv2/core/mat.hpp>
#include "slam_remote/slam/pose.hpp"
#include "slam_remote/slam/point_cloud_delta_reducer.hpp"

namespace slam_remote::rtabmap_backend {

struct RgbdTrackerConfig final {
    int width;
    int height;
    double fx;
    double fy;
    double cx;
    double cy;
};

/// One RGB-D mapping session. Images must already be registered and rectified.
/// Color is BGR8; depth is float32 metres with zero for missing measurements.
/// No RTAB-Map objects cross this interface. Reset by creating a fresh tracker
/// AND a new boundary session; point identities must never span resets.
class RgbdTracker final {
 public:
    explicit RgbdTracker(RgbdTrackerConfig config);
    ~RgbdTracker();
    RgbdTracker(const RgbdTracker&) = delete;
    RgbdTracker& operator=(const RgbdTracker&) = delete;
    slam::TrackingResult Track(std::uint64_t frame_id,
                               camera::ImageFrame::Timestamp timestamp,
                               const cv::Mat& color, const cv::Mat& depth_metres);
    std::vector<slam::MapPoint> MapPoints() const;
 private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace slam_remote::rtabmap_backend
