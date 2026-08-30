#include "slam_remote/orbslam3/monocular_tracker.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include <GL/glew.h>
#include <opencv2/core.hpp>

#include "System.h"
#include "Tracking.h"
#include "slam_remote/orbslam3/tracked_map_points.hpp"

namespace slam_remote::orbslam3 {
namespace {

int OpenCvType(camera::PixelFormat format) {
    switch (format) {
        case camera::PixelFormat::kGray8:
            return CV_8UC1;
        case camera::PixelFormat::kBgr8:
        case camera::PixelFormat::kRgb8:
            return CV_8UC3;
    }
    throw std::invalid_argument("unsupported camera pixel format");
}

TrackingState ConvertState(int state) noexcept {
    switch (state) {
        case ORB_SLAM3::Tracking::OK:
        case ORB_SLAM3::Tracking::OK_KLT:
            return TrackingState::kTracking;
        case ORB_SLAM3::Tracking::RECENTLY_LOST:
            return TrackingState::kRelocalizing;
        case ORB_SLAM3::Tracking::LOST:
            return TrackingState::kLost;
        default:
            return TrackingState::kInitializing;
    }
}

CameraPose ToCameraPose(const Sophus::SE3f& camera_from_world) {
    const Sophus::SE3f world_from_camera = camera_from_world.inverse();
    const auto translation = world_from_camera.translation();
    const auto quaternion = world_from_camera.unit_quaternion().normalized();
    return {{{translation.x(), translation.y(), translation.z()}},
            {{quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()}}};
}

}  // namespace

MonocularTracker::MonocularTracker(MonocularTrackerConfig config) {
    if (config.vocabulary_path.empty() || config.settings_path.empty()) {
        throw std::invalid_argument("ORB-SLAM3 vocabulary and settings paths are required");
    }
    system_ = std::make_unique<ORB_SLAM3::System>(
        config.vocabulary_path, config.settings_path, ORB_SLAM3::System::MONOCULAR,
        config.enable_pangolin_viewer);
}

MonocularTracker::~MonocularTracker() { Shutdown(); }

MonocularTrackingResult MonocularTracker::Track(const camera::ImageFrame& frame) {
    if (shut_down_) {
        throw std::logic_error("cannot track after ORB-SLAM3 shutdown");
    }

    cv::Mat image(static_cast<int>(frame.height()), static_cast<int>(frame.width()),
                  OpenCvType(frame.pixel_format()),
                  const_cast<std::uint8_t*>(frame.pixels().data()));
    const double timestamp =
        std::chrono::duration<double>(frame.timestamp()).count();
    const Sophus::SE3f camera_from_world = system_->TrackMonocular(image, timestamp);
    const TrackingState state = ConvertState(system_->GetTrackingState());

    if (state == TrackingState::kTracking) {
        last_valid_pose_ = ToCameraPose(camera_from_world);
    }

    return {frame.frame_id(), timestamp, state, last_valid_pose_,
            CopyTrackedMapPoints(*system_)};
}

void MonocularTracker::Reset() {
    if (shut_down_) {
        throw std::logic_error("cannot reset after ORB-SLAM3 shutdown");
    }
    system_->ResetActiveMap();
    last_valid_pose_.reset();
}

void MonocularTracker::Shutdown() {
    if (!shut_down_ && system_) {
        system_->Shutdown();
        shut_down_ = true;
    }
}

}  // namespace slam_remote::orbslam3
