#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "slam_remote/camera/image_frame.hpp"

namespace slam_remote::slam {

enum class TrackingState { kInitializing, kTracking, kLost, kRelocalizing };

struct CameraPose final {
    std::array<double, 3> position_metres;
    std::array<double, 4> orientation_xyzw;
};

struct TrackingResult final {
    std::uint64_t frame_id;
    camera::ImageFrame::Timestamp timestamp;
    TrackingState state;
    std::optional<CameraPose> pose;
};

/// Backend-neutral rigid transform used at the ORB-SLAM3 conversion seam.
/// Rotation is a row-major 3x3 matrix and translation is in metres.
struct RigidTransform final {
    std::array<double, 9> rotation;
    std::array<double, 3> translation_metres;
};

/// Convert ORB-SLAM3's world-to-camera Tcw into canonical camera-to-world Twc.
CameraPose ConvertTcwToTwc(const RigidTransform& camera_from_world);

/// Preserve capture metadata and expose a pose only while tracking is valid.
TrackingResult MakeTrackingResult(const camera::ImageFrame& frame, TrackingState state,
                                  const std::optional<RigidTransform>& camera_from_world);

}  // namespace slam_remote::slam
