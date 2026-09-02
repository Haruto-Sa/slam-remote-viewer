#include "slam_remote/slam/pose.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace slam_remote::slam {
namespace {

void RequireFinite(const RigidTransform& transform) {
    const auto finite = [](double value) { return std::isfinite(value); };
    if (!std::all_of(transform.rotation.begin(), transform.rotation.end(), finite) ||
        !std::all_of(transform.translation_metres.begin(),
                     transform.translation_metres.end(), finite)) {
        throw std::invalid_argument("SLAM transform must contain only finite values");
    }
    const auto& r = transform.rotation;
    constexpr double kRotationTolerance = 1e-5;
    for (std::size_t row = 0; row < 3; ++row) {
        double norm = 0.0;
        for (std::size_t column = 0; column < 3; ++column) {
            norm += r[row * 3 + column] * r[row * 3 + column];
        }
        if (std::abs(norm - 1.0) > kRotationTolerance) {
            throw std::invalid_argument("SLAM rotation rows must be unit length");
        }
    }
    for (std::size_t left = 0; left < 3; ++left) {
        for (std::size_t right = left + 1; right < 3; ++right) {
            double dot = 0.0;
            for (std::size_t column = 0; column < 3; ++column) {
                dot += r[left * 3 + column] * r[right * 3 + column];
            }
            if (std::abs(dot) > kRotationTolerance) {
                throw std::invalid_argument("SLAM rotation rows must be orthogonal");
            }
        }
    }
    const double determinant =
        r[0] * (r[4] * r[8] - r[5] * r[7]) -
        r[1] * (r[3] * r[8] - r[5] * r[6]) +
        r[2] * (r[3] * r[7] - r[4] * r[6]);
    if (std::abs(determinant - 1.0) > kRotationTolerance) {
        throw std::invalid_argument("SLAM rotation must be right-handed");
    }
}

std::array<double, 4> RotationToQuaternion(const std::array<double, 9>& rotation) {
    const double trace = rotation[0] + rotation[4] + rotation[8];
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;

    if (trace > 0.0) {
        const double scale = 2.0 * std::sqrt(trace + 1.0);
        w = 0.25 * scale;
        x = (rotation[7] - rotation[5]) / scale;
        y = (rotation[2] - rotation[6]) / scale;
        z = (rotation[3] - rotation[1]) / scale;
    } else if (rotation[0] > rotation[4] && rotation[0] > rotation[8]) {
        const double scale = 2.0 * std::sqrt(1.0 + rotation[0] - rotation[4] - rotation[8]);
        w = (rotation[7] - rotation[5]) / scale;
        x = 0.25 * scale;
        y = (rotation[1] + rotation[3]) / scale;
        z = (rotation[2] + rotation[6]) / scale;
    } else if (rotation[4] > rotation[8]) {
        const double scale = 2.0 * std::sqrt(1.0 + rotation[4] - rotation[0] - rotation[8]);
        w = (rotation[2] - rotation[6]) / scale;
        x = (rotation[1] + rotation[3]) / scale;
        y = 0.25 * scale;
        z = (rotation[5] + rotation[7]) / scale;
    } else {
        const double scale = 2.0 * std::sqrt(1.0 + rotation[8] - rotation[0] - rotation[4]);
        w = (rotation[3] - rotation[1]) / scale;
        x = (rotation[2] + rotation[6]) / scale;
        y = (rotation[5] + rotation[7]) / scale;
        z = 0.25 * scale;
    }

    const double norm = std::hypot(std::hypot(x, y), std::hypot(z, w));
    if (!std::isfinite(norm) || norm <= 1e-12) {
        throw std::invalid_argument("SLAM rotation does not produce a valid quaternion");
    }
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
    if (w < 0.0) {
        x = -x;
        y = -y;
        z = -z;
        w = -w;
    }
    return {x, y, z, w};
}

}  // namespace

CameraPose ConvertTcwToTwc(const RigidTransform& camera_from_world) {
    RequireFinite(camera_from_world);
    const auto& source = camera_from_world.rotation;
    const std::array<double, 9> world_from_camera_rotation{
        source[0], source[3], source[6], source[1], source[4],
        source[7], source[2], source[5], source[8]};
    const auto& translation = camera_from_world.translation_metres;
    const std::array<double, 3> world_from_camera_translation{
        -(source[0] * translation[0] + source[3] * translation[1] +
          source[6] * translation[2]),
        -(source[1] * translation[0] + source[4] * translation[1] +
          source[7] * translation[2]),
        -(source[2] * translation[0] + source[5] * translation[1] +
          source[8] * translation[2])};
    return {world_from_camera_translation,
            RotationToQuaternion(world_from_camera_rotation)};
}

TrackingResult MakeTrackingResult(const camera::ImageFrame& frame, TrackingState state,
                                  const std::optional<RigidTransform>& camera_from_world) {
    if (state == TrackingState::kTracking) {
        if (!camera_from_world.has_value()) {
            throw std::invalid_argument("tracking state requires a valid Tcw transform");
        }
        return {frame.frame_id(), frame.timestamp(), state,
                ConvertTcwToTwc(*camera_from_world)};
    }
    return {frame.frame_id(), frame.timestamp(), state, std::nullopt};
}

}  // namespace slam_remote::slam
