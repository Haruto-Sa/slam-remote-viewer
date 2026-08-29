#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace slam_remote::camera {

enum class CameraModel { kPinhole, kFisheye };

struct CameraCalibration final {
    std::string device_id;
    std::uint32_t width;
    std::uint32_t height;
    CameraModel model;
    double fx;
    double fy;
    double cx;
    double cy;
    std::vector<double> distortion;

    [[nodiscard]] std::optional<std::string> Validate() const {
        if (device_id.empty()) {
            return "calibration device ID must not be empty";
        }
        if (width == 0 || height == 0) {
            return "calibration dimensions must be positive";
        }
        if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0) {
            return "calibration focal lengths must be finite and positive";
        }
        if (!std::isfinite(cx) || !std::isfinite(cy) || cx < 0.0 || cy < 0.0 ||
            cx >= width || cy >= height) {
            return "calibration principal point must be finite and inside the image";
        }
        for (const double coefficient : distortion) {
            if (!std::isfinite(coefficient)) {
                return "calibration distortion coefficients must be finite";
            }
        }
        if (model == CameraModel::kPinhole && distortion.size() != 4 &&
            distortion.size() != 5) {
            return "pinhole calibration requires four or five distortion coefficients";
        }
        if (model == CameraModel::kFisheye && distortion.size() != 4) {
            return "fisheye calibration requires four distortion coefficients";
        }
        return std::nullopt;
    }
};

}  // namespace slam_remote::camera
