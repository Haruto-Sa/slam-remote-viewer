#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "slam_remote/camera/calibration.hpp"

namespace slam_remote::camera {

struct CalibrationDocument final {
    CameraCalibration camera;
    std::uint32_t fps;
    double rms_reprojection_error_px;
    std::uint32_t board_columns;
    std::uint32_t board_rows;
    double square_size_m;
    std::string calibrated_at_utc;
    std::string source;
};

struct OrbFeatureSettings final {
    std::uint32_t features{1000};
    double scale_factor{1.2};
    std::uint32_t levels{8};
    std::uint32_t initial_fast_threshold{20};
    std::uint32_t minimum_fast_threshold{7};
};

CalibrationDocument LoadCalibrationDocument(const std::filesystem::path& path);
void ValidateCalibrationDocument(const CalibrationDocument& document);
std::string ToOrbSlam3MonocularYaml(
    const CalibrationDocument& document,
    const OrbFeatureSettings& features = OrbFeatureSettings{});

}  // namespace slam_remote::camera
