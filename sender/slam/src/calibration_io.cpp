#include "slam_remote/camera/calibration_io.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace slam_remote::camera {
namespace {

std::string Trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::uint32_t ParseU32(const std::map<std::string, std::string>& values,
                       const std::string& key) {
    const auto& text = values.at(key);
    std::size_t consumed = 0;
    unsigned long parsed = 0;
    try {
        parsed = std::stoul(text, &consumed, 10);
    } catch (const std::exception&) {
        throw std::runtime_error("invalid unsigned integer for '" + key + "'");
    }
    if (consumed != text.size() || parsed > UINT32_MAX) {
        throw std::runtime_error("invalid unsigned integer for '" + key + "'");
    }
    return static_cast<std::uint32_t>(parsed);
}

double ParseDouble(const std::map<std::string, std::string>& values, const std::string& key) {
    const auto& text = values.at(key);
    std::size_t consumed = 0;
    double parsed = 0.0;
    try {
        parsed = std::stod(text, &consumed);
    } catch (const std::exception&) {
        throw std::runtime_error("invalid number for '" + key + "'");
    }
    if (consumed != text.size() || !std::isfinite(parsed)) {
        throw std::runtime_error("invalid number for '" + key + "'");
    }
    return parsed;
}

std::vector<double> ParseDoubles(const std::string& text) {
    std::vector<double> result;
    std::istringstream input(text);
    std::string token;
    while (std::getline(input, token, ',')) {
        token = Trim(token);
        std::size_t consumed = 0;
        double parsed = 0.0;
        try {
            parsed = std::stod(token, &consumed);
        } catch (const std::exception&) {
            throw std::runtime_error("invalid distortion coefficient");
        }
        if (consumed != token.size() || !std::isfinite(parsed)) {
            throw std::runtime_error("invalid distortion coefficient");
        }
        result.push_back(parsed);
    }
    return result;
}

std::map<std::string, std::string> ParseFile(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open calibration file: " + path.string());
    }
    const std::set<std::string> allowed{
        "version",       "device_id",      "width",          "height",
        "fps",           "model",          "fx",             "fy",
        "cx",            "cy",             "distortion",     "rms_reprojection_error_px",
        "board_columns", "board_rows",     "square_size_m",  "calibrated_at_utc",
        "source",
    };
    std::map<std::string, std::string> values;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        line = Trim(line);
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const auto separator = line.find('=');
        if (separator == std::string::npos) {
            throw std::runtime_error("calibration line " + std::to_string(line_number) +
                                     " must contain '='");
        }
        auto key = Trim(line.substr(0, separator));
        auto value = Trim(line.substr(separator + 1));
        if (!allowed.count(key)) {
            throw std::runtime_error("unknown calibration field '" + key + "'");
        }
        if (value.empty()) {
            throw std::runtime_error("calibration field '" + key + "' must not be empty");
        }
        if (!values.emplace(std::move(key), std::move(value)).second) {
            throw std::runtime_error("duplicate calibration field on line " +
                                     std::to_string(line_number));
        }
    }
    for (const auto& key : allowed) {
        if (!values.count(key)) {
            throw std::runtime_error("missing calibration field '" + key + "'");
        }
    }
    return values;
}

void ValidateFeatures(const OrbFeatureSettings& features) {
    if (features.features == 0 || !std::isfinite(features.scale_factor) ||
        features.scale_factor <= 1.0 || features.levels == 0 ||
        features.initial_fast_threshold == 0 || features.minimum_fast_threshold == 0 ||
        features.minimum_fast_threshold > features.initial_fast_threshold) {
        throw std::runtime_error("invalid ORB feature settings");
    }
}

}  // namespace

CalibrationDocument LoadCalibrationDocument(const std::filesystem::path& path) {
    const auto values = ParseFile(path);
    if (values.at("version") != "1") {
        throw std::runtime_error("unsupported calibration version");
    }
    CameraModel model;
    if (values.at("model") == "pinhole") {
        model = CameraModel::kPinhole;
    } else if (values.at("model") == "fisheye") {
        model = CameraModel::kFisheye;
    } else {
        throw std::runtime_error("unsupported calibration camera model");
    }

    CalibrationDocument document{
        {values.at("device_id"), ParseU32(values, "width"), ParseU32(values, "height"), model,
         ParseDouble(values, "fx"), ParseDouble(values, "fy"), ParseDouble(values, "cx"),
         ParseDouble(values, "cy"), ParseDoubles(values.at("distortion"))},
        ParseU32(values, "fps"),
        ParseDouble(values, "rms_reprojection_error_px"),
        ParseU32(values, "board_columns"),
        ParseU32(values, "board_rows"),
        ParseDouble(values, "square_size_m"),
        values.at("calibrated_at_utc"),
        values.at("source"),
    };
    ValidateCalibrationDocument(document);
    return document;
}

void ValidateCalibrationDocument(const CalibrationDocument& document) {
    if (const auto error = document.camera.Validate(); error.has_value()) {
        throw std::runtime_error(*error);
    }
    if (document.fps == 0) {
        throw std::runtime_error("calibration FPS must be positive");
    }
    if (!std::isfinite(document.rms_reprojection_error_px) ||
        document.rms_reprojection_error_px < 0.0) {
        throw std::runtime_error("reprojection error must be finite and non-negative");
    }
    if (document.board_columns < 2 || document.board_rows < 2) {
        throw std::runtime_error("calibration board must contain at least 2x2 inner corners");
    }
    if (!std::isfinite(document.square_size_m) || document.square_size_m <= 0.0) {
        throw std::runtime_error("calibration square size must be finite and positive");
    }
    if (document.calibrated_at_utc.empty() || document.source.empty()) {
        throw std::runtime_error("calibration provenance must not be empty");
    }
    const std::regex utc_timestamp(R"(^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z$)");
    if (!std::regex_match(document.calibrated_at_utc, utc_timestamp)) {
        throw std::runtime_error("calibration UTC timestamp must use YYYY-MM-DDTHH:MM:SSZ");
    }
}

std::string ToOrbSlam3MonocularYaml(const CalibrationDocument& document,
                                    const OrbFeatureSettings& features) {
    ValidateCalibrationDocument(document);
    ValidateFeatures(features);
    if (document.camera.model != CameraModel::kPinhole) {
        throw std::runtime_error("ORB-SLAM3 YAML generation currently supports pinhole only");
    }
    const auto& distortion = document.camera.distortion;
    std::ostringstream output;
    output << std::setprecision(17)
           << "%YAML:1.0\n---\n"
           << "File.version: \"1.0\"\n"
           << "Camera.type: \"PinHole\"\n"
           << "Camera1.fx: " << document.camera.fx << '\n'
           << "Camera1.fy: " << document.camera.fy << '\n'
           << "Camera1.cx: " << document.camera.cx << '\n'
           << "Camera1.cy: " << document.camera.cy << '\n'
           << "Camera1.k1: " << distortion.at(0) << '\n'
           << "Camera1.k2: " << distortion.at(1) << '\n'
           << "Camera1.p1: " << distortion.at(2) << '\n'
           << "Camera1.p2: " << distortion.at(3) << '\n'
           << "Camera1.k3: " << (distortion.size() == 5 ? distortion.at(4) : 0.0) << '\n'
           << "Camera.width: " << document.camera.width << '\n'
           << "Camera.height: " << document.camera.height << '\n'
           << "Camera.fps: " << document.fps << '\n'
           << "Camera.RGB: 0\n"
           << "ORBextractor.nFeatures: " << features.features << '\n'
           << "ORBextractor.scaleFactor: " << features.scale_factor << '\n'
           << "ORBextractor.nLevels: " << features.levels << '\n'
           << "ORBextractor.iniThFAST: " << features.initial_fast_threshold << '\n'
           << "ORBextractor.minThFAST: " << features.minimum_fast_threshold << '\n';
    return output.str();
}

}  // namespace slam_remote::camera
