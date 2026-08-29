#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "slam_remote/camera/calibration_io.hpp"

namespace {

using slam_remote::camera::LoadCalibrationDocument;
using slam_remote::camera::ToOrbSlam3MonocularYaml;

void Check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class Fixture final {
   public:
    Fixture() : path_(std::filesystem::temp_directory_path() / "slam-calibration-test.txt") {}
    ~Fixture() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }
    void Write(const std::string& content) const {
        std::ofstream output(path_);
        output << content;
    }
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

   private:
    std::filesystem::path path_;
};

std::string ValidCalibration() {
    return R"(version=1
device_id=builtin-camera
width=1280
height=720
fps=30
model=pinhole
fx=900.5
fy=901.5
cx=640
cy=360
distortion=0.1,-0.2,0.003,-0.004,0.05
rms_reprojection_error_px=0.42
board_columns=9
board_rows=6
square_size_m=0.024
calibrated_at_utc=2026-08-30T00:00:00Z
source=checkerboard/*.png
)";
}

void TestLoadsAndGeneratesOrbYaml(const Fixture& fixture) {
    fixture.Write(ValidCalibration());
    const auto document = LoadCalibrationDocument(fixture.path());
    Check(document.camera.device_id == "builtin-camera", "device ID must load");
    Check(document.camera.width == 1280 && document.camera.height == 720,
          "dimensions must load");
    Check(document.rms_reprojection_error_px == 0.42, "reprojection error must load");

    const auto yaml = ToOrbSlam3MonocularYaml(document);
    Check(yaml.find("File.version: \"1.0\"") != std::string::npos,
          "ORB settings version must be emitted");
    Check(yaml.find("Camera.type: \"PinHole\"") != std::string::npos,
          "ORB camera model must be emitted");
    Check(yaml.find("Camera1.fx: 900.5") != std::string::npos,
          "intrinsics must be emitted");
    Check(yaml.find("Camera.width: 1280") != std::string::npos,
          "dimensions must be emitted");
    Check(yaml.find("Camera.fps: 30") != std::string::npos, "FPS must be emitted");
    Check(yaml.find("ORBextractor.nFeatures: 1000") != std::string::npos,
          "explicit ORB defaults must be emitted");
}

void ExpectFailure(const Fixture& fixture, const std::string& content,
                   const std::string& expected) {
    fixture.Write(content);
    try {
        static_cast<void>(LoadCalibrationDocument(fixture.path()));
        throw std::runtime_error("invalid calibration unexpectedly passed");
    } catch (const std::runtime_error& error) {
        Check(error.what() == expected, "unexpected validation error: " + std::string(error.what()));
    }
}

void TestRejectsUnknownMissingDuplicateAndWrongTypes(const Fixture& fixture) {
    ExpectFailure(fixture, ValidCalibration() + "unknown=value\n",
                  "unknown calibration field 'unknown'");

    auto missing = ValidCalibration();
    const auto position = missing.find("fps=30\n");
    missing.erase(position, std::string("fps=30\n").size());
    ExpectFailure(fixture, missing, "missing calibration field 'fps'");

    ExpectFailure(fixture, ValidCalibration() + "fps=60\n",
                  "duplicate calibration field on line 18");

    auto wrong = ValidCalibration();
    wrong.replace(wrong.find("width=1280"), std::string("width=1280").size(), "width=1280px");
    ExpectFailure(fixture, wrong, "invalid unsigned integer for 'width'");
}

void TestRejectsModeAndProvenanceErrors(const Fixture& fixture) {
    auto invalid = ValidCalibration();
    invalid.replace(invalid.find("cx=640"), std::string("cx=640").size(), "cx=1280");
    ExpectFailure(fixture, invalid,
                  "calibration principal point must be finite and inside the image");

    invalid = ValidCalibration();
    invalid.replace(invalid.find("rms_reprojection_error_px=0.42"),
                    std::string("rms_reprojection_error_px=0.42").size(),
                    "rms_reprojection_error_px=-1");
    ExpectFailure(fixture, invalid, "reprojection error must be finite and non-negative");

    invalid = ValidCalibration();
    invalid.replace(invalid.find("source=checkerboard/*.png"),
                    std::string("source=checkerboard/*.png").size(), "source= ");
    ExpectFailure(fixture, invalid, "calibration field 'source' must not be empty");

    invalid = ValidCalibration();
    invalid.replace(invalid.find("calibrated_at_utc=2026-08-30T00:00:00Z"),
                    std::string("calibrated_at_utc=2026-08-30T00:00:00Z").size(),
                    "calibrated_at_utc=2026/08/30");
    ExpectFailure(fixture, invalid,
                  "calibration UTC timestamp must use YYYY-MM-DDTHH:MM:SSZ");
}

}  // namespace

int main() {
    try {
        const Fixture fixture;
        TestLoadsAndGeneratesOrbYaml(fixture);
        TestRejectsUnknownMissingDuplicateAndWrongTypes(fixture);
        TestRejectsModeAndProvenanceErrors(fixture);
    } catch (const std::exception& error) {
        std::cerr << "calibration IO test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "calibration IO tests passed\n";
    return EXIT_SUCCESS;
}
