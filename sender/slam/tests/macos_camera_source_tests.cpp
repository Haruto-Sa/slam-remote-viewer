#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "slam_remote/camera/macos_camera_source.hpp"

namespace {

using slam_remote::camera::CameraAuthorization;
using slam_remote::camera::CameraCalibration;
using slam_remote::camera::CameraModel;
using slam_remote::camera::GetCameraAuthorization;
using slam_remote::camera::ListMacosCameraDevices;
using slam_remote::camera::MacosCameraConfig;
using slam_remote::camera::MacosCameraSource;

void Check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

CameraCalibration Calibration() {
    return {"test-device", 640, 480, CameraModel::kPinhole, 500.0, 500.0, 320.0, 240.0,
            {0.0, 0.0, 0.0, 0.0}};
}

MacosCameraConfig Config() { return {"test-device", 640, 480, 30, 4, Calibration()}; }

void TestRejectsInvalidConfigurationBeforePermissionCheck() {
    auto config = Config();
    config.queue_capacity = 0;
    MacosCameraSource invalid_queue(std::move(config));
    Check(invalid_queue.Start().error == "camera queue capacity must be positive",
          "zero queue capacity must be diagnosed before camera access");

    config = Config();
    config.calibration.width = 1280;
    MacosCameraSource mismatched_calibration(std::move(config));
    Check(mismatched_calibration.Start().error ==
              "camera mode dimensions do not match calibration",
          "calibration mismatch must be diagnosed before camera access");
}

void TestPlatformQueriesAreSafeWithoutCamera() {
    const auto authorization = GetCameraAuthorization();
    Check(authorization == CameraAuthorization::kAuthorized ||
              authorization == CameraAuthorization::kDenied ||
              authorization == CameraAuthorization::kRestricted ||
              authorization == CameraAuthorization::kNotDetermined,
          "authorization must map to a public status");
    for (const auto& device : ListMacosCameraDevices()) {
        Check(!device.unique_id.empty(), "enumerated camera must have a stable unique ID");
        Check(!device.name.empty(), "enumerated camera must have a display name");
    }
}

void TestUnauthorizedStartIsActionable() {
    if (GetCameraAuthorization() == CameraAuthorization::kAuthorized) {
        return;
    }
    MacosCameraSource source(Config());
    const auto result = source.Start();
    Check(!result.started &&
              result.error ==
                  "camera access is not authorized; run macos_camera_dump --request-permission",
          "unauthorized start must explain how to request access");
}

}  // namespace

int main() {
    try {
        TestRejectsInvalidConfigurationBeforePermissionCheck();
        TestPlatformQueriesAreSafeWithoutCamera();
        TestUnauthorizedStartIsActionable();
    } catch (const std::exception& error) {
        std::cerr << "macOS camera source test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "macOS camera source tests passed\n";
    return EXIT_SUCCESS;
}
