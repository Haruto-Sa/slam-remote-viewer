#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "slam_remote/camera/frame_source.hpp"

namespace slam_remote::camera {

enum class CameraAuthorization { kAuthorized, kDenied, kRestricted, kNotDetermined };

struct MacosCameraDevice final {
    std::string unique_id;
    std::string name;
};

struct MacosCameraConfig final {
    std::string device_id;
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t fps;
    std::size_t queue_capacity;
    CameraCalibration calibration;
};

struct MacosCaptureInfo final {
    std::string device_id;
    std::string device_name;
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t fps;
};

CameraAuthorization GetCameraAuthorization() noexcept;
bool RequestCameraAuthorization();
std::vector<MacosCameraDevice> ListMacosCameraDevices();

class MacosCameraSource final : public FrameSource {
   public:
    class Impl;

    explicit MacosCameraSource(MacosCameraConfig config);
    ~MacosCameraSource() override;
    MacosCameraSource(const MacosCameraSource&) = delete;
    MacosCameraSource& operator=(const MacosCameraSource&) = delete;

    StartResult Start() override;
    FrameResult NextFrame(std::chrono::milliseconds timeout) override;
    void RequestStop() noexcept override;
    void Stop() noexcept override;
    [[nodiscard]] const CameraCalibration& calibration() const noexcept override;
    [[nodiscard]] const MacosCaptureInfo& capture_info() const noexcept;
    [[nodiscard]] std::uint64_t dropped_frames() const noexcept;

   private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace slam_remote::camera
