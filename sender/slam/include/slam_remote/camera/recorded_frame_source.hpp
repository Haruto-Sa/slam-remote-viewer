#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "slam_remote/camera/frame_source.hpp"

namespace slam_remote::camera {

struct RecordedFrameSourceConfig final {
    CameraCalibration calibration;
    std::vector<std::filesystem::path> image_paths;
    ImageFrame::Timestamp frame_period;
};

/// Deterministic source for binary PGM (`P5`, 8-bit) monocular image sequences.
class RecordedFrameSource final : public FrameSource {
   public:
    explicit RecordedFrameSource(RecordedFrameSourceConfig config);

    StartResult Start() override;
    FrameResult NextFrame(std::chrono::milliseconds timeout) override;
    void RequestStop() noexcept override;
    void Stop() noexcept override;
    [[nodiscard]] const CameraCalibration& calibration() const noexcept override;

   private:
    FrameResult LoadFrame(std::size_t index);

    RecordedFrameSourceConfig config_;
    FrameSequenceValidator sequence_validator_;
    std::size_t next_index_{0};
    bool started_{false};
    std::atomic<bool> cancelled_{false};
};

}  // namespace slam_remote::camera
