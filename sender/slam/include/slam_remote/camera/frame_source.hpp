#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "slam_remote/camera/calibration.hpp"
#include "slam_remote/camera/image_frame.hpp"

namespace slam_remote::camera {

enum class FrameStatus {
    kFrameAvailable,
    kTimeout,
    kEndOfStream,
    kCancelled,
    kRecoverableError,
    kFatalError,
};

struct FrameResult final {
    FrameStatus status;
    std::optional<ImageFrame> frame;
    std::string error;

    static FrameResult Available(ImageFrame value) {
        return {FrameStatus::kFrameAvailable, std::move(value), {}};
    }
    static FrameResult WithoutFrame(FrameStatus value, std::string message = {}) {
        return {value, std::nullopt, std::move(message)};
    }
    [[nodiscard]] bool IsValid() const noexcept {
        if (status == FrameStatus::kFrameAvailable) {
            return frame.has_value() && error.empty();
        }
        if (frame.has_value()) {
            return false;
        }
        const bool requires_error = status == FrameStatus::kRecoverableError ||
                                    status == FrameStatus::kFatalError;
        return !requires_error || !error.empty();
    }
};

struct StartResult final {
    bool started;
    std::string error;

    static StartResult Success() { return {true, {}}; }
    static StartResult Failure(std::string message) { return {false, std::move(message)}; }
    [[nodiscard]] bool IsValid() const noexcept { return started == error.empty(); }
};

/// Reusable ordering check for every frame-source implementation.
class FrameSequenceValidator final {
   public:
    [[nodiscard]] std::optional<std::string> Validate(const ImageFrame& frame) {
        if (last_frame_id_.has_value() && frame.frame_id() <= *last_frame_id_) {
            return "frame ID must increase monotonically";
        }
        if (last_timestamp_.has_value() && frame.timestamp() <= *last_timestamp_) {
            return "frame timestamp must increase monotonically";
        }
        last_frame_id_ = frame.frame_id();
        last_timestamp_ = frame.timestamp();
        return std::nullopt;
    }

    void Reset() noexcept {
        last_frame_id_.reset();
        last_timestamp_.reset();
    }

   private:
    std::optional<std::uint64_t> last_frame_id_;
    std::optional<ImageFrame::Timestamp> last_timestamp_;
};

/// Single-consumer frame source for recorded and live monocular cameras.
/// `RequestStop` is the only method required to be safe from another thread.
class FrameSource {
   public:
    virtual ~FrameSource() = default;
    virtual StartResult Start() = 0;
    virtual FrameResult NextFrame(std::chrono::milliseconds timeout) = 0;
    virtual void RequestStop() noexcept = 0;
    virtual void Stop() noexcept = 0;
    [[nodiscard]] virtual const CameraCalibration& calibration() const noexcept = 0;
};

}  // namespace slam_remote::camera
