#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "slam_remote/camera/calibration.hpp"
#include "slam_remote/camera/frame_source.hpp"
#include "slam_remote/camera/image_frame.hpp"

namespace {

using namespace std::chrono_literals;
using slam_remote::camera::CameraCalibration;
using slam_remote::camera::CameraModel;
using slam_remote::camera::FrameResult;
using slam_remote::camera::FrameSource;
using slam_remote::camera::FrameSequenceValidator;
using slam_remote::camera::FrameStatus;
using slam_remote::camera::ImageFrame;
using slam_remote::camera::PixelFormat;
using slam_remote::camera::StartResult;

void Check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

ImageFrame MakeFrame(std::uint64_t frame_id, std::int64_t timestamp_ns) {
    return ImageFrame(frame_id, ImageFrame::Timestamp(timestamp_ns), 2, 2, PixelFormat::kGray8,
                      {0, 1, 2, 3});
}

CameraCalibration MakeCalibration() {
    return {"fixture-camera", 2, 2, CameraModel::kPinhole, 100.0, 100.0, 1.0, 1.0,
            {0.0, 0.0, 0.0, 0.0}};
}

class InMemoryFrameSource final : public FrameSource {
   public:
    explicit InMemoryFrameSource(std::vector<ImageFrame> frames)
        : frames_(std::move(frames)), calibration_(MakeCalibration()) {}

    StartResult Start() override {
        if (started_) {
            return StartResult::Failure("frame source is already started");
        }
        started_ = true;
        cancelled_ = false;
        return StartResult::Success();
    }

    FrameResult NextFrame(std::chrono::milliseconds timeout) override {
        if (!started_) {
            return FrameResult::WithoutFrame(FrameStatus::kFatalError,
                                             "frame source is not started");
        }
        if (cancelled_) {
            return FrameResult::WithoutFrame(FrameStatus::kCancelled);
        }
        if (timeout <= 0ms) {
            return FrameResult::WithoutFrame(FrameStatus::kTimeout);
        }
        if (next_ == frames_.size()) {
            return FrameResult::WithoutFrame(FrameStatus::kEndOfStream);
        }
        return FrameResult::Available(frames_.at(next_++));
    }

    void RequestStop() noexcept override { cancelled_ = true; }
    void Stop() noexcept override {
        started_ = false;
        cancelled_ = true;
    }
    [[nodiscard]] const CameraCalibration& calibration() const noexcept override {
        return calibration_;
    }

   private:
    std::vector<ImageFrame> frames_;
    CameraCalibration calibration_;
    std::size_t next_{0};
    bool started_{false};
    bool cancelled_{false};
};

void TestImmutableFrameMetadata() {
    const auto frame = MakeFrame(42, 1'500'000);
    Check(frame.frame_id() == 42, "frame ID must be preserved");
    Check(frame.timestamp() == ImageFrame::Timestamp(1'500'000),
          "timestamp must be preserved");
    Check(frame.width() == 2 && frame.height() == 2, "dimensions must be preserved");
    Check(frame.pixel_format() == PixelFormat::kGray8, "pixel format must be preserved");
    Check(frame.pixels() == std::vector<std::uint8_t>({0, 1, 2, 3}),
          "pixels must be owned by the immutable frame");
}

void TestInvalidFrameMetadata() {
    bool negative_timestamp_rejected = false;
    try {
        static_cast<void>(MakeFrame(0, -1));
    } catch (const std::invalid_argument&) {
        negative_timestamp_rejected = true;
    }
    Check(negative_timestamp_rejected, "negative timestamp must be rejected");

    bool byte_count_rejected = false;
    try {
        static_cast<void>(ImageFrame(0, ImageFrame::Timestamp(0), 2, 2, PixelFormat::kRgb8,
                                     {0, 1, 2, 3}));
    } catch (const std::invalid_argument&) {
        byte_count_rejected = true;
    }
    Check(byte_count_rejected, "invalid pixel byte count must be rejected");

    bool pixel_format_rejected = false;
    try {
        static_cast<void>(ImageFrame(0, ImageFrame::Timestamp(0), 1, 1,
                                     static_cast<PixelFormat>(255), {}));
    } catch (const std::invalid_argument&) {
        pixel_format_rejected = true;
    }
    Check(pixel_format_rejected, "unknown pixel format must be rejected");
}

void TestCalibrationValidation() {
    Check(!MakeCalibration().Validate().has_value(), "valid calibration must pass");
    auto invalid = MakeCalibration();
    invalid.fx = std::nan("");
    Check(invalid.Validate() == "calibration focal lengths must be finite and positive",
          "non-finite focal length must be rejected");
    invalid = MakeCalibration();
    invalid.width = 1;
    Check(invalid.Validate() == "calibration principal point must be finite and inside the image",
          "principal point outside image must be rejected");
}

void TestSourceLifecycleAndOrdering() {
    InMemoryFrameSource source({MakeFrame(7, 100), MakeFrame(8, 200)});
    Check(source.NextFrame(1ms).status == FrameStatus::kFatalError,
          "read before start must fail");
    const auto started = source.Start();
    Check(started.IsValid() && started.started, "source must start");
    const auto duplicate_start = source.Start();
    Check(duplicate_start.IsValid() && !duplicate_start.started,
          "double start must return a contextual failure");
    Check(source.NextFrame(0ms).status == FrameStatus::kTimeout, "timeout must be observable");

    const auto first = source.NextFrame(1ms);
    const auto second = source.NextFrame(1ms);
    Check(first.IsValid() && second.IsValid(), "available frame results must be valid");
    Check(first.frame->frame_id() < second.frame->frame_id(), "frame IDs must be monotonic");
    Check(first.frame->timestamp() < second.frame->timestamp(), "timestamps must be monotonic");
    Check(source.NextFrame(1ms).status == FrameStatus::kEndOfStream,
          "finite source must expose end of stream");
    source.RequestStop();
    Check(source.NextFrame(1ms).status == FrameStatus::kCancelled,
          "cancellation must be observable");
    source.Stop();
}

void TestResultInvariants() {
    Check(FrameResult::Available(MakeFrame(0, 0)).IsValid(),
          "available result must contain one frame");
    Check(FrameResult::WithoutFrame(FrameStatus::kRecoverableError, "device dropped frame")
              .IsValid(),
          "recoverable error must include context");
    Check(!FrameResult::WithoutFrame(FrameStatus::kFatalError).IsValid(),
          "fatal error without context must be invalid");
}

void TestSequenceValidation() {
    FrameSequenceValidator validator;
    Check(!validator.Validate(MakeFrame(10, 100)).has_value(), "first frame must pass");
    Check(validator.Validate(MakeFrame(10, 200)) == "frame ID must increase monotonically",
          "duplicate frame ID must be rejected");
    Check(validator.Validate(MakeFrame(11, 100)) == "frame timestamp must increase monotonically",
          "duplicate timestamp must be rejected");
    Check(!validator.Validate(MakeFrame(11, 200)).has_value(),
          "rejected frames must not replace the previous frame");
    validator.Reset();
    Check(!validator.Validate(MakeFrame(0, 0)).has_value(), "reset must begin a new sequence");
}

}  // namespace

int main() {
    try {
        TestImmutableFrameMetadata();
        TestInvalidFrameMetadata();
        TestCalibrationValidation();
        TestSourceLifecycleAndOrdering();
        TestResultInvariants();
        TestSequenceValidation();
    } catch (const std::exception& error) {
        std::cerr << "camera contract test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "camera contract tests passed\n";
    return EXIT_SUCCESS;
}
