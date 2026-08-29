#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "slam_remote/camera/recorded_frame_source.hpp"

namespace {

using namespace std::chrono_literals;
using slam_remote::camera::CameraCalibration;
using slam_remote::camera::CameraModel;
using slam_remote::camera::FrameStatus;
using slam_remote::camera::ImageFrame;
using slam_remote::camera::RecordedFrameSource;
using slam_remote::camera::RecordedFrameSourceConfig;

void Check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class FixtureDirectory final {
   public:
    FixtureDirectory() {
        path_ = std::filesystem::temp_directory_path() / "slam-recorded-frame-source-tests";
        std::filesystem::remove_all(path_);
        std::filesystem::create_directories(path_);
    }
    ~FixtureDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

   private:
    std::filesystem::path path_;
};

void WritePgm(const std::filesystem::path& path, std::uint32_t width, std::uint32_t height,
              const std::vector<std::uint8_t>& pixels) {
    std::ofstream output(path, std::ios::binary);
    output << "P5\n# generated test fixture\n" << width << ' ' << height << "\n255\n";
    output.write(reinterpret_cast<const char*>(pixels.data()),
                 static_cast<std::streamsize>(pixels.size()));
}

CameraCalibration Calibration() {
    return {"recorded-fixture", 2, 2, CameraModel::kPinhole, 100.0, 100.0, 1.0, 1.0,
            {0.0, 0.0, 0.0, 0.0}};
}

RecordedFrameSourceConfig Config(std::vector<std::filesystem::path> paths) {
    return {Calibration(), std::move(paths), ImageFrame::Timestamp(50'000'000)};
}

void TestDeterministicFramesAndEndOfStream(const FixtureDirectory& fixtures) {
    const auto first_path = fixtures.path() / "000.pgm";
    const auto second_path = fixtures.path() / "001.pgm";
    WritePgm(first_path, 2, 2, {0, 1, 2, 3});
    WritePgm(second_path, 2, 2, {4, 5, 6, 7});

    RecordedFrameSource source(Config({first_path, second_path}));
    const auto start = source.Start();
    Check(start.IsValid() && start.started, "valid source must start");

    const auto first = source.NextFrame(1ms);
    const auto second = source.NextFrame(1ms);
    Check(first.IsValid() && second.IsValid(), "both recorded frames must load");
    Check(first.frame->frame_id() == 0 && second.frame->frame_id() == 1,
          "recorded frame IDs must start at zero and increase");
    Check(first.frame->timestamp().count() == 0 &&
              second.frame->timestamp().count() == 50'000'000,
          "timestamps must derive only from index and configured period");
    Check(first.frame->pixels() == std::vector<std::uint8_t>({0, 1, 2, 3}),
          "first fixture bytes must be preserved");
    Check(second.frame->pixels() == std::vector<std::uint8_t>({4, 5, 6, 7}),
          "second fixture bytes must be preserved");
    Check(source.NextFrame(1ms).status == FrameStatus::kEndOfStream,
          "finite playback must report end of stream");
    source.Stop();
}

void TestMalformedAndMissingFramesAreRecoverable(const FixtureDirectory& fixtures) {
    const auto malformed_path = fixtures.path() / "malformed.pgm";
    const auto valid_path = fixtures.path() / "valid.pgm";
    {
        std::ofstream malformed(malformed_path, std::ios::binary);
        malformed << "P2\n2 2\n255\n0 1 2 3\n";
    }
    WritePgm(valid_path, 2, 2, {9, 8, 7, 6});

    RecordedFrameSource source(
        Config({fixtures.path() / "missing.pgm", malformed_path, valid_path}));
    Check(source.Start().started, "source config must start before opening individual frames");

    const auto missing = source.NextFrame(1ms);
    const auto malformed = source.NextFrame(1ms);
    const auto valid = source.NextFrame(1ms);
    Check(missing.status == FrameStatus::kRecoverableError && !missing.error.empty(),
          "missing frame must have a recoverable diagnostic");
    Check(malformed.status == FrameStatus::kRecoverableError && !malformed.error.empty(),
          "malformed frame must have a recoverable diagnostic");
    Check(valid.IsValid() && valid.frame->frame_id() == 2,
          "source must continue deterministically after invalid files");
}

void TestLifecycleValidationAndCancellation(const FixtureDirectory& fixtures) {
    const auto path = fixtures.path() / "cancel.pgm";
    WritePgm(path, 2, 2, {0, 0, 0, 0});

    auto empty = Config({});
    RecordedFrameSource empty_source(std::move(empty));
    const auto empty_start = empty_source.Start();
    Check(empty_start.IsValid() && !empty_start.started,
          "empty sequence must return a contextual start failure");

    auto invalid_period = Config({path});
    invalid_period.frame_period = ImageFrame::Timestamp(0);
    RecordedFrameSource invalid_period_source(std::move(invalid_period));
    Check(!invalid_period_source.Start().started, "zero frame period must fail");

    RecordedFrameSource source(Config({path}));
    Check(source.NextFrame(1ms).status == FrameStatus::kFatalError,
          "read before start must fail");
    Check(source.Start().started, "source must start");
    Check(source.NextFrame(0ms).status == FrameStatus::kTimeout,
          "zero timeout must not consume a frame");
    source.RequestStop();
    Check(source.NextFrame(1ms).status == FrameStatus::kCancelled,
          "requested cancellation must be observable");
    source.Stop();
}

void TestDimensionAndPayloadValidation(const FixtureDirectory& fixtures) {
    const auto wrong_size = fixtures.path() / "wrong-size.pgm";
    const auto truncated = fixtures.path() / "truncated.pgm";
    WritePgm(wrong_size, 1, 1, {0});
    WritePgm(truncated, 2, 2, {0, 1});

    RecordedFrameSource source(Config({wrong_size, truncated}));
    Check(source.Start().started, "source must start");
    Check(source.NextFrame(1ms).status == FrameStatus::kRecoverableError,
          "calibration dimension mismatch must be rejected");
    Check(source.NextFrame(1ms).status == FrameStatus::kRecoverableError,
          "truncated payload must be rejected");
}

}  // namespace

int main() {
    try {
        const FixtureDirectory fixtures;
        TestDeterministicFramesAndEndOfStream(fixtures);
        TestMalformedAndMissingFramesAreRecoverable(fixtures);
        TestLifecycleValidationAndCancellation(fixtures);
        TestDimensionAndPayloadValidation(fixtures);
    } catch (const std::exception& error) {
        std::cerr << "recorded frame source test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "recorded frame source tests passed\n";
    return EXIT_SUCCESS;
}
