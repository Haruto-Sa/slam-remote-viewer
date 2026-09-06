#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "slam_remote/diagnostics/live_diagnostics.hpp"
#include "slam_remote/diagnostics/diagnostics_option.hpp"

namespace {

using slam_remote::camera::ImageFrame;
using slam_remote::camera::PixelFormat;
using slam_remote::diagnostics::LiveDiagnosticsStats;
using slam_remote::diagnostics::LiveDiagnosticsStore;
using slam_remote::diagnostics::MakePreviewImage;
using slam_remote::diagnostics::ConsumeDiagnosticsOption;

void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

ImageFrame MakeFrame(PixelFormat format, std::vector<std::uint8_t> pixels) {
    return {7, std::chrono::nanoseconds(100), 2, 1, format, std::move(pixels)};
}

void TestPreviewConversion() {
    const auto gray = MakePreviewImage(MakeFrame(PixelFormat::kGray8, {10, 20}));
    Check(gray.rgb_pixels == std::vector<std::uint8_t>({10, 10, 10, 20, 20, 20}),
          "gray preview must expand to RGB");

    const auto bgr = MakePreviewImage(
        MakeFrame(PixelFormat::kBgr8, {1, 2, 3, 4, 5, 6}));
    Check(bgr.rgb_pixels == std::vector<std::uint8_t>({3, 2, 1, 6, 5, 4}),
          "BGR preview must convert to RGB");

    const auto rgb = MakePreviewImage(
        MakeFrame(PixelFormat::kRgb8, {1, 2, 3, 4, 5, 6}));
    Check(rgb.rgb_pixels == std::vector<std::uint8_t>({1, 2, 3, 4, 5, 6}),
          "RGB preview must preserve channel order");
}

void TestLatestOnlySnapshot() {
    LiveDiagnosticsStore store;
    LiveDiagnosticsStats first_stats;
    first_stats.frames = 1;
    store.UpdateFrame(MakeFrame(PixelFormat::kGray8, {10, 20}), first_stats);
    store.UpdatePointCloud({{10, {1.0, 2.0, 3.0}}});
    const auto first = store.Snapshot();

    LiveDiagnosticsStats second_stats;
    second_stats.frames = 2;
    second_stats.poses = 1;
    second_stats.tracking_state = slam_remote::slam::TrackingState::kTracking;
    store.UpdateFrame(MakeFrame(PixelFormat::kGray8, {30, 40}), second_stats);
    store.UpdatePointCloud({{20, {4.0, 5.0, 6.0}}});
    const auto second = store.Snapshot();

    Check(first.stats.frames == 1 && first.points->front().id == 10,
          "existing readers must retain an immutable snapshot");
    Check(second.stats.frames == 2 && second.stats.poses == 1,
          "new readers must receive the latest statistics");
    Check(second.points->size() == 1 && second.points->front().id == 20,
          "point-cloud update must replace rather than queue snapshots");
    Check(second.image->rgb_pixels.front() == 30,
          "image update must replace rather than queue frames");
}

void TestFinishedState() {
    LiveDiagnosticsStore store;
    store.MarkFinished("producer failed");
    const auto snapshot = store.Snapshot();
    Check(snapshot.finished, "finished state must be observable");
    Check(snapshot.error == "producer failed", "producer error must be preserved");
}

void TestDiagnosticsOption() {
    char executable[] = "sender";
    char frame_limit[] = "0";
    char diagnostics[] = "--diagnostics";
    char* enabled_arguments[] = {executable, frame_limit, diagnostics};
    int enabled_count = 3;
    Check(ConsumeDiagnosticsOption(enabled_count, enabled_arguments),
          "trailing diagnostics option must be enabled");
    Check(enabled_count == 2,
          "diagnostics option must be hidden from the positional parser");

    char* headless_arguments[] = {executable, frame_limit};
    int headless_count = 2;
    Check(!ConsumeDiagnosticsOption(headless_count, headless_arguments),
          "headless mode must remain the default");
    Check(headless_count == 2, "headless positional arguments must remain unchanged");
}

}  // namespace

int main() {
    try {
        TestPreviewConversion();
        TestLatestOnlySnapshot();
        TestFinishedState();
        TestDiagnosticsOption();
    } catch (const std::exception& error) {
        std::cerr << "live diagnostics test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "live diagnostics tests passed\n";
    return EXIT_SUCCESS;
}
