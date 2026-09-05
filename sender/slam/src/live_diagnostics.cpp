#include "slam_remote/diagnostics/live_diagnostics.hpp"

#include "slam_remote/diagnostics/diagnostics_option.hpp"

#include <cstddef>
#include <utility>

namespace slam_remote::diagnostics {

bool ConsumeDiagnosticsOption(int& argc, char** argv) {
    if (argc <= 1 || argv == nullptr || argv[argc - 1] == nullptr ||
        std::string(argv[argc - 1]) != "--diagnostics") {
        return false;
    }
    --argc;
    return true;
}

PreviewImage MakePreviewImage(const camera::ImageFrame& frame) {
    PreviewImage preview{frame.width(), frame.height(), {}};
    const auto& source = frame.pixels();
    const auto pixel_count = static_cast<std::size_t>(frame.width()) * frame.height();
    preview.rgb_pixels.resize(pixel_count * 3);

    for (std::size_t index = 0; index < pixel_count; ++index) {
        const auto destination = index * 3;
        switch (frame.pixel_format()) {
            case camera::PixelFormat::kGray8:
                preview.rgb_pixels[destination] = source[index];
                preview.rgb_pixels[destination + 1] = source[index];
                preview.rgb_pixels[destination + 2] = source[index];
                break;
            case camera::PixelFormat::kBgr8:
                preview.rgb_pixels[destination] = source[destination + 2];
                preview.rgb_pixels[destination + 1] = source[destination + 1];
                preview.rgb_pixels[destination + 2] = source[destination];
                break;
            case camera::PixelFormat::kRgb8:
                preview.rgb_pixels[destination] = source[destination];
                preview.rgb_pixels[destination + 1] = source[destination + 1];
                preview.rgb_pixels[destination + 2] = source[destination + 2];
                break;
        }
    }
    return preview;
}

LiveDiagnosticsStore::LiveDiagnosticsStore() {
    snapshot_.points = std::make_shared<const std::vector<slam::MapPoint>>();
}

void LiveDiagnosticsStore::UpdateFrame(const camera::ImageFrame& frame,
                                       LiveDiagnosticsStats stats) {
    auto image = std::make_shared<const PreviewImage>(MakePreviewImage(frame));
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.image = std::move(image);
    snapshot_.stats = stats;
}

void LiveDiagnosticsStore::UpdatePointCloud(std::vector<slam::MapPoint> points) {
    auto snapshot =
        std::make_shared<const std::vector<slam::MapPoint>>(std::move(points));
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.points = std::move(snapshot);
}

void LiveDiagnosticsStore::MarkFinished(std::string error) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.finished = true;
    snapshot_.error = std::move(error);
}

LiveDiagnosticsSnapshot LiveDiagnosticsStore::Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

}  // namespace slam_remote::diagnostics
