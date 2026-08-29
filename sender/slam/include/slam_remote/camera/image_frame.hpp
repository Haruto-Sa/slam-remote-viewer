#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace slam_remote::camera {

enum class PixelFormat { kGray8, kBgr8, kRgb8 };

constexpr std::size_t BytesPerPixel(PixelFormat format) noexcept {
    switch (format) {
        case PixelFormat::kGray8:
            return 1;
        case PixelFormat::kBgr8:
        case PixelFormat::kRgb8:
            return 3;
    }
    return 0;
}

/// Immutable image and capture metadata owned by one pipeline stage.
class ImageFrame final {
   public:
    using Timestamp = std::chrono::nanoseconds;

    ImageFrame(std::uint64_t frame_id, Timestamp timestamp, std::uint32_t width,
               std::uint32_t height, PixelFormat pixel_format,
               std::vector<std::uint8_t> pixels)
        : frame_id_(frame_id),
          timestamp_(timestamp),
          width_(width),
          height_(height),
          pixel_format_(pixel_format),
          pixels_(std::move(pixels)) {
        if (timestamp_.count() < 0) {
            throw std::invalid_argument("frame timestamp must be non-negative");
        }
        if (width_ == 0 || height_ == 0) {
            throw std::invalid_argument("frame dimensions must be positive");
        }
        const auto bytes_per_pixel = BytesPerPixel(pixel_format_);
        if (bytes_per_pixel == 0) {
            throw std::invalid_argument("unsupported pixel format");
        }
        const auto pixel_count = static_cast<std::uint64_t>(width_) * height_;
        if (pixel_count > std::numeric_limits<std::size_t>::max() / bytes_per_pixel) {
            throw std::invalid_argument("frame byte count exceeds addressable memory");
        }
        const auto bytes = static_cast<std::size_t>(pixel_count) * bytes_per_pixel;
        if (bytes != pixels_.size()) {
            throw std::invalid_argument("pixel byte count does not match frame metadata");
        }
    }

    [[nodiscard]] std::uint64_t frame_id() const noexcept { return frame_id_; }
    [[nodiscard]] Timestamp timestamp() const noexcept { return timestamp_; }
    [[nodiscard]] std::uint32_t width() const noexcept { return width_; }
    [[nodiscard]] std::uint32_t height() const noexcept { return height_; }
    [[nodiscard]] PixelFormat pixel_format() const noexcept { return pixel_format_; }
    [[nodiscard]] const std::vector<std::uint8_t>& pixels() const noexcept { return pixels_; }

   private:
    std::uint64_t frame_id_;
    Timestamp timestamp_;
    std::uint32_t width_;
    std::uint32_t height_;
    PixelFormat pixel_format_;
    std::vector<std::uint8_t> pixels_;
};

}  // namespace slam_remote::camera
