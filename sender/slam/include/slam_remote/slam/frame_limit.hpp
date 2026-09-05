#pragma once

#include <charconv>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <system_error>

namespace slam_remote::slam {

/// A zero frame limit runs until an external stop is requested.
class FrameLimit final {
   public:
    static FrameLimit Parse(std::string_view text) {
        std::uint32_t value = 0;
        const auto result = std::from_chars(text.data(), text.data() + text.size(), value, 10);
        if (text.empty() || result.ec != std::errc{} ||
            result.ptr != text.data() + text.size()) {
            throw std::invalid_argument("FRAME_LIMIT must be a uint32 (0 means run until stop)");
        }
        return FrameLimit(value);
    }

    [[nodiscard]] bool unlimited() const noexcept { return value_ == 0; }
    [[nodiscard]] bool reached(std::uint64_t processed_frames) const noexcept {
        return !unlimited() && processed_frames >= value_;
    }
    [[nodiscard]] std::uint32_t value() const noexcept { return value_; }

   private:
    explicit FrameLimit(std::uint32_t value) : value_(value) {}

    std::uint32_t value_;
};

}  // namespace slam_remote::slam
