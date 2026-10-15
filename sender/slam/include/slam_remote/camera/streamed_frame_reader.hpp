#pragma once

#include <istream>
#include <optional>

#include "slam_remote/camera/image_frame.hpp"

namespace slam_remote::camera {

/// Reads SLF1 Gray8 frames from a local pipe. The owner controls pipe lifetime
/// and idle timeouts. Errors terminate the stream; resynchronization is unsafe.
class StreamedFrameReader final {
   public:
    StreamedFrameReader(std::istream& input, std::uint32_t width, std::uint32_t height);
    std::optional<ImageFrame> Next();

   private:
    std::istream& input_;
    std::uint32_t width_;
    std::uint32_t height_;
    std::optional<std::uint64_t> last_id_;
    std::optional<std::uint64_t> last_timestamp_;
    bool finished_{false};
};

}  // namespace slam_remote::camera
