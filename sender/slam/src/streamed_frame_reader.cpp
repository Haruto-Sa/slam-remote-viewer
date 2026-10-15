#include "slam_remote/camera/streamed_frame_reader.hpp"

#include <array>
#include <limits>
#include <stdexcept>

namespace slam_remote::camera {
namespace {
constexpr std::uint64_t kMaxPixels = 1920ULL * 1080;
constexpr std::uint64_t kMaxFrameId = 9'007'199'254'740'991ULL;

std::uint64_t Decode(const unsigned char* bytes, std::size_t count) {
    std::uint64_t result = 0;
    for (std::size_t i = 0; i < count; ++i) result = (result << 8) | bytes[i];
    return result;
}
}  // namespace

StreamedFrameReader::StreamedFrameReader(std::istream& input, std::uint32_t width,
                                       std::uint32_t height)
    : input_(input), width_(width), height_(height) {
    if (width == 0 || height == 0 || width > 1920 || height > 1920 ||
        static_cast<std::uint64_t>(width) * height > kMaxPixels) {
        throw std::invalid_argument("stream dimensions exceed the 1920x1080 pixel budget");
    }
}

std::optional<ImageFrame> StreamedFrameReader::Next() {
    if (finished_) return std::nullopt;
    // Every exit except a complete valid frame makes this reader terminal.
    finished_ = true;
    std::array<unsigned char, 28> header{};
    input_.read(reinterpret_cast<char*>(header.data()), header.size());
    if (input_.gcount() == 0 && input_.eof() && !input_.bad()) return std::nullopt;
    if (input_.gcount() != static_cast<std::streamsize>(header.size()) || !input_) {
        throw std::runtime_error("truncated or unreadable SLF1 header");
    }
    if (header[0] != 'S' || header[1] != 'L' || header[2] != 'F' || header[3] != '1') {
        throw std::runtime_error("unsupported camera stream magic/version (expected SLF1)");
    }
    const auto width = Decode(header.data() + 4, 4);
    const auto height = Decode(header.data() + 8, 4);
    const auto id = Decode(header.data() + 12, 8);
    const auto timestamp = Decode(header.data() + 20, 8);
    if (width != width_ || height != height_) {
        throw std::runtime_error("stream dimensions do not match camera calibration");
    }
    if (id > kMaxFrameId || timestamp > std::numeric_limits<std::int64_t>::max()) {
        throw std::runtime_error("stream frame ID or timestamp is out of range");
    }
    if ((last_id_ && id <= *last_id_) ||
        (last_timestamp_ && timestamp <= *last_timestamp_)) {
        throw std::runtime_error("stream frame ID and capture timestamp must strictly increase");
    }
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height);
    input_.read(reinterpret_cast<char*>(pixels.data()), pixels.size());
    if (input_.gcount() != static_cast<std::streamsize>(pixels.size()) || !input_) {
        throw std::runtime_error("truncated or unreadable SLF1 pixels");
    }
    ImageFrame frame(id, ImageFrame::Timestamp(timestamp), width_, height_,
                     PixelFormat::kGray8, std::move(pixels));
    last_id_ = id;
    last_timestamp_ = timestamp;
    finished_ = false;
    return frame;
}
}  // namespace slam_remote::camera
