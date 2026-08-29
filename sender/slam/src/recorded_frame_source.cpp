#include "slam_remote/camera/recorded_frame_source.hpp"

#include <cctype>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace slam_remote::camera {
namespace {

bool ReadToken(std::istream& input, std::string& token) {
    token.clear();
    char current = '\0';
    while (input.get(current)) {
        if (current == '#') {
            input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        if (!std::isspace(static_cast<unsigned char>(current))) {
            token.push_back(current);
            break;
        }
    }
    while (input.get(current)) {
        if (std::isspace(static_cast<unsigned char>(current))) {
            return true;
        }
        if (current == '#') {
            input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return true;
        }
        token.push_back(current);
    }
    return !token.empty();
}

std::uint32_t ParseDimension(const std::string& token, const char* name) {
    std::size_t consumed = 0;
    unsigned long parsed = 0;
    try {
        parsed = std::stoul(token, &consumed, 10);
    } catch (const std::exception&) {
        throw std::runtime_error(std::string("invalid PGM ") + name);
    }
    if (consumed != token.size() || parsed == 0 ||
        parsed > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error(std::string("invalid PGM ") + name);
    }
    return static_cast<std::uint32_t>(parsed);
}

std::vector<std::uint8_t> ReadPgm(const std::filesystem::path& path, std::uint32_t expected_width,
                                  std::uint32_t expected_height) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("cannot open recorded frame: " + path.string());
    }

    std::string magic;
    std::string width_token;
    std::string height_token;
    std::string max_value_token;
    if (!ReadToken(input, magic) || !ReadToken(input, width_token) ||
        !ReadToken(input, height_token) || !ReadToken(input, max_value_token)) {
        throw std::runtime_error("incomplete PGM header: " + path.string());
    }
    if (magic != "P5") {
        throw std::runtime_error("recorded frame must be binary PGM (P5): " + path.string());
    }

    const auto width = ParseDimension(width_token, "width");
    const auto height = ParseDimension(height_token, "height");
    const auto max_value = ParseDimension(max_value_token, "max value");
    if (max_value != 255) {
        throw std::runtime_error("recorded PGM max value must be 255: " + path.string());
    }
    if (width != expected_width || height != expected_height) {
        throw std::runtime_error("recorded frame dimensions do not match calibration: " +
                                 path.string());
    }

    const auto byte_count = static_cast<std::size_t>(width) * height;
    std::vector<std::uint8_t> pixels(byte_count);
    input.read(reinterpret_cast<char*>(pixels.data()), static_cast<std::streamsize>(byte_count));
    if (input.gcount() != static_cast<std::streamsize>(byte_count)) {
        throw std::runtime_error("recorded PGM pixel data is truncated: " + path.string());
    }
    if (input.peek() != std::char_traits<char>::eof()) {
        throw std::runtime_error("recorded PGM has trailing pixel data: " + path.string());
    }
    return pixels;
}

}  // namespace

RecordedFrameSource::RecordedFrameSource(RecordedFrameSourceConfig config)
    : config_(std::move(config)) {}

StartResult RecordedFrameSource::Start() {
    if (started_) {
        return StartResult::Failure("recorded frame source is already started");
    }
    if (const auto error = config_.calibration.Validate(); error.has_value()) {
        return StartResult::Failure(*error);
    }
    if (config_.image_paths.empty()) {
        return StartResult::Failure("recorded frame sequence must not be empty");
    }
    if (config_.frame_period.count() <= 0) {
        return StartResult::Failure("recorded frame period must be positive");
    }

    next_index_ = 0;
    sequence_validator_.Reset();
    cancelled_.store(false);
    started_ = true;
    return StartResult::Success();
}

FrameResult RecordedFrameSource::NextFrame(std::chrono::milliseconds timeout) {
    if (!started_) {
        return FrameResult::WithoutFrame(FrameStatus::kFatalError,
                                         "recorded frame source is not started");
    }
    if (cancelled_.load()) {
        return FrameResult::WithoutFrame(FrameStatus::kCancelled);
    }
    if (timeout.count() <= 0) {
        return FrameResult::WithoutFrame(FrameStatus::kTimeout);
    }
    if (next_index_ >= config_.image_paths.size()) {
        return FrameResult::WithoutFrame(FrameStatus::kEndOfStream);
    }
    return LoadFrame(next_index_++);
}

void RecordedFrameSource::RequestStop() noexcept { cancelled_.store(true); }

void RecordedFrameSource::Stop() noexcept {
    cancelled_.store(true);
    started_ = false;
}

const CameraCalibration& RecordedFrameSource::calibration() const noexcept {
    return config_.calibration;
}

FrameResult RecordedFrameSource::LoadFrame(std::size_t index) {
    try {
        const auto frame_id = static_cast<std::uint64_t>(index);
        if (frame_id > static_cast<std::uint64_t>(
                           std::numeric_limits<ImageFrame::Timestamp::rep>::max() /
                           config_.frame_period.count())) {
            return FrameResult::WithoutFrame(FrameStatus::kFatalError,
                                             "recorded frame timestamp overflow");
        }
        ImageFrame frame(frame_id, config_.frame_period * frame_id, config_.calibration.width,
                         config_.calibration.height, PixelFormat::kGray8,
                         ReadPgm(config_.image_paths.at(index), config_.calibration.width,
                                 config_.calibration.height));
        if (const auto error = sequence_validator_.Validate(frame); error.has_value()) {
            return FrameResult::WithoutFrame(FrameStatus::kFatalError, *error);
        }
        return FrameResult::Available(std::move(frame));
    } catch (const std::exception& error) {
        return FrameResult::WithoutFrame(FrameStatus::kRecoverableError, error.what());
    }
}

}  // namespace slam_remote::camera
