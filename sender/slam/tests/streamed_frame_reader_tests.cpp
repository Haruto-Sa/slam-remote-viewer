#include "slam_remote/camera/streamed_frame_reader.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
void Check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

std::string Frame(std::uint64_t id, std::uint64_t timestamp,
                  std::uint32_t width = 2, std::uint32_t height = 2) {
    std::string bytes = "SLF1";
    const auto append = [&](std::uint64_t value, int count) {
        for (int shift = (count - 1) * 8; shift >= 0; shift -= 8) {
            bytes.push_back(static_cast<char>((value >> shift) & 255));
        }
    };
    append(width, 4); append(height, 4); append(id, 8); append(timestamp, 8);
    bytes.append("\0\x40\x80\xff", 4);
    return bytes;
}

void Reject(const std::string& bytes, bool after_first = false) {
    std::istringstream input(bytes);
    slam_remote::camera::StreamedFrameReader reader(input, 2, 2);
    if (after_first) Check(reader.Next().has_value(), "first valid frame missing");
    bool failed = false;
    try { reader.Next(); } catch (const std::runtime_error&) { failed = true; }
    Check(failed, "malformed stream was accepted");
    Check(!reader.Next(), "malformed stream must be terminal");
}
}  // namespace

int main() {
    try {
        std::istringstream input(Frame(0, 0) + Frame(9, 123456789));
        slam_remote::camera::StreamedFrameReader reader(input, 2, 2);
        const auto first = reader.Next();
        Check(first && first->frame_id() == 0 && first->timestamp().count() == 0 &&
                  first->pixels() == std::vector<std::uint8_t>({0, 64, 128, 255}),
              "first metadata/pixels were not preserved");
        const auto second = reader.Next();
        Check(second && second->frame_id() == 9 && second->timestamp().count() == 123456789,
              "capture timestamps and skipped frame IDs must be preserved");
        Check(!reader.Next() && !reader.Next(), "boundary EOF must be repeatably clean");
        const auto good = Frame(1, 1);
        for (std::size_t length = 1; length < good.size(); ++length) Reject(good.substr(0, length));
        auto bad_version = good; bad_version[3] = '2'; Reject(bad_version);
        Reject(Frame(1, 1, 0, 2));
        Reject(Frame(1, 1, UINT32_MAX, UINT32_MAX));
        Reject(Frame(UINT64_MAX, 1));
        Reject(Frame(1, UINT64_MAX));
        Reject(good + Frame(1, 2), true);
        Reject(good + Frame(2, 1), true);
        Reject(good + Frame(0, 2), true);
        Reject(good + Frame(2, 0), true);
        bool invalid_dimensions = false;
        try { slam_remote::camera::StreamedFrameReader invalid(input, 1920, 1920); }
        catch (const std::invalid_argument&) { invalid_dimensions = true; }
        Check(invalid_dimensions, "oversized configuration accepted");
        std::istringstream broken;
        broken.setstate(std::ios::badbit);
        slam_remote::camera::StreamedFrameReader failed(broken, 2, 2);
        bool io_failed = false;
        try { failed.Next(); } catch (const std::runtime_error&) { io_failed = true; }
        Check(io_failed, "I/O failure must not look like clean EOF");
        std::cout << "streamed frame reader tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
