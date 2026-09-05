#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "slam_remote/slam/frame_limit.hpp"

namespace {

using slam_remote::slam::FrameLimit;

void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void CheckRejected(const std::string& text) {
    bool rejected = false;
    try {
        static_cast<void>(FrameLimit::Parse(text));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Check(rejected, "invalid frame limit must be rejected: " + text);
}

void TestUnlimitedLimit() {
    const auto limit = FrameLimit::Parse("0");
    Check(limit.unlimited(), "zero must select run-until-stop");
    Check(limit.value() == 0, "zero value must be preserved");
    Check(!limit.reached(0), "unlimited run must allow its first frame");
    Check(!limit.reached(std::numeric_limits<std::uint64_t>::max()),
          "unlimited run must never reach a frame limit");
}

void TestFiniteLimit() {
    const auto limit = FrameLimit::Parse("900");
    Check(!limit.unlimited(), "positive limit must remain finite");
    Check(!limit.reached(899), "finite run must continue below its limit");
    Check(limit.reached(900), "finite run must stop at its limit");
    Check(limit.reached(901), "finite run must stay stopped above its limit");

    const auto maximum = FrameLimit::Parse("4294967295");
    Check(maximum.value() == std::numeric_limits<std::uint32_t>::max(),
          "maximum uint32 limit must be accepted");
}

void TestInvalidLimits() {
    for (const auto* text : {"", "-1", "+1", "1.0", " 1", "1 ", "abc", "4294967296"}) {
        CheckRejected(text);
    }
}

}  // namespace

int main() {
    try {
        TestUnlimitedLimit();
        TestFiniteLimit();
        TestInvalidLimits();
    } catch (const std::exception& error) {
        std::cerr << "frame limit test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "frame limit tests passed\n";
    return EXIT_SUCCESS;
}
