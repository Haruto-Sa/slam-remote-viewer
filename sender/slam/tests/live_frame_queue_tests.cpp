#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "slam_remote/camera/bounded_frame_queue.hpp"

namespace {

using namespace std::chrono_literals;
using slam_remote::camera::BoundedFrameQueue;
using slam_remote::camera::ImageFrame;
using slam_remote::camera::PixelFormat;

void Check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

ImageFrame Frame(std::uint64_t id) {
    return ImageFrame(id, ImageFrame::Timestamp(id), 1, 1, PixelFormat::kGray8,
                      {static_cast<std::uint8_t>(id)});
}

void TestDropsOldestDeterministically() {
    BoundedFrameQueue queue(2);
    queue.Push(Frame(0));
    queue.Push(Frame(1));
    queue.Push(Frame(2));

    Check(queue.dropped_frames() == 1, "overflow must be counted");
    const auto first = queue.WaitPop(1ms);
    const auto second = queue.WaitPop(1ms);
    Check(first.has_value() && first->frame_id() == 1, "oldest frame must be dropped");
    Check(second.has_value() && second->frame_id() == 2, "new frames must preserve order");
}

void TestTimeoutDoesNotChangeQueue() {
    BoundedFrameQueue queue(1);
    Check(!queue.WaitPop(1ms).has_value(), "empty queue must time out");
    Check(queue.dropped_frames() == 0, "timeout must not count as overflow");
}

void TestCancellationUnblocksWaiter() {
    BoundedFrameQueue queue(1);
    auto waiting = std::async(std::launch::async, [&queue] { return queue.WaitPop(5s); });
    queue.Cancel();
    Check(waiting.wait_for(1s) == std::future_status::ready,
          "cancellation must promptly unblock the consumer");
    Check(!waiting.get().has_value(), "cancelled wait must not return a frame");
}

void TestResetStartsFreshSession() {
    BoundedFrameQueue queue(1);
    queue.Push(Frame(0));
    queue.Push(Frame(1));
    queue.Cancel();
    queue.Reset();
    queue.Push(Frame(2));
    Check(queue.dropped_frames() == 0, "reset must clear overflow count");
    const auto frame = queue.WaitPop(1ms);
    Check(frame.has_value() && frame->frame_id() == 2, "reset must discard old session frames");
}

}  // namespace

int main() {
    try {
        TestDropsOldestDeterministically();
        TestTimeoutDoesNotChangeQueue();
        TestCancellationUnblocksWaiter();
        TestResetStartsFreshSession();
    } catch (const std::exception& error) {
        std::cerr << "live frame queue test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "live frame queue tests passed\n";
    return EXIT_SUCCESS;
}
