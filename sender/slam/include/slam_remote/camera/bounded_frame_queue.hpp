#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

#include "slam_remote/camera/image_frame.hpp"

namespace slam_remote::camera {

/// Thread-safe bounded queue that deterministically drops the oldest frame.
class BoundedFrameQueue final {
   public:
    explicit BoundedFrameQueue(std::size_t capacity) : capacity_(capacity) {
        if (capacity == 0) {
            throw std::invalid_argument("frame queue capacity must be positive");
        }
    }

    void Push(ImageFrame frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (cancelled_) {
            return;
        }
        if (frames_.size() == capacity_) {
            frames_.pop_front();
            ++dropped_frames_;
        }
        frames_.push_back(std::move(frame));
        available_.notify_one();
    }

    template <typename Rep, typename Period>
    std::optional<ImageFrame> WaitPop(const std::chrono::duration<Rep, Period>& timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        available_.wait_for(lock, timeout, [this] { return cancelled_ || !frames_.empty(); });
        if (cancelled_ || frames_.empty()) {
            return std::nullopt;
        }
        auto frame = std::move(frames_.front());
        frames_.pop_front();
        return frame;
    }

    void Cancel() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = true;
        frames_.clear();
        available_.notify_all();
    }

    void Reset() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = false;
        dropped_frames_ = 0;
        frames_.clear();
    }

    [[nodiscard]] std::uint64_t dropped_frames() const noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_frames_;
    }

   private:
    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable available_;
    std::deque<ImageFrame> frames_;
    std::uint64_t dropped_frames_{0};
    bool cancelled_{false};
};

}  // namespace slam_remote::camera
