#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "slam_remote/slam/pose.hpp"

namespace slam_remote::boundary {

inline constexpr std::uint32_t kBoundaryVersion = 1;
inline constexpr std::uint64_t kMaxSafeJsonInteger = 9'007'199'254'740'991ULL;
inline constexpr std::size_t kMaxPayloadBytes = 1024 * 1024;

struct CameraInfo final {
    std::string id;
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t fps;
};

struct PublisherConfig final {
    std::string socket_path;
    std::string session_id;
    std::string producer{"orbslam3-monocular"};
    CameraInfo camera;
    camera::ImageFrame::Timestamp timestamp_origin;
    std::chrono::milliseconds send_timeout{250};
};

std::string SerializeHello(std::string_view session_id, std::string_view producer,
                           const CameraInfo& camera);
std::string SerializeTracking(std::string_view session_id,
                              const slam::TrackingResult& result,
                              camera::ImageFrame::Timestamp timestamp_origin);
std::string SerializeSessionEnd(std::string_view session_id, std::string_view reason);
std::vector<std::uint8_t> FramePayload(std::string_view payload);

/// One synchronous, bounded-buffer connection to the Rust-owned Unix listener.
class Publisher final {
   public:
    explicit Publisher(PublisherConfig config);
    ~Publisher();
    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;

    bool Connect();
    bool PublishTracking(const slam::TrackingResult& result);
    bool EndSession(std::string_view reason = "shutdown");
    void Close() noexcept;
    [[nodiscard]] bool connected() const noexcept;
    [[nodiscard]] const std::string& last_error() const noexcept;

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace slam_remote::boundary
