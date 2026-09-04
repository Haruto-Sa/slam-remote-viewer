#include "slam_remote/boundary/publisher.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

namespace slam_remote::boundary {
namespace {

void RequireText(std::string_view value, const char* field) {
    if (value.empty()) {
        throw std::invalid_argument(std::string(field) + " must not be empty");
    }
    for (std::size_t index = 0; index < value.size();) {
        const auto byte = static_cast<unsigned char>(value[index]);
        std::size_t continuation_count = 0;
        std::uint32_t codepoint = 0;
        if (byte <= 0x7f) {
            ++index;
            continue;
        } else if (byte >= 0xc2 && byte <= 0xdf) {
            continuation_count = 1;
            codepoint = byte & 0x1f;
        } else if (byte >= 0xe0 && byte <= 0xef) {
            continuation_count = 2;
            codepoint = byte & 0x0f;
        } else if (byte >= 0xf0 && byte <= 0xf4) {
            continuation_count = 3;
            codepoint = byte & 0x07;
        } else {
            throw std::invalid_argument(std::string(field) + " must be valid UTF-8");
        }
        if (index + continuation_count >= value.size()) {
            throw std::invalid_argument(std::string(field) + " must be valid UTF-8");
        }
        for (std::size_t offset = 1; offset <= continuation_count; ++offset) {
            const auto continuation = static_cast<unsigned char>(value[index + offset]);
            if ((continuation & 0xc0) != 0x80) {
                throw std::invalid_argument(std::string(field) + " must be valid UTF-8");
            }
            codepoint = (codepoint << 6) | (continuation & 0x3f);
        }
        const bool overlong = (continuation_count == 2 && codepoint < 0x800) ||
                              (continuation_count == 3 && codepoint < 0x10000);
        if (overlong || (codepoint >= 0xd800 && codepoint <= 0xdfff) ||
            codepoint > 0x10ffff) {
            throw std::invalid_argument(std::string(field) + " must be valid UTF-8");
        }
        index += continuation_count + 1;
    }
}

std::string JsonString(std::string_view value) {
    std::ostringstream output;
    output << '"';
    for (const unsigned char character : value) {
        switch (character) {
            case '"': output << "\\\""; break;
            case '\\': output << "\\\\"; break;
            case '\b': output << "\\b"; break;
            case '\f': output << "\\f"; break;
            case '\n': output << "\\n"; break;
            case '\r': output << "\\r"; break;
            case '\t': output << "\\t"; break;
            default:
                if (character < 0x20) {
                    output << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                           << static_cast<int>(character) << std::dec;
                } else {
                    output << static_cast<char>(character);
                }
        }
    }
    output << '"';
    return output.str();
}

const char* StateName(slam::TrackingState state) {
    switch (state) {
        case slam::TrackingState::kInitializing: return "initializing";
        case slam::TrackingState::kTracking: return "tracking";
        case slam::TrackingState::kLost: return "lost";
        case slam::TrackingState::kRelocalizing: return "relocalizing";
    }
    throw std::invalid_argument("unknown tracking state");
}

void AppendNumber(std::ostringstream& output, double value) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument("boundary number must be finite");
    }
    output << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
}

void AppendPose(std::ostringstream& output, const slam::CameraPose& pose) {
    double norm_squared = 0.0;
    for (double value : pose.position_metres) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("pose translation must be finite");
        }
    }
    for (double value : pose.orientation_xyzw) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("pose orientation must be finite");
        }
        norm_squared += value * value;
    }
    if (std::abs(std::sqrt(norm_squared) - 1.0) > 1e-6) {
        throw std::invalid_argument("pose orientation must be a unit quaternion");
    }
    output << "{\"translation\":[";
    for (std::size_t index = 0; index < pose.position_metres.size(); ++index) {
        if (index != 0) output << ',';
        AppendNumber(output, pose.position_metres[index]);
    }
    output << "],\"orientation_xyzw\":[";
    for (std::size_t index = 0; index < pose.orientation_xyzw.size(); ++index) {
        if (index != 0) output << ',';
        AppendNumber(output, pose.orientation_xyzw[index]);
    }
    output << "]}";
}

void AppendMapPoints(std::ostringstream& output, const std::vector<slam::MapPoint>& points) {
    output << '[';
    for (std::size_t index = 0; index < points.size(); ++index) {
        if (index != 0) output << ',';
        const auto& point = points[index];
        if (point.id > kMaxSafeJsonInteger) {
            throw std::invalid_argument("map point ID exceeds JSON-safe integer maximum");
        }
        output << "{\"id\":" << point.id << ",\"position\":[";
        for (std::size_t axis = 0; axis < point.position_metres.size(); ++axis) {
            if (axis != 0) output << ',';
            AppendNumber(output, point.position_metres[axis]);
        }
        output << "]}";
    }
    output << ']';
}

}  // namespace

std::string SerializeHello(std::string_view session_id, std::string_view producer,
                           const CameraInfo& camera) {
    RequireText(session_id, "session_id");
    RequireText(producer, "producer");
    RequireText(camera.id, "camera.id");
    if (camera.width == 0 || camera.height == 0 || camera.fps == 0) {
        throw std::invalid_argument("camera dimensions and fps must be positive");
    }
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << "{\"type\":\"hello\",\"boundary_version\":1,\"session_id\":"
           << JsonString(session_id) << ",\"producer\":" << JsonString(producer)
           << ",\"camera\":{\"camera_type\":\"monocular\",\"id\":"
           << JsonString(camera.id) << ",\"width\":" << camera.width
           << ",\"height\":" << camera.height << ",\"fps\":" << camera.fps
           << "}}";
    return output.str();
}

std::string SerializeTracking(std::string_view session_id,
                              const slam::TrackingResult& result,
                              camera::ImageFrame::Timestamp timestamp_origin) {
    RequireText(session_id, "session_id");
    if (result.frame_id > kMaxSafeJsonInteger) {
        throw std::invalid_argument("frame ID exceeds JSON-safe integer maximum");
    }
    if (result.timestamp < timestamp_origin) {
        throw std::invalid_argument("frame timestamp precedes session origin");
    }
    if (result.state == slam::TrackingState::kTracking && !result.pose.has_value()) {
        throw std::invalid_argument("tracking state requires a pose");
    }
    const double timestamp_seconds =
        std::chrono::duration<double>(result.timestamp - timestamp_origin).count();
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << "{\"type\":\"tracking_frame\",\"boundary_version\":1,\"session_id\":"
           << JsonString(session_id) << ",\"frame_id\":" << result.frame_id
           << ",\"timestamp_seconds\":";
    AppendNumber(output, timestamp_seconds);
    output << ",\"tracking_state\":" << JsonString(StateName(result.state))
           << ",\"pose\":";
    if (result.pose.has_value()) {
        AppendPose(output, *result.pose);
    } else {
        output << "null";
    }
    output << '}';
    return output.str();
}

std::string SerializePointCloudDelta(std::string_view session_id, std::uint64_t frame_id,
                                     camera::ImageFrame::Timestamp timestamp,
                                     camera::ImageFrame::Timestamp timestamp_origin,
                                     const slam::PointCloudDelta& delta) {
    RequireText(session_id, "session_id");
    if (frame_id > kMaxSafeJsonInteger) {
        throw std::invalid_argument("frame ID exceeds JSON-safe integer maximum");
    }
    if (timestamp < timestamp_origin) {
        throw std::invalid_argument("point-cloud timestamp precedes session origin");
    }
    std::unordered_set<std::uint64_t> point_ids;
    const auto require_unique = [&point_ids](std::uint64_t id) {
        if (!point_ids.insert(id).second) {
            throw std::invalid_argument("map point ID occurs more than once in delta");
        }
    };
    for (const auto& point : delta.add) require_unique(point.id);
    for (const auto& point : delta.update) require_unique(point.id);
    for (const auto id : delta.remove) require_unique(id);
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << "{\"type\":\"pointcloud_delta\",\"boundary_version\":1,\"session_id\":"
           << JsonString(session_id) << ",\"frame_id\":" << frame_id
           << ",\"timestamp_seconds\":";
    AppendNumber(output, std::chrono::duration<double>(timestamp - timestamp_origin).count());
    output << ",\"add\":";
    AppendMapPoints(output, delta.add);
    output << ",\"update\":";
    AppendMapPoints(output, delta.update);
    output << ",\"remove\":[";
    for (std::size_t index = 0; index < delta.remove.size(); ++index) {
        if (index != 0) output << ',';
        if (delta.remove[index] > kMaxSafeJsonInteger) {
            throw std::invalid_argument("removed map point ID exceeds JSON-safe integer maximum");
        }
        output << delta.remove[index];
    }
    output << "]}";
    return output.str();
}

std::string SerializeSessionEnd(std::string_view session_id, std::string_view reason) {
    RequireText(session_id, "session_id");
    RequireText(reason, "reason");
    return "{\"type\":\"session_end\",\"boundary_version\":1,\"session_id\":" +
           JsonString(session_id) + ",\"reason\":" + JsonString(reason) + '}';
}

std::vector<std::uint8_t> FramePayload(std::string_view payload) {
    if (payload.size() > kMaxPayloadBytes) {
        throw std::invalid_argument("boundary payload exceeds 1 MiB maximum");
    }
    const auto length = static_cast<std::uint32_t>(payload.size());
    std::vector<std::uint8_t> frame{
        static_cast<std::uint8_t>((length >> 24) & 0xff),
        static_cast<std::uint8_t>((length >> 16) & 0xff),
        static_cast<std::uint8_t>((length >> 8) & 0xff),
        static_cast<std::uint8_t>(length & 0xff)};
    frame.insert(frame.end(), payload.begin(), payload.end());
    return frame;
}

class Publisher::Impl final {
   public:
    explicit Impl(PublisherConfig config) : config_(std::move(config)) {
        RequireText(config_.socket_path, "socket_path");
        static_cast<void>(FramePayload(
            SerializeHello(config_.session_id, config_.producer, config_.camera)));
        if (config_.timestamp_origin.count() < 0 || config_.send_timeout.count() <= 0) {
            throw std::invalid_argument(
                "timestamp origin must be non-negative and send timeout must be positive");
        }
    }

    ~Impl() { Close(); }

    bool Connect() {
        if (fd_ >= 0) return true;
        if (ended_) return Fail("cannot reconnect an ended boundary session");
        if (config_.socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
            return Fail("Unix socket path is too long");
        }
        const int candidate = socket(AF_UNIX, SOCK_STREAM, 0);
        if (candidate < 0) return FailErrno("failed to create Unix socket");
#ifdef SO_NOSIGPIPE
        const int enabled = 1;
        static_cast<void>(setsockopt(candidate, SOL_SOCKET, SO_NOSIGPIPE, &enabled,
                                     sizeof(enabled)));
#endif
        const auto milliseconds = config_.send_timeout.count();
        timeval timeout{static_cast<time_t>(milliseconds / 1000),
                        static_cast<suseconds_t>((milliseconds % 1000) * 1000)};
        if (setsockopt(candidate, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) != 0) {
            close(candidate);
            return FailErrno("failed to configure socket send timeout");
        }
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        std::memcpy(address.sun_path, config_.socket_path.c_str(),
                    config_.socket_path.size() + 1);
        if (connect(candidate, reinterpret_cast<const sockaddr*>(&address),
                    sizeof(address)) != 0) {
            close(candidate);
            return FailErrno("failed to connect to Rust SLAM listener");
        }
        fd_ = candidate;
        if (!Send(SerializeHello(config_.session_id, config_.producer, config_.camera))) {
            return false;
        }
        last_error_.clear();
        return true;
    }

    bool PublishTracking(const slam::TrackingResult& result) {
        if (fd_ < 0) return Fail("SLAM boundary is not connected");
        if (ended_) return Fail("SLAM boundary session has ended");
        if (last_frame_.has_value() && result.frame_id < *last_frame_) {
            return Fail("tracking frame ID regressed");
        }
        if (last_timestamp_.has_value() && result.timestamp < *last_timestamp_) {
            return Fail("tracking timestamp regressed");
        }
        std::string payload;
        try {
            payload = SerializeTracking(config_.session_id, result,
                                        config_.timestamp_origin);
        } catch (const std::exception& error) {
            return Fail(error.what());
        }
        if (!Send(payload)) return false;
        last_frame_ = result.frame_id;
        last_timestamp_ = result.timestamp;
        return true;
    }

    bool PublishPointCloud(std::uint64_t frame_id,
                           camera::ImageFrame::Timestamp timestamp,
                           const slam::PointCloudDelta& delta) {
        if (fd_ < 0) return Fail("SLAM boundary is not connected");
        if (ended_) return Fail("SLAM boundary session has ended");
        if (last_pointcloud_frame_.has_value() && frame_id < *last_pointcloud_frame_) {
            return Fail("point-cloud frame ID regressed");
        }
        if (last_pointcloud_timestamp_.has_value() && timestamp < *last_pointcloud_timestamp_) {
            return Fail("point-cloud timestamp regressed");
        }
        std::string payload;
        try {
            payload = SerializePointCloudDelta(config_.session_id, frame_id, timestamp,
                                               config_.timestamp_origin, delta);
        } catch (const std::exception& error) {
            return Fail(error.what());
        }
        if (!Send(payload)) return false;
        last_pointcloud_frame_ = frame_id;
        last_pointcloud_timestamp_ = timestamp;
        return true;
    }

    bool EndSession(std::string_view reason) {
        if (fd_ < 0) return Fail("SLAM boundary is not connected");
        if (ended_) return Fail("SLAM boundary session has already ended");
        std::string payload;
        try {
            payload = SerializeSessionEnd(config_.session_id, reason);
        } catch (const std::exception& error) {
            return Fail(error.what());
        }
        const bool sent = Send(payload);
        ended_ = true;
        Close();
        return sent;
    }

    void Close() noexcept {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    [[nodiscard]] bool connected() const noexcept { return fd_ >= 0; }
    [[nodiscard]] const std::string& last_error() const noexcept { return last_error_; }

   private:
    bool Send(std::string_view payload) {
        std::vector<std::uint8_t> frame;
        try {
            frame = FramePayload(payload);
        } catch (const std::exception& error) {
            return Fail(error.what());
        }
        std::size_t offset = 0;
        while (offset < frame.size()) {
            int flags = 0;
#ifdef MSG_NOSIGNAL
            flags = MSG_NOSIGNAL;
#endif
            const ssize_t written =
                send(fd_, frame.data() + offset, frame.size() - offset, flags);
            if (written > 0) {
                offset += static_cast<std::size_t>(written);
                continue;
            }
            if (written < 0 && errno == EINTR) continue;
            FailErrno("failed to write SLAM boundary frame");
            Close();
            return false;
        }
        return true;
    }

    bool Fail(std::string message) {
        last_error_ = std::move(message);
        return false;
    }

    bool FailErrno(const char* context) {
        return Fail(std::string(context) + ": " + std::strerror(errno));
    }

    PublisherConfig config_;
    int fd_{-1};
    bool ended_{false};
    std::optional<std::uint64_t> last_frame_;
    std::optional<camera::ImageFrame::Timestamp> last_timestamp_;
    std::optional<std::uint64_t> last_pointcloud_frame_;
    std::optional<camera::ImageFrame::Timestamp> last_pointcloud_timestamp_;
    std::string last_error_;
};

Publisher::Publisher(PublisherConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
Publisher::~Publisher() = default;
bool Publisher::Connect() { return impl_->Connect(); }
bool Publisher::PublishTracking(const slam::TrackingResult& result) {
    return impl_->PublishTracking(result);
}
bool Publisher::PublishPointCloud(std::uint64_t frame_id,
                                  camera::ImageFrame::Timestamp timestamp,
                                  const slam::PointCloudDelta& delta) {
    return impl_->PublishPointCloud(frame_id, timestamp, delta);
}
bool Publisher::EndSession(std::string_view reason) { return impl_->EndSession(reason); }
void Publisher::Close() noexcept { impl_->Close(); }
bool Publisher::connected() const noexcept { return impl_->connected(); }
const std::string& Publisher::last_error() const noexcept { return impl_->last_error(); }

}  // namespace slam_remote::boundary
