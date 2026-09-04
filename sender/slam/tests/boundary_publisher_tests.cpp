#include "slam_remote/boundary/publisher.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

using slam_remote::boundary::CameraInfo;
using slam_remote::boundary::Publisher;
using slam_remote::boundary::PublisherConfig;
using slam_remote::slam::CameraPose;
using slam_remote::slam::PointCloudDelta;
using slam_remote::slam::TrackingResult;
using slam_remote::slam::TrackingState;

void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::string ReadFrame(int fd) {
    std::array<std::uint8_t, 4> length_bytes{};
    std::size_t offset = 0;
    while (offset < length_bytes.size()) {
        const ssize_t count = read(fd, length_bytes.data() + offset,
                                   length_bytes.size() - offset);
        if (count <= 0) throw std::runtime_error("failed to read frame length");
        offset += static_cast<std::size_t>(count);
    }
    const std::uint32_t length =
        (static_cast<std::uint32_t>(length_bytes[0]) << 24) |
        (static_cast<std::uint32_t>(length_bytes[1]) << 16) |
        (static_cast<std::uint32_t>(length_bytes[2]) << 8) | length_bytes[3];
    std::string payload(length, '\0');
    offset = 0;
    while (offset < payload.size()) {
        const ssize_t count = read(fd, payload.data() + offset, payload.size() - offset);
        if (count <= 0) throw std::runtime_error("failed to read frame payload");
        offset += static_cast<std::size_t>(count);
    }
    return payload;
}

void TestSerializationAndValidation() {
    const CameraInfo camera{"fixture-camera", 1280, 720, 30};
    const auto hello = slam_remote::boundary::SerializeHello(
        "fixture-session", "fixture-slam", camera);
    Check(hello.find("\"type\":\"hello\"") != std::string::npos,
          "hello type must be serialized");
    Check(hello.find("\"camera_type\":\"monocular\"") != std::string::npos,
          "monocular camera type must be serialized");

    const auto origin = std::chrono::seconds(4);
    const TrackingResult tracked{
        7, origin + std::chrono::milliseconds(250), TrackingState::kTracking,
        CameraPose{{1.0, 2.0, 3.0}, {0.0, 0.0, 0.0, 1.0}}};
    const auto tracking = slam_remote::boundary::SerializeTracking(
        "fixture-session", tracked, origin);
    Check(tracking.find("\"frame_id\":7") != std::string::npos,
          "frame ID must be serialized");
    Check(tracking.find("\"timestamp_seconds\":0.25") != std::string::npos,
          "session-relative timestamp must be serialized");
    Check(tracking.find("\"tracking_state\":\"tracking\"") != std::string::npos,
          "tracking state must be serialized");
    const PointCloudDelta delta{{{1001, {0.1, 0.2, 1.4}}},
                                {{1002, {0.2, 0.3, 1.5}}}, {1003}};
    const auto pointcloud = slam_remote::boundary::SerializePointCloudDelta(
        "fixture-session", 7, origin + std::chrono::milliseconds(250), origin, delta);
    Check(pointcloud.find("\"type\":\"pointcloud_delta\"") != std::string::npos,
          "point-cloud type must be serialized");
    Check(pointcloud.find("\"add\":[{\"id\":1001") != std::string::npos,
          "point-cloud add must be serialized");
    Check(pointcloud.find("\"remove\":[1003]") != std::string::npos,
          "point-cloud remove must be serialized");

    bool duplicate_rejected = false;
    try {
        static_cast<void>(slam_remote::boundary::SerializePointCloudDelta(
            "fixture-session", 7, origin, origin,
            PointCloudDelta{{{1001, {0.0, 0.0, 0.0}}}, {}, {1001}}));
    } catch (const std::invalid_argument&) {
        duplicate_rejected = true;
    }
    Check(duplicate_rejected, "duplicate point IDs must be rejected before writing");

    const auto framed = slam_remote::boundary::FramePayload(tracking);
    Check(framed.size() == tracking.size() + 4, "frame must include four length bytes");
    Check(framed[0] == 0 && framed[1] == 0,
          "small payload must use big-endian length prefix");

    auto invalid = tracked;
    invalid.pose->position_metres[0] = std::numeric_limits<double>::infinity();
    bool rejected = false;
    try {
        static_cast<void>(slam_remote::boundary::SerializeTracking(
            "fixture-session", invalid, origin));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Check(rejected, "non-finite pose must be rejected before writing");

    invalid = tracked;
    invalid.frame_id = slam_remote::boundary::kMaxSafeJsonInteger + 1;
    rejected = false;
    try {
        static_cast<void>(slam_remote::boundary::SerializeTracking(
            "fixture-session", invalid, origin));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Check(rejected, "unsafe JSON frame ID must be rejected before writing");

    invalid = tracked;
    invalid.pose.reset();
    rejected = false;
    try {
        static_cast<void>(slam_remote::boundary::SerializeTracking(
            "fixture-session", invalid, origin));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Check(rejected, "tracking without pose must be rejected before writing");

    rejected = false;
    try {
        static_cast<void>(slam_remote::boundary::SerializeHello(
            std::string("bad\xc0\x80", 5), "producer", camera));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Check(rejected, "invalid UTF-8 must be rejected before writing");
}

void TestCompleteSocketSession() {
    const std::string path = "/private/tmp/slam-boundary-test-" +
                             std::to_string(static_cast<long long>(getpid())) + ".sock";
    unlink(path.c_str());
    const int listener = socket(AF_UNIX, SOCK_STREAM, 0);
    Check(listener >= 0, "test listener socket must be created");
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
    Check(bind(listener, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0,
          "test listener must bind");
    Check(listen(listener, 1) == 0, "test listener must listen");

    std::vector<std::string> received;
    std::string server_error;
    std::thread server([&] {
        try {
            const int connection = accept(listener, nullptr, nullptr);
            if (connection < 0) throw std::runtime_error("test listener accept failed");
            received.push_back(ReadFrame(connection));
            received.push_back(ReadFrame(connection));
            received.push_back(ReadFrame(connection));
            received.push_back(ReadFrame(connection));
            close(connection);
        } catch (const std::exception& error) {
            server_error = error.what();
        }
    });

    const auto origin = std::chrono::seconds(2);
    Publisher publisher(PublisherConfig{path, "fixture-session", "fixture-slam",
                                        {"fixture-camera", 1280, 720, 30}, origin,
                                        std::chrono::milliseconds(250)});
    Check(publisher.Connect(), publisher.last_error());
    const TrackingResult tracked{
        7, origin + std::chrono::milliseconds(250), TrackingState::kTracking,
        CameraPose{{1.0, 2.0, 3.0}, {0.0, 0.0, 0.0, 1.0}}};
    Check(publisher.PublishTracking(tracked), publisher.last_error());
    Check(publisher.PublishPointCloud(
              7, origin + std::chrono::milliseconds(250),
              PointCloudDelta{{{1001, {0.1, 0.2, 1.4}}}, {}, {}}),
          publisher.last_error());
    Check(publisher.EndSession(), publisher.last_error());
    server.join();
    close(listener);
    unlink(path.c_str());

    Check(server_error.empty(), server_error);
    Check(received.size() == 4, "hello, tracking, point-cloud, and session_end must be sent");
    Check(received[0].find("\"type\":\"hello\"") != std::string::npos,
          "hello must be first");
    Check(received[1].find("\"type\":\"tracking_frame\"") != std::string::npos,
          "tracking must be second");
    Check(received[2].find("\"type\":\"pointcloud_delta\"") != std::string::npos,
          "point-cloud must be third");
    Check(received[3].find("\"type\":\"session_end\"") != std::string::npos,
          "session_end must be last");
    Check(!publisher.connected(), "publisher must close after session_end");
}

void TestMissingConsumerDoesNotQueue() {
    Publisher publisher(PublisherConfig{
        "/private/tmp/slam-boundary-no-consumer.sock", "session", "producer",
        {"camera", 640, 480, 30}, std::chrono::nanoseconds(0),
        std::chrono::milliseconds(20)});
    Check(!publisher.Connect(), "connect without a listener must fail");
    Check(!publisher.connected(), "failed connection must retain no socket");
}

}  // namespace

int main() {
    try {
        TestSerializationAndValidation();
        TestCompleteSocketSession();
        TestMissingConsumerDoesNotQueue();
    } catch (const std::exception& error) {
        std::cerr << "boundary publisher test failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "boundary publisher tests passed\n";
    return 0;
}
