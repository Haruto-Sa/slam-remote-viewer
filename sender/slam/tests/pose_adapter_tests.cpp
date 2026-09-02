#include "slam_remote/slam/pose.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using slam_remote::camera::ImageFrame;
using slam_remote::camera::PixelFormat;
using slam_remote::slam::RigidTransform;
using slam_remote::slam::TrackingState;

constexpr double kTolerance = 1e-9;

void Check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void CheckNear(double actual, double expected, const std::string& message) {
    Check(std::abs(actual - expected) <= kTolerance, message);
}

ImageFrame FixtureFrame(std::uint64_t frame_id = 42,
                        std::chrono::nanoseconds timestamp =
                            std::chrono::nanoseconds(1'234'567'890)) {
    return ImageFrame(frame_id, timestamp, 2, 2, PixelFormat::kGray8,
                      std::vector<std::uint8_t>(4, 0));
}

RigidTransform IdentityTcw() {
    return {{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}},
            {{0.0, 0.0, 0.0}}};
}

void TestIdentity() {
    const auto pose = slam_remote::slam::ConvertTcwToTwc(IdentityTcw());
    for (double value : pose.position_metres) {
        CheckNear(value, 0.0, "identity translation must remain zero");
    }
    CheckNear(pose.orientation_xyzw[0], 0.0, "identity qx");
    CheckNear(pose.orientation_xyzw[1], 0.0, "identity qy");
    CheckNear(pose.orientation_xyzw[2], 0.0, "identity qz");
    CheckNear(pose.orientation_xyzw[3], 1.0, "identity qw");
}

void TestTranslationIsInverted() {
    auto transform = IdentityTcw();
    transform.translation_metres = {1.0, -2.0, 3.5};
    const auto pose = slam_remote::slam::ConvertTcwToTwc(transform);
    CheckNear(pose.position_metres[0], -1.0, "Twc x translation");
    CheckNear(pose.position_metres[1], 2.0, "Twc y translation");
    CheckNear(pose.position_metres[2], -3.5, "Twc z translation");
}

void TestKnownQuarterTurns() {
    constexpr double kHalfSqrtTwo = 0.7071067811865475244;
    const RigidTransform positive_z_tcw{
        {{0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
        {{0.0, 0.0, 0.0}}};
    const auto z_pose = slam_remote::slam::ConvertTcwToTwc(positive_z_tcw);
    CheckNear(z_pose.orientation_xyzw[0], 0.0, "quarter-turn z qx");
    CheckNear(z_pose.orientation_xyzw[1], 0.0, "quarter-turn z qy");
    CheckNear(z_pose.orientation_xyzw[2], -kHalfSqrtTwo, "inverted quarter-turn z qz");
    CheckNear(z_pose.orientation_xyzw[3], kHalfSqrtTwo, "quarter-turn z qw");

    const RigidTransform positive_x_tcw{
        {{1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0}},
        {{0.0, 0.0, 0.0}}};
    const auto x_pose = slam_remote::slam::ConvertTcwToTwc(positive_x_tcw);
    CheckNear(x_pose.orientation_xyzw[0], -kHalfSqrtTwo, "inverted quarter-turn x qx");
    CheckNear(x_pose.orientation_xyzw[1], 0.0, "quarter-turn x qy");
    CheckNear(x_pose.orientation_xyzw[2], 0.0, "quarter-turn x qz");
    CheckNear(x_pose.orientation_xyzw[3], kHalfSqrtTwo, "quarter-turn x qw");
}

void TestMetadataAndTrackingLoss() {
    const auto frame = FixtureFrame();
    const auto tracked = slam_remote::slam::MakeTrackingResult(
        frame, TrackingState::kTracking, IdentityTcw());
    Check(tracked.frame_id == frame.frame_id(), "frame ID must come from input");
    Check(tracked.timestamp == frame.timestamp(), "timestamp must come from input");
    Check(tracked.pose.has_value(), "tracking must emit the converted pose");

    for (const auto state : {TrackingState::kInitializing, TrackingState::kLost,
                             TrackingState::kRelocalizing}) {
        const auto result =
            slam_remote::slam::MakeTrackingResult(frame, state, IdentityTcw());
        Check(!result.pose.has_value(), "invalid tracking must not emit a cached pose");
    }
}

void TestDeterministicFixtureMetadata() {
    const auto frame = FixtureFrame(7, std::chrono::nanoseconds(99));
    const auto first = slam_remote::slam::MakeTrackingResult(
        frame, TrackingState::kTracking, IdentityTcw());
    const auto second = slam_remote::slam::MakeTrackingResult(
        frame, TrackingState::kTracking, IdentityTcw());
    Check(first.frame_id == second.frame_id && first.timestamp == second.timestamp &&
              first.state == second.state && first.pose.has_value() &&
              second.pose.has_value() &&
              first.pose->position_metres == second.pose->position_metres &&
              first.pose->orientation_xyzw == second.pose->orientation_xyzw,
          "replayed fixture metadata and pose must be deterministic");
}

void TestInvalidTransformsAreRejected() {
    auto transform = IdentityTcw();
    transform.translation_metres[0] = std::numeric_limits<double>::infinity();
    bool rejected = false;
    try {
        static_cast<void>(slam_remote::slam::ConvertTcwToTwc(transform));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Check(rejected, "non-finite transforms must be rejected");

    transform = IdentityTcw();
    transform.rotation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    rejected = false;
    try {
        static_cast<void>(slam_remote::slam::ConvertTcwToTwc(transform));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Check(rejected, "non-rigid rotations must be rejected");

    rejected = false;
    try {
        static_cast<void>(slam_remote::slam::MakeTrackingResult(
            FixtureFrame(), TrackingState::kTracking, std::nullopt));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Check(rejected, "tracking without Tcw must be rejected");
}

}  // namespace

int main() {
    try {
        TestIdentity();
        TestTranslationIsInverted();
        TestKnownQuarterTurns();
        TestMetadataAndTrackingLoss();
        TestDeterministicFixtureMetadata();
        TestInvalidTransformsAreRejected();
    } catch (const std::exception& error) {
        std::cerr << "pose adapter test failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "pose adapter tests passed\n";
    return 0;
}
