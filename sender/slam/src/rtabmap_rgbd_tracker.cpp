#include "slam_remote/rtabmap/rgbd_tracker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Signature.h>

namespace slam_remote::rtabmap_backend {
namespace {
slam::RigidTransform ToRigid(const rtabmap::Transform& t) {
    return {{t.r11(), t.r12(), t.r13(), t.r21(), t.r22(), t.r23(),
             t.r31(), t.r32(), t.r33()}, {t.x(), t.y(), t.z()}};
}
rtabmap::ParametersMap Parameters() {
    // Built-in F2M odometry and GTSAM optimization: no ORB-SLAM dependency.
    return {{"Odom/Strategy", "0"}, {"Optimizer/Strategy", "2"},
            {"Odom/ResetCountdown", "0"}, {"RGBD/Enabled", "true"},
            {"OdomF2M/BundleAdjustment", "0"}, {"Vis/FeatureType", "8"},
            {"Kp/DetectorStrategy", "8"}, {"RGBD/CreateOccupancyGrid", "false"}};
}
}  // namespace

class RgbdTracker::Impl final {
 public:
    explicit Impl(RgbdTrackerConfig c) : config(c) {
        if (c.width <= 0 || c.height <= 0 || !std::isfinite(c.fx) ||
            !std::isfinite(c.fy) || c.fx <= 0 || c.fy <= 0 ||
            !std::isfinite(c.cx) || !std::isfinite(c.cy) ||
            c.cx < 0 || c.cx >= c.width || c.cy < 0 || c.cy >= c.height) {
            throw std::invalid_argument("invalid RGB-D camera calibration");
        }
        if (!rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeGTSAM)) {
            throw std::runtime_error("RTAB-Map must be built with GTSAM support");
        }
        // Identity local transform makes base_link the optical frame. Thus
        // odometry and map correction are Twc in right/down/forward axes,
        // matching the canonical camera frame without an extra ROS rotation.
        model = rtabmap::CameraModel(c.fx, c.fy, c.cx, c.cy,
                                     rtabmap::Transform::getIdentity(), 0,
                                     cv::Size(c.width, c.height));
        odometry.reset(rtabmap::Odometry::create(Parameters()));
        if (!odometry) throw std::runtime_error("RTAB-Map odometry unavailable");
        mapping.init(Parameters());
    }
    RgbdTrackerConfig config;
    rtabmap::CameraModel model;
    std::unique_ptr<rtabmap::Odometry> odometry;
    rtabmap::Rtabmap mapping;
    std::optional<std::uint64_t> last_frame;
    camera::ImageFrame::Timestamp last_timestamp{};
    bool tracked{false};
};

RgbdTracker::RgbdTracker(RgbdTrackerConfig config) : impl_(std::make_unique<Impl>(config)) {}
RgbdTracker::~RgbdTracker() = default;

slam::TrackingResult RgbdTracker::Track(std::uint64_t frame_id,
                                      camera::ImageFrame::Timestamp timestamp,
                                      const cv::Mat& color, const cv::Mat& depth) {
    auto& state = *impl_;
    if (frame_id >= static_cast<std::uint64_t>(std::numeric_limits<int>::max()) ||
        timestamp.count() < 0 || (state.last_frame &&
        (frame_id <= *state.last_frame || timestamp <= state.last_timestamp))) {
        throw std::invalid_argument("RGB-D frame IDs and timestamps must increase");
    }
    if (color.type() != CV_8UC3 || depth.type() != CV_32FC1 ||
        color.cols != state.config.width || color.rows != state.config.height ||
        depth.size() != color.size()) {
        throw std::invalid_argument("expected registered BGR8 and float32 depth at calibrated size");
    }
    for (int row = 0; row < depth.rows; ++row) {
        const float* values = depth.ptr<float>(row);
        for (int col = 0; col < depth.cols; ++col) {
            if (!std::isfinite(values[col]) || values[col] < 0) {
                throw std::invalid_argument("depth must be finite nonnegative metres");
            }
        }
    }
    rtabmap::SensorData data(color, depth, state.model, static_cast<int>(frame_id + 1),
                            std::chrono::duration<double>(timestamp).count());
    const auto odom = state.odometry->process(data);
    state.last_frame = frame_id;
    state.last_timestamp = timestamp;
    if (odom.isNull()) {
        return {frame_id, timestamp, state.tracked ? slam::TrackingState::kLost :
                slam::TrackingState::kInitializing, std::nullopt};
    }
    state.mapping.process(data, odom);
    const auto twc = state.mapping.getMapCorrection() * odom;
    if (twc.isNull()) throw std::runtime_error("RTAB-Map returned a null map pose");
    auto pose = slam::ConvertTwcToPose(ToRigid(twc));
    state.tracked = true;
    return {frame_id, timestamp, slam::TrackingState::kTracking, pose};
}

std::vector<slam::MapPoint> RgbdTracker::MapPoints() const {
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> links;
    std::map<int, rtabmap::Signature> signatures;
    impl_->mapping.getGraph(poses, links, true, true, &signatures,
                            false, false, false, false, true, false);
    std::vector<slam::MapPoint> result;
    for (const auto& entry : signatures) {
        if (entry.first <= 0) continue;
        const auto pose = poses.find(entry.first);
        if (pose == poses.end() || pose->second.isNull()) continue;
        const auto& words = entry.second.getWords3();
        constexpr std::uint64_t kSlotsPerNode = 1ULL << 20;
        if (words.size() >= kSlotsPerNode) throw std::runtime_error("too many features per map node");
        const auto t = ToRigid(pose->second);
        for (std::size_t i = 0; i < words.size(); ++i) {
            const auto& p = words[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= 0) continue;
            if (result.size() == 100'000) throw std::runtime_error("RTAB-Map point export exceeds 100000 points");
            std::array<double, 3> world{};
            for (std::size_t axis = 0; axis < 3; ++axis) {
                world[axis] = t.rotation[axis * 3] * p.x +
                    t.rotation[axis * 3 + 1] * p.y + t.rotation[axis * 3 + 2] * p.z +
                    t.translation_metres[axis];
            }
            // Observation identity, not visual-word identity: repeated words
            // in different keyframes must not overwrite one another.
            result.push_back({static_cast<std::uint64_t>(entry.first) * kSlotsPerNode + i, world});
        }
    }
    return result;
}
}  // namespace slam_remote::rtabmap_backend
