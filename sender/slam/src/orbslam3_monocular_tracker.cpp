#include "slam_remote/orbslam3/monocular_tracker.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include <GL/glew.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "System.h"
#include "MapPoint.h"
#include "Tracking.h"

namespace slam_remote::orbslam3 {
namespace {

int OpenCvType(camera::PixelFormat format) {
    switch (format) {
        case camera::PixelFormat::kGray8:
            return CV_8UC1;
        case camera::PixelFormat::kBgr8:
        case camera::PixelFormat::kRgb8:
            return CV_8UC3;
    }
    throw std::invalid_argument("unsupported camera pixel format");
}

slam::TrackingState ConvertTrackingState(int state) noexcept {
    switch (state) {
        case ORB_SLAM3::Tracking::OK:
        case ORB_SLAM3::Tracking::OK_KLT:
            return slam::TrackingState::kTracking;
        case ORB_SLAM3::Tracking::RECENTLY_LOST:
            return slam::TrackingState::kRelocalizing;
        case ORB_SLAM3::Tracking::LOST:
            return slam::TrackingState::kLost;
        default:
            return slam::TrackingState::kInitializing;
    }
}

slam::RigidTransform CopyTransform(const Sophus::SE3f& transform) {
    const Eigen::Matrix3f rotation = transform.rotationMatrix();
    const Eigen::Vector3f translation = transform.translation();
    return {{{rotation(0, 0), rotation(0, 1), rotation(0, 2),
              rotation(1, 0), rotation(1, 1), rotation(1, 2),
              rotation(2, 0), rotation(2, 1), rotation(2, 2)}},
            {{translation.x(), translation.y(), translation.z()}}};
}

}  // namespace

class MonocularTracker::Impl final {
   public:
    explicit Impl(MonocularTrackerConfig config) {
        if (config.vocabulary_path.empty() || config.settings_path.empty()) {
            throw std::invalid_argument(
                "ORB-SLAM3 vocabulary and settings paths are required");
        }
        system_ = std::make_unique<ORB_SLAM3::System>(
            config.vocabulary_path, config.settings_path,
            ORB_SLAM3::System::MONOCULAR, config.enable_viewer);
    }

    ~Impl() { Shutdown(); }

    slam::TrackingResult Track(const camera::ImageFrame& frame) {
        if (shut_down_) {
            throw std::logic_error("cannot track after ORB-SLAM3 shutdown");
        }
        cv::Mat source(static_cast<int>(frame.height()), static_cast<int>(frame.width()),
                       OpenCvType(frame.pixel_format()),
                       const_cast<std::uint8_t*>(frame.pixels().data()));
        cv::Mat image;
        switch (frame.pixel_format()) {
            case camera::PixelFormat::kGray8:
                image = source;
                break;
            case camera::PixelFormat::kBgr8:
                cv::cvtColor(source, image, cv::COLOR_BGR2GRAY);
                break;
            case camera::PixelFormat::kRgb8:
                cv::cvtColor(source, image, cv::COLOR_RGB2GRAY);
                break;
        }
        const double timestamp_seconds =
            std::chrono::duration<double>(frame.timestamp()).count();
        const Sophus::SE3f camera_from_world =
            system_->TrackMonocular(image, timestamp_seconds);
        const auto state = ConvertTrackingState(system_->GetTrackingState());
        if (state != slam::TrackingState::kTracking) {
            return slam::MakeTrackingResult(frame, state, std::nullopt);
        }
        return slam::MakeTrackingResult(frame, state,
                                        CopyTransform(camera_from_world));
    }

    void Reset() {
        if (shut_down_) {
            throw std::logic_error("cannot reset after ORB-SLAM3 shutdown");
        }
        system_->ResetActiveMap();
    }

    std::vector<slam::MapPoint> ActiveMapPoints() {
        if (shut_down_) {
            throw std::logic_error("cannot read map points after ORB-SLAM3 shutdown");
        }
        std::vector<slam::MapPoint> snapshot;
        const auto points = system_->GetActiveMapPoints();
        snapshot.reserve(points.size());
        for (ORB_SLAM3::MapPoint* point : points) {
            if (point == nullptr || point->isBad()) continue;
            const Eigen::Vector3f position = point->GetWorldPos();
            snapshot.push_back({point->mnId,
                                {static_cast<double>(position.x()),
                                 static_cast<double>(position.y()),
                                 static_cast<double>(position.z())}});
        }
        return snapshot;
    }

    void Shutdown() noexcept {
        if (!shut_down_ && system_) {
            system_->Shutdown();
            shut_down_ = true;
        }
    }

   private:
    std::unique_ptr<ORB_SLAM3::System> system_;
    bool shut_down_{false};
};

MonocularTracker::MonocularTracker(MonocularTrackerConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

MonocularTracker::~MonocularTracker() = default;

slam::TrackingResult MonocularTracker::Track(const camera::ImageFrame& frame) {
    return impl_->Track(frame);
}

std::vector<slam::MapPoint> MonocularTracker::ActiveMapPoints() {
    return impl_->ActiveMapPoints();
}

void MonocularTracker::Reset() { impl_->Reset(); }

void MonocularTracker::Shutdown() noexcept { impl_->Shutdown(); }

}  // namespace slam_remote::orbslam3
