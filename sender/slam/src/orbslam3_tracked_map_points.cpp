#include "slam_remote/orbslam3/tracked_map_points.hpp"

#include <algorithm>
#include <cstdint>

#include <GL/glew.h>
#include "MapPoint.h"
#include "System.h"

namespace slam_remote::orbslam3 {

std::vector<telemetry::MapPoint> CopyTrackedMapPoints(ORB_SLAM3::System& system) {
    const auto backend_points = system.GetTrackedMapPoints();
    std::vector<telemetry::MapPoint> points;
    points.reserve(backend_points.size());

    for (auto* point : backend_points) {
        if (point == nullptr || point->isBad()) {
            continue;
        }
        const Eigen::Vector3f position = point->GetWorldPos();
        points.push_back({static_cast<std::uint64_t>(point->mnId),
                          static_cast<double>(position.x()),
                          static_cast<double>(position.y()),
                          static_cast<double>(position.z())});
    }

    std::sort(points.begin(), points.end(),
              [](const telemetry::MapPoint& left, const telemetry::MapPoint& right) {
                  return left.id < right.id;
              });
    return points;
}

}  // namespace slam_remote::orbslam3
