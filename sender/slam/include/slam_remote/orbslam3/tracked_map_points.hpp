#pragma once

#include <vector>

#include "slam_remote/telemetry/point_cloud_delta_reducer.hpp"

namespace ORB_SLAM3 {
class MapPoint;
class System;
}  // namespace ORB_SLAM3

namespace slam_remote::orbslam3 {

/// Copies valid points from ORB-SLAM3's public tracked-point snapshot. Null and
/// bad points are omitted; backend pointers never escape this function.
std::vector<telemetry::MapPoint> CopyTrackedMapPoints(ORB_SLAM3::System& system);

}  // namespace slam_remote::orbslam3
