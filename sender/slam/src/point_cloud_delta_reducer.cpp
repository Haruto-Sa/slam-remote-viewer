#include "slam_remote/telemetry/point_cloud_delta_reducer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <tuple>

namespace slam_remote::telemetry {
namespace {

constexpr std::uint64_t kMaximumJsonInteger = 9'007'199'254'740'991ULL;

using Voxel = std::tuple<std::int64_t, std::int64_t, std::int64_t>;

std::int64_t VoxelCoordinate(double coordinate, double voxel_size) {
    const double value = std::floor(coordinate / voxel_size);
    if (value < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
        value > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
        throw std::invalid_argument("map point exceeds the supported voxel range");
    }
    return static_cast<std::int64_t>(value);
}

Voxel ToVoxel(const MapPoint& point, double voxel_size) {
    return {VoxelCoordinate(point.x, voxel_size), VoxelCoordinate(point.y, voxel_size),
            VoxelCoordinate(point.z, voxel_size)};
}

void ValidatePoint(const MapPoint& point) {
    if (point.id > kMaximumJsonInteger) {
        throw std::invalid_argument("map point ID exceeds the Protocol v1 JSON limit");
    }
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        throw std::invalid_argument("map point coordinates must be finite");
    }
}

double SquaredDistance(const MapPoint& left, const MapPoint& right) noexcept {
    const double dx = left.x - right.x;
    const double dy = left.y - right.y;
    const double dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

}  // namespace

PointCloudDeltaReducer::PointCloudDeltaReducer(PointCloudReducerConfig config)
    : config_(config) {
    if (!std::isfinite(config_.voxel_size_m) || config_.voxel_size_m <= 0.0) {
        throw std::invalid_argument("voxel size must be finite and positive");
    }
    if (!std::isfinite(config_.movement_threshold_m) ||
        config_.movement_threshold_m < 0.0) {
        throw std::invalid_argument("movement threshold must be finite and non-negative");
    }
    if (config_.max_points == 0) {
        throw std::invalid_argument("maximum point count must be positive");
    }
}

PointCloudReductionResult PointCloudDeltaReducer::Reduce(
    const std::vector<MapPoint>& snapshot) {
    auto sorted = snapshot;
    std::sort(sorted.begin(), sorted.end(),
              [](const MapPoint& left, const MapPoint& right) { return left.id < right.id; });

    std::set<std::uint64_t> ids;
    std::set<Voxel> occupied_voxels;
    std::map<std::uint64_t, MapPoint> selected;
    std::size_t voxel_filtered = 0;
    std::size_t capacity_filtered = 0;

    for (const auto& point : sorted) {
        ValidatePoint(point);
        if (!ids.insert(point.id).second) {
            throw std::invalid_argument("map point snapshot contains a duplicate ID");
        }
        if (!occupied_voxels.insert(ToVoxel(point, config_.voxel_size_m)).second) {
            ++voxel_filtered;
            continue;
        }
        if (selected.size() == config_.max_points) {
            ++capacity_filtered;
            continue;
        }
        selected.emplace(point.id, point);
    }

    PointCloudDelta delta;
    std::map<std::uint64_t, MapPoint> next_sent;
    const double threshold_squared =
        config_.movement_threshold_m * config_.movement_threshold_m;

    for (const auto& [id, previous] : sent_points_) {
        if (selected.find(id) == selected.end()) {
            delta.remove.push_back(id);
        }
    }
    for (const auto& [id, current] : selected) {
        const auto previous = sent_points_.find(id);
        if (previous == sent_points_.end()) {
            delta.add.push_back(current);
            next_sent.emplace(id, current);
        } else if (SquaredDistance(previous->second, current) > threshold_squared) {
            delta.update.push_back(current);
            next_sent.emplace(id, current);
        } else {
            next_sent.emplace(id, previous->second);
        }
    }

    sent_points_ = std::move(next_sent);
    return {std::move(delta),
            {snapshot.size(), sent_points_.size(), voxel_filtered, capacity_filtered}};
}

void PointCloudDeltaReducer::Reset() noexcept { sent_points_.clear(); }

std::size_t PointCloudDeltaReducer::retained_point_count() const noexcept {
    return sent_points_.size();
}

}  // namespace slam_remote::telemetry
