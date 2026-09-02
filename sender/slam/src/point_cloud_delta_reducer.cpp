#include "slam_remote/slam/point_cloud_delta_reducer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace slam_remote::slam {
namespace {

bool Changed(const std::array<double, 3>& previous, const std::array<double, 3>& current,
             double epsilon) {
    for (std::size_t axis = 0; axis < previous.size(); ++axis) {
        if (std::abs(previous[axis] - current[axis]) > epsilon) return true;
    }
    return false;
}

}  // namespace

PointCloudDeltaReducer::PointCloudDeltaReducer(PointCloudDeltaReducerConfig config)
    : config_(config) {
    if (config_.max_snapshot_points == 0 || config_.max_delta_operations == 0) {
        throw std::invalid_argument("point-cloud reducer bounds must be positive");
    }
    if (!std::isfinite(config_.update_epsilon_metres) ||
        config_.update_epsilon_metres < 0.0) {
        throw std::invalid_argument("point-cloud update epsilon must be finite and non-negative");
    }
}

PointCloudDelta PointCloudDeltaReducer::Reduce(const std::vector<MapPoint>& snapshot) {
    if (snapshot.size() > config_.max_snapshot_points) {
        throw std::length_error("point-cloud snapshot exceeds configured point limit");
    }

    std::map<std::uint64_t, std::array<double, 3>> next;
    for (const auto& point : snapshot) {
        if (point.id > kMaxSafePointId) {
            throw std::invalid_argument("point-cloud ID exceeds JSON safe integer range");
        }
        if (!std::all_of(point.position_metres.begin(), point.position_metres.end(),
                         [](double value) { return std::isfinite(value); })) {
            throw std::invalid_argument("point-cloud position must contain only finite values");
        }
        if (!next.emplace(point.id, point.position_metres).second) {
            throw std::invalid_argument("point-cloud snapshot contains duplicate ID " +
                                        std::to_string(point.id));
        }
    }

    PointCloudDelta delta;
    for (const auto& [id, position] : next) {
        const auto previous = baseline_.find(id);
        if (previous == baseline_.end()) {
            delta.add.push_back({id, position});
        } else if (Changed(previous->second, position, config_.update_epsilon_metres)) {
            delta.update.push_back({id, position});
        }
    }
    for (const auto& [id, position] : baseline_) {
        static_cast<void>(position);
        if (next.find(id) == next.end()) delta.remove.push_back(id);
    }
    if (delta.operation_count() > config_.max_delta_operations) {
        throw std::length_error("point-cloud delta exceeds configured operation limit");
    }
    auto committed = baseline_;
    for (const auto& point : delta.add) committed[point.id] = point.position_metres;
    for (const auto& point : delta.update) committed[point.id] = point.position_metres;
    for (const auto id : delta.remove) committed.erase(id);
    baseline_ = std::move(committed);
    return delta;
}

void PointCloudDeltaReducer::Reset() noexcept { baseline_.clear(); }

std::size_t PointCloudDeltaReducer::retained_points() const noexcept { return baseline_.size(); }

}  // namespace slam_remote::slam
