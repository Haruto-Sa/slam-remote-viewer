#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace slam_remote::telemetry {

struct MapPoint final {
    std::uint64_t id;
    double x;
    double y;
    double z;
};

struct PointCloudDelta final {
    std::vector<MapPoint> add;
    std::vector<MapPoint> update;
    std::vector<std::uint64_t> remove;

    [[nodiscard]] bool empty() const noexcept {
        return add.empty() && update.empty() && remove.empty();
    }
};

struct PointCloudReductionStats final {
    std::size_t input_points;
    std::size_t selected_points;
    std::size_t voxel_filtered_points;
    std::size_t capacity_filtered_points;
};

struct PointCloudReductionResult final {
    PointCloudDelta delta;
    PointCloudReductionStats stats;
};

struct PointCloudReducerConfig final {
    double voxel_size_m{0.03};
    double movement_threshold_m{0.005};
    std::size_t max_points{50'000};
};

/// Converts complete backend map snapshots into small, deterministic Protocol v1
/// deltas. The lowest stable map-point ID represents each occupied voxel.
class PointCloudDeltaReducer final {
   public:
    explicit PointCloudDeltaReducer(PointCloudReducerConfig config = {});

    PointCloudReductionResult Reduce(const std::vector<MapPoint>& snapshot);
    void Reset() noexcept;
    [[nodiscard]] std::size_t retained_point_count() const noexcept;

   private:
    PointCloudReducerConfig config_;
    std::map<std::uint64_t, MapPoint> sent_points_;
};

}  // namespace slam_remote::telemetry
