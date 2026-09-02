#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace slam_remote::slam {

inline constexpr std::uint64_t kMaxSafePointId = 9'007'199'254'740'991ULL;

struct MapPoint final {
    std::uint64_t id;
    std::array<double, 3> position_metres;
};

struct PointCloudDelta final {
    std::vector<MapPoint> add;
    std::vector<MapPoint> update;
    std::vector<std::uint64_t> remove;

    [[nodiscard]] std::size_t operation_count() const noexcept {
        return add.size() + update.size() + remove.size();
    }
};

struct PointCloudDeltaReducerConfig final {
    std::size_t max_snapshot_points{100'000};
    std::size_t max_delta_operations{200'000};
    double update_epsilon_metres{1e-6};
};

/// Retains one validated baseline and emits deterministic snapshot differences.
/// Failed reductions never modify the retained baseline.
class PointCloudDeltaReducer final {
   public:
    explicit PointCloudDeltaReducer(PointCloudDeltaReducerConfig config = {});

    PointCloudDelta Reduce(const std::vector<MapPoint>& snapshot);
    void Reset() noexcept;
    [[nodiscard]] std::size_t retained_points() const noexcept;

   private:
    PointCloudDeltaReducerConfig config_;
    std::map<std::uint64_t, std::array<double, 3>> baseline_;
};

}  // namespace slam_remote::slam
