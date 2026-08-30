#include "slam_remote/telemetry/point_cloud_delta_reducer.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

using slam_remote::telemetry::MapPoint;
using slam_remote::telemetry::PointCloudDeltaReducer;
using slam_remote::telemetry::PointCloudReducerConfig;

namespace {

void SelectsOneStableRepresentativePerVoxel() {
    PointCloudDeltaReducer reducer({1.0, 0.1, 100});
    const auto result = reducer.Reduce({{20, 0.2, 0.2, 0.2}, {10, 0.1, 0.1, 0.1},
                                        {30, 1.1, 0.1, 0.1}});
    assert(result.delta.add.size() == 2);
    assert(result.delta.add[0].id == 10);
    assert(result.delta.add[1].id == 30);
    assert(result.stats.voxel_filtered_points == 1);
}

void EmitsOnlyMeaningfulChanges() {
    PointCloudDeltaReducer reducer({0.01, 0.1, 100});
    reducer.Reduce({{1, 0.0, 0.0, 0.0}, {2, 1.0, 0.0, 0.0}});

    const auto below_threshold =
        reducer.Reduce({{1, 0.05, 0.0, 0.0}, {2, 1.0, 0.0, 0.0}});
    assert(below_threshold.delta.empty());

    const auto changed = reducer.Reduce({{1, 0.11, 0.0, 0.0}, {3, 2.0, 0.0, 0.0}});
    assert(changed.delta.remove == std::vector<std::uint64_t>{2});
    assert(changed.delta.update.size() == 1 && changed.delta.update[0].id == 1);
    assert(changed.delta.add.size() == 1 && changed.delta.add[0].id == 3);
}

void CoalescesMovementAgainstLastSentPosition() {
    PointCloudDeltaReducer reducer({0.001, 0.1, 100});
    reducer.Reduce({{1, 0.0, 0.0, 0.0}});
    assert(reducer.Reduce({{1, 0.06, 0.0, 0.0}}).delta.empty());
    const auto accumulated = reducer.Reduce({{1, 0.11, 0.0, 0.0}});
    assert(accumulated.delta.update.size() == 1);
}

void AppliesDeterministicCapacityLimit() {
    PointCloudDeltaReducer reducer({0.01, 0.0, 2});
    const auto result =
        reducer.Reduce({{30, 3.0, 0.0, 0.0}, {10, 1.0, 0.0, 0.0}, {20, 2.0, 0.0, 0.0}});
    assert(result.delta.add.size() == 2);
    assert(result.delta.add[0].id == 10 && result.delta.add[1].id == 20);
    assert(result.stats.capacity_filtered_points == 1);
}

void ResetForcesACompleteReAdd() {
    PointCloudDeltaReducer reducer;
    reducer.Reduce({{1, 0.0, 0.0, 0.0}});
    reducer.Reset();
    const auto result = reducer.Reduce({{1, 0.0, 0.0, 0.0}});
    assert(result.delta.add.size() == 1);
}

void RejectsInvalidSnapshotsAndConfiguration() {
    bool rejected = false;
    try {
        PointCloudDeltaReducer invalid({0.0, 0.0, 1});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);

    PointCloudDeltaReducer reducer;
    rejected = false;
    try {
        reducer.Reduce({{1, NAN, 0.0, 0.0}});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);

    rejected = false;
    try {
        reducer.Reduce({{1, 0.0, 0.0, 0.0}, {1, 1.0, 0.0, 0.0}});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}

}  // namespace

int main() {
    SelectsOneStableRepresentativePerVoxel();
    EmitsOnlyMeaningfulChanges();
    CoalescesMovementAgainstLastSentPosition();
    AppliesDeterministicCapacityLimit();
    ResetForcesACompleteReAdd();
    RejectsInvalidSnapshotsAndConfiguration();
    return 0;
}
