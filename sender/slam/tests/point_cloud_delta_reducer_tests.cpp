#include "slam_remote/slam/point_cloud_delta_reducer.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using slam_remote::slam::MapPoint;
using slam_remote::slam::PointCloudDeltaReducer;
using slam_remote::slam::PointCloudDeltaReducerConfig;

void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void TestDeterministicDeltasAndReset() {
    PointCloudDeltaReducer reducer;
    const auto first = reducer.Reduce({{2, {2.0, 0.0, 0.0}}, {1, {1.0, 0.0, 0.0}}});
    Check(first.add.size() == 2 && first.add[0].id == 1 && first.add[1].id == 2,
          "first snapshot must add points in ID order");
    Check(reducer.Reduce({{1, {1.0, 0.0, 0.0}}, {2, {2.0, 0.0, 0.0}}})
              .operation_count() == 0,
          "identical snapshot must be empty");

    const auto changed = reducer.Reduce({{3, {3.0, 0.0, 0.0}}, {2, {2.5, 0.0, 0.0}}});
    Check(changed.add.size() == 1 && changed.add[0].id == 3, "new point must be added");
    Check(changed.update.size() == 1 && changed.update[0].id == 2,
          "changed point must be updated");
    Check(changed.remove == std::vector<std::uint64_t>{1}, "missing point must be removed");
    const auto cleared = reducer.Reduce({});
    Check(cleared.remove == std::vector<std::uint64_t>({2, 3}),
          "empty snapshot must remove every retained point in ID order");

    reducer.Reset();
    Check(reducer.retained_points() == 0, "reset must clear baseline");
    Check(reducer.Reduce({{2, {2.5, 0.0, 0.0}}}).add.size() == 1,
          "snapshot after reset must start a new baseline");
}

void TestEpsilon() {
    PointCloudDeltaReducer reducer({10, 20, 0.1});
    reducer.Reduce({{1, {0.0, 0.0, 0.0}}});
    Check(reducer.Reduce({{1, {0.1, 0.0, 0.0}}}).update.empty(),
          "epsilon boundary must not update");
    Check(reducer.Reduce({{1, {0.1001, 0.0, 0.0}}}).update.size() == 1,
          "change above epsilon must update");
}

template <typename Action>
void ExpectFailureWithoutMutation(PointCloudDeltaReducer& reducer, Action action) {
    const auto retained = reducer.retained_points();
    bool failed = false;
    try {
        action();
    } catch (const std::exception&) {
        failed = true;
    }
    Check(failed, "invalid reduction must fail");
    Check(reducer.retained_points() == retained, "failure must not mutate baseline");
}

void TestInvalidAndBoundedInputIsTransactional() {
    PointCloudDeltaReducer reducer({2, 3, 0.0});
    reducer.Reduce({{1, {1.0, 0.0, 0.0}}});
    ExpectFailureWithoutMutation(reducer, [&] {
        reducer.Reduce({{2, {0.0, 0.0, 0.0}}, {2, {1.0, 0.0, 0.0}}});
    });
    ExpectFailureWithoutMutation(reducer, [&] {
        reducer.Reduce({{2, {std::numeric_limits<double>::infinity(), 0.0, 0.0}}});
    });
    ExpectFailureWithoutMutation(reducer, [&] {
        reducer.Reduce({{slam_remote::slam::kMaxSafePointId + 1, {0.0, 0.0, 0.0}}});
    });
    ExpectFailureWithoutMutation(reducer, [&] {
        reducer.Reduce({{1, {0.0, 0.0, 0.0}}, {2, {0.0, 0.0, 0.0}},
                        {3, {0.0, 0.0, 0.0}}});
    });

    PointCloudDeltaReducer output_limited({2, 1, 0.0});
    output_limited.Reduce({{1, {0.0, 0.0, 0.0}}});
    ExpectFailureWithoutMutation(output_limited, [&] {
        output_limited.Reduce({{2, {0.0, 0.0, 0.0}}});
    });
    Check(output_limited.Reduce({{1, {0.0, 0.0, 0.0}}}).operation_count() == 0,
          "failed output must retain the previous point, not only its count");
}

}  // namespace

int main() {
    try {
        TestDeterministicDeltasAndReset();
        TestEpsilon();
        TestInvalidAndBoundedInputIsTransactional();
    } catch (const std::exception& error) {
        std::cerr << "point-cloud delta reducer tests failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "point-cloud delta reducer tests passed\n";
    return 0;
}
