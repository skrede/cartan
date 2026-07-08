#include "cartan/analytical/selection.h"

#include <catch2/catch_test_macros.hpp>

using cartan::closest_to_seed;
using cartan::analytical_result;
using cartan::unwrapped_result;
using cartan::range_status;
using cartan::analytical_failure;

TEST_CASE("closest_to_seed over analytical_result picks the L2-nearest branch")
{
    analytical_result<double, 2, 4> r;
    r.solutions[0] = Eigen::Vector2d(1.0, 0.0);
    r.solutions[1] = Eigen::Vector2d(0.1, 0.0);
    r.solutions[2] = Eigen::Vector2d(-2.0, 0.0);
    r.count = 3;

    auto pick = closest_to_seed(r, Eigen::Vector2d(0.0, 0.0));
    REQUIRE(pick.has_value());
    REQUIRE((*pick - Eigen::Vector2d(0.1, 0.0)).norm() < 1e-12);
}

TEST_CASE("closest_to_seed over an empty analytical_result is unreachable")
{
    analytical_result<double, 2, 4> r;
    r.count = 0;

    auto pick = closest_to_seed(r, Eigen::Vector2d(0.0, 0.0));
    REQUIRE_FALSE(pick.has_value());
    REQUIRE(pick.error().reason == analytical_failure::unreachable);
}

TEST_CASE("closest_to_seed over unwrapped_result skips a nearer out-of-range branch")
{
    unwrapped_result<double, 2, 4> r;
    r.solutions[0] = Eigen::Vector2d(0.1, 0.0);
    r.tags[0] = range_status::joint_limits_violated;
    r.solutions[1] = Eigen::Vector2d(0.5, 0.0);
    r.tags[1] = range_status::in_range;
    r.solutions[2] = Eigen::Vector2d(2.0, 0.0);
    r.tags[2] = range_status::in_range;
    r.count = 3;

    auto pick = closest_to_seed(r, Eigen::Vector2d(0.0, 0.0));
    REQUIRE(pick.has_value());
    REQUIRE(pick->status == range_status::in_range);
    REQUIRE((pick->q - Eigen::Vector2d(0.5, 0.0)).norm() < 1e-12);
}

TEST_CASE("closest_to_seed over unwrapped_result returns the nearest when it is in range")
{
    unwrapped_result<double, 2, 4> r;
    r.solutions[0] = Eigen::Vector2d(0.2, 0.0);
    r.tags[0] = range_status::in_range;
    r.solutions[1] = Eigen::Vector2d(0.9, 0.0);
    r.tags[1] = range_status::joint_limits_violated;
    r.count = 2;

    auto pick = closest_to_seed(r, Eigen::Vector2d(0.0, 0.0));
    REQUIRE(pick.has_value());
    REQUIRE(pick->status == range_status::in_range);
    REQUIRE((pick->q - Eigen::Vector2d(0.2, 0.0)).norm() < 1e-12);
}

TEST_CASE("closest_to_seed over an all-violated unwrapped_result returns the nearest overall")
{
    unwrapped_result<double, 2, 4> r;
    r.solutions[0] = Eigen::Vector2d(2.0, 0.0);
    r.tags[0] = range_status::joint_limits_violated;
    r.solutions[1] = Eigen::Vector2d(0.3, 0.0);
    r.tags[1] = range_status::joint_limits_violated;
    r.count = 2;

    auto pick = closest_to_seed(r, Eigen::Vector2d(0.0, 0.0));
    REQUIRE(pick.has_value());
    REQUIRE(pick->status == range_status::joint_limits_violated);
    REQUIRE((pick->q - Eigen::Vector2d(0.3, 0.0)).norm() < 1e-12);
}

TEST_CASE("closest_to_seed over an empty unwrapped_result is unreachable")
{
    unwrapped_result<double, 2, 4> r;
    r.count = 0;

    auto pick = closest_to_seed(r, Eigen::Vector2d(0.0, 0.0));
    REQUIRE_FALSE(pick.has_value());
    REQUIRE(pick.error().reason == analytical_failure::unreachable);
}
