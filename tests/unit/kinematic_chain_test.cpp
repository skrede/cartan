#include <cartan/serial/chain/kinematic_chain.h>

#include <cartan/serial/chain/joint_state.h>

#include "../support/joint_limits_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <vector>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>

using Catch::Approx;

// ============================================================================
// kinematic_chain<double, 3> construction (3R planar robot, Lynch & Park Fig. 4.1)
// ============================================================================

TEST_CASE("kinematic_chain<double, 3> construction", "[kinematic_chain]")
{
    using namespace cartan;

    double L1 = 1.0, L2 = 1.0, L3 = 1.0;

    // Home configuration: end-effector at (L1+L2+L3, 0, 0)
    vector3<double> home_trans;
    home_trans << L1 + L2 + L3, 0, 0;
    auto home = se3<double>(so3<double>::identity(), home_trans);

    // Three z-axis revolute joints at origin, (L1,0,0), (L1+L2,0,0)
    auto s1 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {L1, 0, 0});
    auto s3 = screw_axis<double>::revolute({0, 0, 1}, {L1 + L2, 0, 0});

    auto lim = testing::limits(-std::numbers::pi, std::numbers::pi);

    std::array<screw_axis<double>, 3> axes = {s1, s2, s3};
    std::array<joint_limits<double>, 3> limits = {lim, lim, lim};

    kinematic_chain<double, 3> chain(home, axes, limits);

    REQUIRE(chain.num_joints() == 3);
    REQUIRE((chain.home().translation() - home_trans).norm() < 1e-12);
    REQUIRE(chain.axes()[0].omega()(2) == Approx(1.0).margin(1e-12));

    // axes[1].v() = -(0,0,1) x (1,0,0) = (0, -1, 0)
    REQUIRE(chain.axes()[1].v()(1) == Approx(-L1).margin(1e-12));
}

// ============================================================================
// kinematic_chain<double, 3> to_dynamic
// ============================================================================

TEST_CASE("kinematic_chain<double, 3> to_dynamic", "[kinematic_chain]")
{
    using namespace cartan;

    double L = 1.0;
    vector3<double> home_trans;
    home_trans << 3 * L, 0, 0;
    auto home = se3<double>(so3<double>::identity(), home_trans);

    auto s1 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {L, 0, 0});
    auto s3 = screw_axis<double>::revolute({0, 0, 1}, {2 * L, 0, 0});

    auto lim = testing::limits(-std::numbers::pi, std::numbers::pi);

    kinematic_chain<double, 3> fixed_chain(
        home,
        {s1, s2, s3},
        {lim, lim, lim});

    auto dyn_chain = fixed_chain.to_dynamic();
    REQUIRE(dyn_chain.num_joints() == 3);
    REQUIRE((dyn_chain.home().translation() - home_trans).norm() < 1e-12);

    for (std::size_t i = 0; i < 3; ++i)
    {
        REQUIRE((dyn_chain.axes()[i].omega() - fixed_chain.axes()[i].omega()).norm() < 1e-12);
        REQUIRE((dyn_chain.axes()[i].v() - fixed_chain.axes()[i].v()).norm() < 1e-12);
    }
}

// ============================================================================
// kinematic_chain<double, dynamic> construction
// ============================================================================

TEST_CASE("kinematic_chain<double, dynamic> construction", "[kinematic_chain]")
{
    using namespace cartan;

    double L = 1.0;
    vector3<double> home_trans;
    home_trans << 3 * L, 0, 0;
    auto home = se3<double>(so3<double>::identity(), home_trans);

    std::vector<screw_axis<double>> axes = {
        screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0}),
        screw_axis<double>::revolute({0, 0, 1}, {L, 0, 0}),
        screw_axis<double>::revolute({0, 0, 1}, {2 * L, 0, 0})
    };

    auto lim = testing::limits(-std::numbers::pi, std::numbers::pi);
    std::vector<joint_limits<double>> limits = {lim, lim, lim};

    kinematic_chain<double, dynamic> chain(home, std::move(axes), std::move(limits));
    REQUIRE(chain.num_joints() == 3);
    REQUIRE((chain.home().translation() - home_trans).norm() < 1e-12);
}

// ============================================================================
// joint_state from_position
// ============================================================================

TEST_CASE("joint_state from_position", "[joint_state]")
{
    Eigen::Vector3d q;
    q << 0.1, 0.2, 0.3;

    auto js = cartan::joint_state<double, 3>::from_position(q);
    REQUIRE(js.num_joints() == 3);
    REQUIRE(js.position(0) == Approx(0.1).margin(1e-14));
    REQUIRE(js.position(1) == Approx(0.2).margin(1e-14));
    REQUIRE(js.position(2) == Approx(0.3).margin(1e-14));
    REQUIRE_FALSE(js.velocity.has_value());
}

// ============================================================================
// kinematic_chain with prismatic joints
// ============================================================================

TEST_CASE("kinematic_chain with prismatic", "[kinematic_chain]")
{
    using namespace cartan;

    auto home = se3<double>::identity();

    // Mix of revolute and prismatic
    auto s1 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = screw_axis<double>::prismatic({0, 0, 1});

    auto lim = testing::limits(-10.0, 10.0);
    std::array<screw_axis<double>, 2> axes = {s1, s2};
    std::array<joint_limits<double>, 2> limits = {lim, lim};

    kinematic_chain<double, 2> chain(home, axes, limits);

    REQUIRE(chain.axes()[0].is_revolute());
    REQUIRE(chain.axes()[1].is_prismatic());
}

// ============================================================================
// Construction rejections, all effective in Release as well as Debug
// ============================================================================

namespace
{

template <typename Scalar>
cartan::screw_axis<Scalar> nonfinite_axis()
{
    cartan::vector3<Scalar> axis;
    axis << std::numeric_limits<Scalar>::quiet_NaN(), 0, 0;
    return cartan::screw_axis<Scalar>::revolute(axis, {0, 0, 0});
}

template <typename Scalar>
cartan::se3<Scalar> nonfinite_home()
{
    cartan::vector3<Scalar> t;
    t << std::numeric_limits<Scalar>::quiet_NaN(), 0, 0;
    return cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), t);
}

}

TEMPLATE_TEST_CASE("kinematic_chain rejects an axis-count / limit-count mismatch",
    "[kinematic_chain][boundary]", double, float)
{
    using S = TestType;
    std::vector<cartan::screw_axis<S>> axes = {
        cartan::screw_axis<S>::revolute({0, 0, 1}, {0, 0, 0}),
        cartan::screw_axis<S>::revolute({0, 0, 1}, {1, 0, 0})};
    std::vector<cartan::joint_limits<S>> one = {cartan::testing::limits(S(-1), S(1))};

    REQUIRE_THROWS_AS(
        (cartan::kinematic_chain<S, cartan::dynamic>(
            cartan::se3<S>::identity(), axes, one)),
        std::invalid_argument);
}

TEMPLATE_TEST_CASE("kinematic_chain rejects a nonfinite screw axis",
    "[kinematic_chain][boundary]", double, float)
{
    using S = TestType;
    auto lim = cartan::testing::limits(S(-1), S(1));
    std::vector<cartan::joint_limits<S>> limits = {lim, lim};
    std::vector<cartan::screw_axis<S>> poisoned = {
        nonfinite_axis<S>(), cartan::screw_axis<S>::revolute({0, 0, 1}, {1, 0, 0})};

    REQUIRE_THROWS_AS(
        (cartan::kinematic_chain<S, cartan::dynamic>(
            cartan::se3<S>::identity(), poisoned, limits)),
        std::invalid_argument);

    std::vector<cartan::screw_axis<S>> trailing = {
        cartan::screw_axis<S>::revolute({0, 0, 1}, {1, 0, 0}), nonfinite_axis<S>()};
    REQUIRE_THROWS_AS(
        (cartan::kinematic_chain<S, cartan::dynamic>(
            cartan::se3<S>::identity(), trailing, limits)),
        std::invalid_argument);
}

TEMPLATE_TEST_CASE("kinematic_chain rejects a nonfinite home pose",
    "[kinematic_chain][boundary]", double, float)
{
    using S = TestType;
    std::vector<cartan::screw_axis<S>> axes = {
        cartan::screw_axis<S>::revolute({0, 0, 1}, {0, 0, 0})};
    std::vector<cartan::joint_limits<S>> limits = {cartan::testing::limits(S(-1), S(1))};

    REQUIRE_THROWS_AS(
        (cartan::kinematic_chain<S, cartan::dynamic>(
            nonfinite_home<S>(), axes, limits)),
        std::invalid_argument);
}

/// A zero-magnitude axis is a separate defect from a nonfinite one and this
/// guard does not catch it: Eigen's normalized() returns a zero vector unchanged
/// rather than dividing by zero, so the stored axis is finite and the joint is
/// simply frozen. Asserted so the boundary between the two is not misread.
TEMPLATE_TEST_CASE("kinematic_chain still admits a zero-magnitude axis",
    "[kinematic_chain][boundary]", double, float)
{
    using S = TestType;
    std::vector<cartan::screw_axis<S>> axes = {
        cartan::screw_axis<S>::revolute(cartan::vector3<S>::Zero(), {0, 0, 0})};
    std::vector<cartan::joint_limits<S>> limits = {cartan::testing::limits(S(-1), S(1))};

    REQUIRE(cartan::screw_axis<S>::revolute(
        cartan::vector3<S>::Zero(), {0, 0, 0}).to_vector().allFinite());
    REQUIRE_NOTHROW(
        (cartan::kinematic_chain<S, cartan::dynamic>(
            cartan::se3<S>::identity(), axes, limits)));
}

// A chain with no joints is a permitted value of the runtime chain, not an
// accident of the validation happening to pass: its workspace is the single
// pose of its home configuration, and the solver semantics built on that
// require it to be constructible. static_chain refuses the same shape at
// compile time, so the two chain types disagree here deliberately on the
// runtime side and unintentionally overall.
TEMPLATE_TEST_CASE("kinematic_chain admits a chain with no joints",
    "[kinematic_chain][boundary]", double, float)
{
    using S = TestType;
    using chain_type = cartan::kinematic_chain<S, cartan::dynamic>;

    REQUIRE_NOTHROW((chain_type(cartan::se3<S>::identity(), {}, {})));

    chain_type chain(cartan::se3<S>::identity(), {}, {});
    REQUIRE(chain.num_joints() == 0);
}
