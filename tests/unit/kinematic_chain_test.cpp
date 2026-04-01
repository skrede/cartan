#include <liepp/serial/chain/kinematic_chain.h>

#include <liepp/serial/chain/joint_state.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <numbers>

using Catch::Approx;

// ============================================================================
// kinematic_chain<double, 3> construction (3R planar robot, Lynch & Park Fig. 4.1)
// ============================================================================

TEST_CASE("kinematic_chain<double, 3> construction", "[kinematic_chain]")
{
    using namespace liepp;

    double L1 = 1.0, L2 = 1.0, L3 = 1.0;

    // Home configuration: end-effector at (L1+L2+L3, 0, 0)
    vector3<double> home_trans;
    home_trans << L1 + L2 + L3, 0, 0;
    auto home = se3<double>(so3<double>::identity(), home_trans);

    // Three z-axis revolute joints at origin, (L1,0,0), (L1+L2,0,0)
    auto s1 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {L1, 0, 0});
    auto s3 = screw_axis<double>::revolute({0, 0, 1}, {L1 + L2, 0, 0});

    joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

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
    using namespace liepp;

    double L = 1.0;
    vector3<double> home_trans;
    home_trans << 3 * L, 0, 0;
    auto home = se3<double>(so3<double>::identity(), home_trans);

    auto s1 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {L, 0, 0});
    auto s3 = screw_axis<double>::revolute({0, 0, 1}, {2 * L, 0, 0});

    joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

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
    using namespace liepp;

    double L = 1.0;
    vector3<double> home_trans;
    home_trans << 3 * L, 0, 0;
    auto home = se3<double>(so3<double>::identity(), home_trans);

    std::vector<screw_axis<double>> axes = {
        screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0}),
        screw_axis<double>::revolute({0, 0, 1}, {L, 0, 0}),
        screw_axis<double>::revolute({0, 0, 1}, {2 * L, 0, 0})
    };

    joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    std::vector<joint_limits<double>> limits = {lim, lim, lim};

    kinematic_chain<double, dynamic> chain(home, std::move(axes), std::move(limits));
    REQUIRE(chain.num_joints() == 3);
    REQUIRE((chain.home().translation() - home_trans).norm() < 1e-12);
}

// ============================================================================
// joint_limits contains
// ============================================================================

TEST_CASE("joint_limits contains", "[joint_limits]")
{
    liepp::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

    REQUIRE(lim.contains(0.0));
    REQUIRE(lim.contains(-std::numbers::pi));
    REQUIRE(lim.contains(std::numbers::pi));
    REQUIRE_FALSE(lim.contains(4.0));
    REQUIRE_FALSE(lim.contains(-4.0));
}

// ============================================================================
// joint_state from_position
// ============================================================================

TEST_CASE("joint_state from_position", "[joint_state]")
{
    Eigen::Vector3d q;
    q << 0.1, 0.2, 0.3;

    auto js = liepp::joint_state<double, 3>::from_position(q);
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
    using namespace liepp;

    auto home = se3<double>::identity();

    // Mix of revolute and prismatic
    auto s1 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = screw_axis<double>::prismatic({0, 0, 1});

    joint_limits<double> lim{-10, 10};
    std::array<screw_axis<double>, 2> axes = {s1, s2};
    std::array<joint_limits<double>, 2> limits = {lim, lim};

    kinematic_chain<double, 2> chain(home, axes, limits);

    REQUIRE(chain.axes()[0].is_revolute());
    REQUIRE(chain.axes()[1].is_prismatic());
}
