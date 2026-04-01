#include <liepp/serial/chain/static_chain.h>
#include <liepp/serial/chain/chain_concept.h>
#include <liepp/serial/chain/joint_tags.h>
#include <liepp/serial/chain/screw_axis.h>
#include <liepp/serial/chain/joint_limits.h>
#include <liepp/serial/chain/kinematic_chain.h>
#include <liepp/serial/chain/joint_state.h>
#include <liepp/serial/fk/forward_kinematics.h>
#include <liepp/serial/fk/jacobian.h>

#include <liepp/lie/se3.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <numbers>

using Catch::Approx;

// ============================================================================
// Compile-time joint count
// ============================================================================

TEST_CASE("static_chain joints count is compile-time", "[static_chain]")
{
    static_assert(liepp::static_chain<double, liepp::revolute_z, liepp::revolute_y, liepp::revolute_z>::joints == 3);
    SUCCEED();
}

// ============================================================================
// Single-joint construction
// ============================================================================

TEST_CASE("static_chain single revolute joint", "[static_chain]")
{
    using namespace liepp;

    auto home = se3<double>::identity();
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

    static_chain<double, revolute_z> sc(home, {s0}, {lim});

    REQUIRE(sc.num_joints() == 1);
    REQUIRE(sc.joints == 1);
}

// ============================================================================
// Three-joint accessor correctness
// ============================================================================

TEST_CASE("static_chain 3R axis accessors", "[static_chain]")
{
    using namespace liepp;

    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.5});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 1.0});
    auto home = se3<double>(so3<double>::identity(), vector3<double>{1.0, 0, 1.0});
    joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

    static_chain<double, revolute_z, revolute_y, revolute_z> sc(
        home, {s0, s1, s2}, {lim, lim, lim});

    REQUIRE((sc.axis(0).to_vector() - s0.to_vector()).norm() < 1e-14);
    REQUIRE((sc.axis(1).to_vector() - s1.to_vector()).norm() < 1e-14);
    REQUIRE((sc.axis(2).to_vector() - s2.to_vector()).norm() < 1e-14);
}

// ============================================================================
// axes() returns iterable range of correct length
// ============================================================================

TEST_CASE("static_chain axes() range", "[static_chain]")
{
    using namespace liepp;

    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.5});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 1.0});
    auto home = se3<double>::identity();
    joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

    static_chain<double, revolute_z, revolute_y, revolute_z> sc(
        home, {s0, s1, s2}, {lim, lim, lim});

    const auto& axes = sc.axes();
    REQUIRE(axes.size() == 3);

    REQUIRE((axes[0].to_vector() - s0.to_vector()).norm() < 1e-14);
    REQUIRE((axes[1].to_vector() - s1.to_vector()).norm() < 1e-14);
    REQUIRE((axes[2].to_vector() - s2.to_vector()).norm() < 1e-14);
}

// ============================================================================
// home() returns the provided SE3 pose
// ============================================================================

TEST_CASE("static_chain home() returns construction pose", "[static_chain]")
{
    using namespace liepp;

    vector3<double> t{1.0, 2.0, 3.0};
    auto home = se3<double>(so3<double>::identity(), t);
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

    static_chain<double, revolute_z> sc(home, {s0}, {lim});

    REQUIRE((sc.home().translation() - t).norm() < 1e-14);
}

// ============================================================================
// limits() returns the provided joint limits
// ============================================================================

TEST_CASE("static_chain limits() returns construction limits", "[static_chain]")
{
    using namespace liepp;

    auto home = se3<double>::identity();
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.5});
    joint_limits<double> lim0{-1.0, 1.0};
    joint_limits<double> lim1{-2.0, 2.0};

    static_chain<double, revolute_z, revolute_y> sc(home, {s0, s1}, {lim0, lim1});

    const auto& limits = sc.limits();
    REQUIRE(limits.size() == 2);
    REQUIRE(limits[0].position_min == Approx(-1.0));
    REQUIRE(limits[0].position_max == Approx(1.0));
    REQUIRE(limits[1].position_min == Approx(-2.0));
    REQUIRE(limits[1].position_max == Approx(2.0));
}

// ============================================================================
// Concept satisfaction
// ============================================================================

TEST_CASE("static_chain satisfies chain concept", "[static_chain]")
{
    static_assert(liepp::chain<liepp::static_chain<double, liepp::revolute_z, liepp::revolute_y, liepp::revolute_z>>);
    SUCCEED();
}

// ============================================================================
// Prismatic joint tag
// ============================================================================

TEST_CASE("static_chain with prismatic joint tag", "[static_chain]")
{
    using namespace liepp;

    auto home = se3<double>::identity();
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::prismatic({0, 0, 1});
    joint_limits<double> lim{-10.0, 10.0};

    static_chain<double, revolute_z, prismatic_z> sc(home, {s0, s1}, {lim, lim});

    REQUIRE(sc.num_joints() == 2);
    REQUIRE(sc.axis(0).is_revolute());
    REQUIRE(sc.axis(1).is_prismatic());
}

// ============================================================================
// Helper: build equivalent static_chain and kinematic_chain for 3R robot
// ============================================================================

static auto make_3r_screw_axes()
{
    using namespace liepp;
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.5});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 1.0});
    return std::array<screw_axis<double>, 3>{s0, s1, s2};
}

static auto make_3r_home()
{
    using namespace liepp;
    return se3<double>(so3<double>::identity(), vector3<double>{1.0, 0.0, 1.0});
}

static auto make_3r_limits()
{
    using namespace liepp;
    joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    return std::array<joint_limits<double>, 3>{lim, lim, lim};
}

// ============================================================================
// FK equivalence: static_chain vs kinematic_chain
// ============================================================================

TEST_CASE("FK on static_chain matches kinematic_chain", "[static_chain][fk]")
{
    using namespace liepp;

    auto axes = make_3r_screw_axes();
    auto home = make_3r_home();
    auto limits = make_3r_limits();

    static_chain<double, revolute_z, revolute_y, revolute_z> sc(home, axes, limits);
    kinematic_chain<double, 3> kc(home, axes, limits);

    // Test at multiple configurations
    std::array<Eigen::Vector3d, 5> configs;
    configs[0] = Eigen::Vector3d::Zero();
    configs[1] = Eigen::Vector3d{0.5, -0.3, 0.8};
    configs[2] = Eigen::Vector3d{-1.0, 1.2, -0.7};
    configs[3] = Eigen::Vector3d{std::numbers::pi / 4, std::numbers::pi / 3, -std::numbers::pi / 6};
    configs[4] = Eigen::Vector3d{2.1, -1.5, 0.3};

    for (const auto& q : configs)
    {
        auto fk_sc = forward_kinematics(sc, q);
        auto fk_kc = forward_kinematics(kc, q);

        REQUIRE((fk_sc.end_effector.matrix() - fk_kc.end_effector.matrix()).norm() < 1e-12);

        for (std::size_t i = 0; i < 3; ++i)
        {
            REQUIRE((fk_sc.intermediates[i].matrix() - fk_kc.intermediates[i].matrix()).norm() < 1e-12);
        }
    }
}

// ============================================================================
// Space Jacobian equivalence: static_chain vs kinematic_chain
// ============================================================================

TEST_CASE("Space Jacobian on static_chain matches kinematic_chain", "[static_chain][jacobian]")
{
    using namespace liepp;

    auto axes = make_3r_screw_axes();
    auto home = make_3r_home();
    auto limits = make_3r_limits();

    static_chain<double, revolute_z, revolute_y, revolute_z> sc(home, axes, limits);
    kinematic_chain<double, 3> kc(home, axes, limits);

    std::array<Eigen::Vector3d, 5> configs;
    configs[0] = Eigen::Vector3d::Zero();
    configs[1] = Eigen::Vector3d{0.5, -0.3, 0.8};
    configs[2] = Eigen::Vector3d{-1.0, 1.2, -0.7};
    configs[3] = Eigen::Vector3d{std::numbers::pi / 4, std::numbers::pi / 3, -std::numbers::pi / 6};
    configs[4] = Eigen::Vector3d{2.1, -1.5, 0.3};

    for (const auto& q : configs)
    {
        auto fk_sc = forward_kinematics(sc, q);
        auto fk_kc = forward_kinematics(kc, q);

        auto J_sc = space_jacobian(sc, fk_sc);
        auto J_kc = space_jacobian(kc, fk_kc);

        REQUIRE((J_sc - J_kc).norm() < 1e-12);
    }
}

// ============================================================================
// Body Jacobian equivalence: static_chain vs kinematic_chain
// ============================================================================

TEST_CASE("Body Jacobian on static_chain matches kinematic_chain", "[static_chain][jacobian]")
{
    using namespace liepp;

    auto axes = make_3r_screw_axes();
    auto home = make_3r_home();
    auto limits = make_3r_limits();

    static_chain<double, revolute_z, revolute_y, revolute_z> sc(home, axes, limits);
    kinematic_chain<double, 3> kc(home, axes, limits);

    std::array<Eigen::Vector3d, 5> configs;
    configs[0] = Eigen::Vector3d::Zero();
    configs[1] = Eigen::Vector3d{0.5, -0.3, 0.8};
    configs[2] = Eigen::Vector3d{-1.0, 1.2, -0.7};
    configs[3] = Eigen::Vector3d{std::numbers::pi / 4, std::numbers::pi / 3, -std::numbers::pi / 6};
    configs[4] = Eigen::Vector3d{2.1, -1.5, 0.3};

    for (const auto& q : configs)
    {
        auto fk_sc = forward_kinematics(sc, q);
        auto fk_kc = forward_kinematics(kc, q);

        auto Jb_sc = body_jacobian(sc, fk_sc);
        auto Jb_kc = body_jacobian(kc, fk_kc);

        REQUIRE((Jb_sc - Jb_kc).norm() < 1e-12);
    }
}
