#include "liepp/serial_chain.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = liepp;
using Catch::Approx;

// ============================================================================
// Helper: 3R planar robot (Lynch & Park, Fig. 4.1 style)
//
// Three z-axis revolute joints at (0,0,0), (1,0,0), (2,0,0).
// Home configuration M places end-effector at (3,0,0) with identity rotation.
// ============================================================================

static spp::kinematic_chain<double, 3> make_3r_chain()
{
    double L = 1.0;
    spp::vector3<double> home_trans;
    home_trans << 3 * L, 0, 0;
    auto home = spp::se3<double>(spp::so3<double>::identity(), home_trans);

    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<double>::revolute({0, 0, 1}, {L, 0, 0});
    auto s3 = spp::screw_axis<double>::revolute({0, 0, 1}, {2 * L, 0, 0});

    spp::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

    return spp::kinematic_chain<double, 3>(
        home,
        {s1, s2, s3},
        {lim, lim, lim});
}

// ============================================================================
// FK at zero configuration returns home pose
// ============================================================================

TEST_CASE("FK at zero configuration returns home pose", "[forward_kinematics]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();

    auto fk = spp::forward_kinematics(chain, q);

    // At zero config, end_effector should equal home (3, 0, 0) with identity rotation
    REQUIRE(fk.end_effector.translation()(0) == Approx(3.0).margin(1e-10));
    REQUIRE(fk.end_effector.translation()(1) == Approx(0.0).margin(1e-10));
    REQUIRE(fk.end_effector.translation()(2) == Approx(0.0).margin(1e-10));

    // Rotation should be identity (log norm near zero)
    REQUIRE(fk.end_effector.log().template head<3>().norm() < 1e-10);
}

// ============================================================================
// FK intermediate count matches num_joints
// ============================================================================

TEST_CASE("FK intermediate count matches num_joints", "[forward_kinematics]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();

    auto fk = spp::forward_kinematics(chain, q);

    REQUIRE(fk.num_joints() == 3);
    REQUIRE(fk.intermediates.size() == 3);
}

// ============================================================================
// FK consistency: intermediates[n-1] * home == end_effector
// ============================================================================

TEST_CASE("FK consistency: intermediates[n-1] * home == end_effector", "[forward_kinematics]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q;
    q << 0.3, -0.5, 0.7;

    auto fk = spp::forward_kinematics(chain, q);

    // intermediates[2] * home should equal end_effector
    auto reconstructed = fk.intermediates[2] * chain.home();
    auto diff = (reconstructed.inverse() * fk.end_effector).log();
    REQUIRE(diff.norm() < 1e-12);
}

// ============================================================================
// FK single joint rotation
// ============================================================================

TEST_CASE("FK single joint rotation", "[forward_kinematics]")
{
    // Single revolute joint about z at origin, home = identity
    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    spp::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

    auto home = spp::se3<double>::identity();
    spp::kinematic_chain<double, 1> chain(home, {s1}, {lim});

    Eigen::Vector<double, 1> q;
    q << std::numbers::pi / 2.0;

    auto fk = spp::forward_kinematics(chain, q);

    // Rotation of pi/2 about z: rotation component should have pi/2 angle
    auto omega = fk.end_effector.rotation().log();
    REQUIRE(omega.norm() == Approx(std::numbers::pi / 2.0).margin(1e-10));
    REQUIRE(omega(2) == Approx(std::numbers::pi / 2.0).margin(1e-10));

    // Translation should remain zero (rotation at origin)
    REQUIRE(fk.end_effector.translation().norm() < 1e-10);
}

// ============================================================================
// FK 3-DOF at non-zero q
// ============================================================================

TEST_CASE("FK 3-DOF at non-zero q", "[forward_kinematics]")
{
    auto chain = make_3r_chain();

    // Rotate joint 1 by pi/2: the entire chain swings to align along y-axis
    Eigen::Vector3d q;
    q << std::numbers::pi / 2.0, 0, 0;

    auto fk = spp::forward_kinematics(chain, q);

    // After rotating joint 1 by pi/2 about z at origin:
    // The originally-at-(3,0,0) end-effector should be at approximately (0, 3, 0)
    REQUIRE(fk.end_effector.translation()(0) == Approx(0.0).margin(1e-10));
    REQUIRE(fk.end_effector.translation()(1) == Approx(3.0).margin(1e-10));
    REQUIRE(fk.end_effector.translation()(2) == Approx(0.0).margin(1e-10));

    // Rotation should be pi/2 about z
    auto omega = fk.end_effector.rotation().log();
    REQUIRE(omega(2) == Approx(std::numbers::pi / 2.0).margin(1e-10));
}

// ============================================================================
// FK dynamic chain matches fixed chain
// ============================================================================

TEST_CASE("FK dynamic chain matches fixed chain", "[forward_kinematics]")
{
    auto fixed_chain = make_3r_chain();
    auto dyn_chain = fixed_chain.to_dynamic();

    Eigen::Vector3d q_fixed;
    q_fixed << 0.3, -0.5, 0.7;

    Eigen::VectorXd q_dyn(3);
    q_dyn << 0.3, -0.5, 0.7;

    auto fk_fixed = spp::forward_kinematics(fixed_chain, q_fixed);
    auto fk_dyn = spp::forward_kinematics(dyn_chain, q_dyn);

    // End-effector poses should match within tight tolerance
    auto diff = (fk_fixed.end_effector.inverse() * fk_dyn.end_effector).log();
    REQUIRE(diff.norm() < 1e-12);

    // Intermediates should also match
    for (int i = 0; i < 3; ++i)
    {
        auto idiff = (fk_fixed.intermediates[static_cast<std::size_t>(i)].inverse()
                      * fk_dyn.intermediates[static_cast<std::size_t>(i)]).log();
        REQUIRE(idiff.norm() < 1e-12);
    }
}

// ============================================================================
// FK with float scalar
// ============================================================================

TEST_CASE("FK with float scalar", "[forward_kinematics]")
{
    float L = 1.0f;
    spp::vector3<float> home_trans;
    home_trans << 3 * L, 0, 0;
    auto home = spp::se3<float>(spp::so3<float>::identity(), home_trans);

    auto s1 = spp::screw_axis<float>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<float>::revolute({0, 0, 1}, {L, 0, 0});
    auto s3 = spp::screw_axis<float>::revolute({0, 0, 1}, {2 * L, 0, 0});

    spp::joint_limits<float> lim{
        -static_cast<float>(std::numbers::pi),
        static_cast<float>(std::numbers::pi)};

    spp::kinematic_chain<float, 3> chain(
        home, {s1, s2, s3}, {lim, lim, lim});

    Eigen::Vector3f q = Eigen::Vector3f::Zero();
    auto fk = spp::forward_kinematics(chain, q);

    // At zero config, end_effector should be at (3, 0, 0)
    REQUIRE(fk.end_effector.translation()(0) == Approx(3.0f).margin(1e-5f));
    REQUIRE(fk.end_effector.translation()(1) == Approx(0.0f).margin(1e-5f));
    REQUIRE(fk.end_effector.translation()(2) == Approx(0.0f).margin(1e-5f));
}
