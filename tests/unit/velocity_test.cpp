#include "liepp/kinematics.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = liepp;
using Catch::Approx;

// ============================================================================
// Helper: 3R planar robot
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
// Velocity at dq=0 is zero
// ============================================================================

TEST_CASE("Velocity at dq=0 is zero", "[velocity]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q;
    q << 0.3, -0.5, 0.7;
    Eigen::Vector3d dq = Eigen::Vector3d::Zero();

    auto vel = spp::end_effector_velocity(chain, q, dq);

    REQUIRE(vel.norm() < 1e-15);
}

// ============================================================================
// Single-joint velocity equals screw axis
// ============================================================================

TEST_CASE("Single-joint velocity equals screw axis", "[velocity]")
{
    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    spp::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    auto home = spp::se3<double>::identity();

    spp::kinematic_chain<double, 1> chain(home, {s1}, {lim});

    Eigen::Vector<double, 1> q;
    q << 0.0;
    Eigen::Vector<double, 1> dq;
    dq << 1.0;

    auto vel = spp::end_effector_velocity(chain, q, dq);

    auto s1_vec = s1.to_vector();
    for (int i = 0; i < 6; ++i)
    {
        REQUIRE(vel(i) == Approx(s1_vec(i)).margin(1e-12));
    }
}

// ============================================================================
// Velocity matches finite-difference FK
//
// V_s approx= log(FK(q + dt*dq) * FK(q)^{-1}) / dt
// ============================================================================

TEST_CASE("Velocity matches finite-difference FK", "[velocity]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q;
    q << std::numbers::pi / 4.0, -std::numbers::pi / 6.0, std::numbers::pi / 3.0;
    Eigen::Vector3d dq;
    dq << 1.0, -0.5, 0.3;

    auto vel = spp::end_effector_velocity(chain, q, dq);

    // Finite-difference: log(FK(q + dt*dq) * FK(q)^{-1}) / dt
    double dt = 1e-8;
    auto fk_base = spp::forward_kinematics(chain, q);
    Eigen::Vector3d q_perturbed = q + dt * dq;
    auto fk_pert = spp::forward_kinematics(chain, q_perturbed);

    auto delta = (fk_pert.end_effector * fk_base.end_effector.inverse()).log();
    spp::vector6<double> vel_fd = delta / dt;

    for (int i = 0; i < 6; ++i)
    {
        REQUIRE(vel(i) == Approx(vel_fd(i)).margin(1e-5));
    }
}
