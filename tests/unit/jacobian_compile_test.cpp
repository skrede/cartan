#include "liepp/serial_chain.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = liepp;
using Catch::Approx;

// ============================================================================
// Helper: 3R planar robot (same as forward_kinematics_test)
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
// jacobian_matrix type alias resolves correctly
// ============================================================================

TEST_CASE("jacobian_matrix fixed is 6xN", "[jacobian]")
{
    using jm = spp::jacobian_matrix<double, 3>;
    static_assert(jm::RowsAtCompileTime == 6);
    static_assert(jm::ColsAtCompileTime == 3);
    SUCCEED();
}

TEST_CASE("jacobian_matrix dynamic is 6xDynamic", "[jacobian]")
{
    using jm = spp::jacobian_matrix<double, spp::dynamic>;
    static_assert(jm::RowsAtCompileTime == 6);
    static_assert(jm::ColsAtCompileTime == Eigen::Dynamic);
    SUCCEED();
}

// ============================================================================
// space_jacobian basic API
// ============================================================================

TEST_CASE("space_jacobian returns 6x3 for 3R chain", "[jacobian]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    auto fk = spp::forward_kinematics(chain, q);

    auto J_s = spp::space_jacobian(chain, fk);

    REQUIRE(J_s.rows() == 6);
    REQUIRE(J_s.cols() == 3);
}

// ============================================================================
// body_jacobian basic API
// ============================================================================

TEST_CASE("body_jacobian returns 6x3 for 3R chain", "[jacobian]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    auto fk = spp::forward_kinematics(chain, q);

    auto J_b = spp::body_jacobian(chain, fk);

    REQUIRE(J_b.rows() == 6);
    REQUIRE(J_b.cols() == 3);
}

// ============================================================================
// end_effector_velocity basic API
// ============================================================================

TEST_CASE("end_effector_velocity returns vector6", "[velocity]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    Eigen::Vector3d dq = Eigen::Vector3d::Zero();

    auto vel = spp::end_effector_velocity(chain, q, dq);

    REQUIRE(vel.size() == 6);
    REQUIRE(vel.norm() < 1e-15);
}
