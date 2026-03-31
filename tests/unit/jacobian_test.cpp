#include "liepp/kinematics.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = liepp;
using Catch::Approx;

// ============================================================================
// Helper: 3R planar robot (Lynch & Park, Fig. 4.1 style)
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
// Finite-difference space Jacobian helper
//
// Column i = log(FK(q + h*e_i) * FK(q - h*e_i)^{-1}) / (2h)
// Central difference for better accuracy.
// ============================================================================

template <int N, typename Scalar>
spp::jacobian_matrix<Scalar, N> finite_difference_space_jacobian(
    const spp::kinematic_chain<Scalar, N>& chain,
    const typename spp::joint_state<Scalar, N>::position_type& q,
    Scalar h = Scalar(1e-8))
{
    int n = chain.num_joints();
    spp::jacobian_matrix<Scalar, N> J;
    if constexpr (N == spp::dynamic)
    {
        J.resize(6, n);
    }

    for (int i = 0; i < n; ++i)
    {
        auto q_plus = q;
        auto q_minus = q;
        q_plus(i) += h;
        q_minus(i) -= h;

        auto fk_plus = spp::forward_kinematics(chain, q_plus);
        auto fk_minus = spp::forward_kinematics(chain, q_minus);

        // Space-frame finite difference: log(T_plus * T_minus^{-1}) / (2h)
        auto delta = (fk_plus.end_effector * fk_minus.end_effector.inverse()).log();
        J.col(i) = delta / (Scalar(2) * h);
    }

    return J;
}

// ============================================================================
// Finite-difference body Jacobian helper
//
// Column i = log(FK(q - h*e_i)^{-1} * FK(q + h*e_i)) / (2h)
// Body-frame: T^{-1} * T_perturbed ordering.
// ============================================================================

template <int N, typename Scalar>
spp::jacobian_matrix<Scalar, N> finite_difference_body_jacobian(
    const spp::kinematic_chain<Scalar, N>& chain,
    const typename spp::joint_state<Scalar, N>::position_type& q,
    Scalar h = Scalar(1e-8))
{
    int n = chain.num_joints();
    spp::jacobian_matrix<Scalar, N> J;
    if constexpr (N == spp::dynamic)
    {
        J.resize(6, n);
    }

    for (int i = 0; i < n; ++i)
    {
        auto q_plus = q;
        auto q_minus = q;
        q_plus(i) += h;
        q_minus(i) -= h;

        auto fk_plus = spp::forward_kinematics(chain, q_plus);
        auto fk_minus = spp::forward_kinematics(chain, q_minus);

        // Body-frame finite difference: log(T_minus^{-1} * T_plus) / (2h)
        auto delta = (fk_minus.end_effector.inverse() * fk_plus.end_effector).log();
        J.col(i) = delta / (Scalar(2) * h);
    }

    return J;
}

// ============================================================================
// Space Jacobian at zero config: column 0 is S_1
// ============================================================================

TEST_CASE("Space Jacobian at zero config: column 0 is S_1", "[jacobian]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    auto fk = spp::forward_kinematics(chain, q);

    auto J_s = spp::space_jacobian(chain, fk);

    auto s1 = chain.axes()[0].to_vector();
    for (int i = 0; i < 6; ++i)
    {
        REQUIRE(J_s(i, 0) == Approx(s1(i)).margin(1e-12));
    }
}

// ============================================================================
// Space Jacobian dimensions
// ============================================================================

TEST_CASE("Space Jacobian dimensions", "[jacobian]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    auto fk = spp::forward_kinematics(chain, q);

    auto J_s = spp::space_jacobian(chain, fk);

    REQUIRE(J_s.rows() == 6);
    REQUIRE(J_s.cols() == 3);
}

// ============================================================================
// Space Jacobian vs finite-difference at q=0
// ============================================================================

TEST_CASE("Space Jacobian vs finite-difference at q=0", "[jacobian]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    auto fk = spp::forward_kinematics(chain, q);

    auto J_s = spp::space_jacobian(chain, fk);
    auto J_fd = finite_difference_space_jacobian(chain, q);

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            REQUIRE(J_s(r, c) == Approx(J_fd(r, c)).margin(1e-6));
        }
    }
}

// ============================================================================
// Space Jacobian vs finite-difference at q={pi/4, -pi/6, pi/3}
// ============================================================================

TEST_CASE("Space Jacobian vs finite-difference at non-zero q", "[jacobian]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q;
    q << std::numbers::pi / 4.0, -std::numbers::pi / 6.0, std::numbers::pi / 3.0;
    auto fk = spp::forward_kinematics(chain, q);

    auto J_s = spp::space_jacobian(chain, fk);
    auto J_fd = finite_difference_space_jacobian(chain, q);

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            REQUIRE(J_s(r, c) == Approx(J_fd(r, c)).margin(1e-6));
        }
    }
}

// ============================================================================
// Body Jacobian vs Ad_{T^{-1}} * J_s
// ============================================================================

TEST_CASE("Body Jacobian vs Ad_{T^{-1}} * J_s", "[jacobian]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q;
    q << 0.3, -0.5, 0.7;
    auto fk = spp::forward_kinematics(chain, q);

    auto J_s = spp::space_jacobian(chain, fk);
    auto J_b = spp::body_jacobian(chain, fk);

    // Manually compute J_b = Ad_{T^{-1}} * J_s
    auto Ad_inv = fk.end_effector.inverse().adjoint();
    auto J_b_expected = Ad_inv * J_s;

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            REQUIRE(J_b(r, c) == Approx(J_b_expected(r, c)).margin(1e-12));
        }
    }
}

// ============================================================================
// Body Jacobian vs finite-difference
// ============================================================================

TEST_CASE("Body Jacobian vs finite-difference", "[jacobian]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q;
    q << std::numbers::pi / 4.0, -std::numbers::pi / 6.0, std::numbers::pi / 3.0;
    auto fk = spp::forward_kinematics(chain, q);

    auto J_b = spp::body_jacobian(chain, fk);
    auto J_fd = finite_difference_body_jacobian(chain, q);

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            REQUIRE(J_b(r, c) == Approx(J_fd(r, c)).margin(1e-6));
        }
    }
}

// ============================================================================
// Dynamic chain Jacobian matches fixed
// ============================================================================

TEST_CASE("Dynamic chain Jacobian matches fixed", "[jacobian]")
{
    auto fixed_chain = make_3r_chain();
    auto dyn_chain = fixed_chain.to_dynamic();

    Eigen::Vector3d q_fixed;
    q_fixed << 0.3, -0.5, 0.7;

    Eigen::VectorXd q_dyn(3);
    q_dyn << 0.3, -0.5, 0.7;

    auto fk_fixed = spp::forward_kinematics(fixed_chain, q_fixed);
    auto fk_dyn = spp::forward_kinematics(dyn_chain, q_dyn);

    auto J_s_fixed = spp::space_jacobian(fixed_chain, fk_fixed);
    auto J_s_dyn = spp::space_jacobian(dyn_chain, fk_dyn);

    REQUIRE(J_s_dyn.rows() == 6);
    REQUIRE(J_s_dyn.cols() == 3);

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            REQUIRE(J_s_fixed(r, c) == Approx(J_s_dyn(r, c)).margin(1e-12));
        }
    }
}

// ============================================================================
// Float Jacobian compiles and passes
// ============================================================================

TEST_CASE("Float Jacobian compiles and passes", "[jacobian]")
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

    auto J_s = spp::space_jacobian(chain, fk);

    REQUIRE(J_s.rows() == 6);
    REQUIRE(J_s.cols() == 3);

    // Column 0 should equal first screw axis
    auto s1_vec = chain.axes()[0].to_vector();
    for (int i = 0; i < 6; ++i)
    {
        REQUIRE(J_s(i, 0) == Approx(s1_vec(i)).margin(1e-5f));
    }
}
