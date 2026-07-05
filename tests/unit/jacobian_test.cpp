#include "cartan/serial_chain.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = cartan;
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

// ============================================================================
// Generic chain-concept path on a dynamic chain must not write out of bounds
//
// generic_chain_wrapper<kinematic_chain<double, dynamic>> forces the generic
// chain-concept FK/Jacobian overloads. For a dynamic chain those overloads
// must size the intermediate storage and the 6xN Jacobian before writing;
// otherwise they index an empty vector / a 6x0 matrix. The results must also
// match the specialized dynamic-chain path.
// ============================================================================

TEST_CASE("Generic dynamic-chain FK/Jacobian stays in bounds", "[jacobian][dynamic]")
{
    auto dyn = make_3r_chain().to_dynamic();
    spp::detail::generic_chain_wrapper<spp::kinematic_chain<double, spp::dynamic>>
        wrapped{dyn};

    Eigen::VectorXd q(3);
    q << 0.3, -0.5, 0.7;

    auto fk_gen = spp::forward_kinematics(wrapped, q);
    REQUIRE(fk_gen.num_joints() == 3);

    auto J_gen = spp::space_jacobian(wrapped, fk_gen);
    REQUIRE(J_gen.rows() == 6);
    REQUIRE(J_gen.cols() == 3);

    auto fk_ref = spp::forward_kinematics(dyn, q);
    auto J_ref = spp::space_jacobian(dyn, fk_ref);
    REQUIRE((J_gen - J_ref).norm() < 1e-12);

    auto diff = (fk_gen.end_effector.inverse() * fk_ref.end_effector).log();
    REQUIRE(diff.norm() < 1e-12);
}

// ============================================================================
// Zero-joint dynamic chain: space Jacobian is 6x0, not a crash
// ============================================================================

TEST_CASE("Zero-joint dynamic chain space Jacobian is 6x0", "[jacobian][dynamic]")
{
    auto home = spp::se3<double>::identity();
    spp::kinematic_chain<double, spp::dynamic> zero_chain(
        home,
        std::vector<spp::screw_axis<double>>{},
        std::vector<spp::joint_limits<double>>{});

    REQUIRE(zero_chain.num_joints() == 0);

    spp::fk_result<double, spp::dynamic> fk;
    auto J = spp::space_jacobian(zero_chain, fk);

    REQUIRE(J.rows() == 6);
    REQUIRE(J.cols() == 0);
}

// ============================================================================
// static_chain rejects a screw axis that contradicts its compile-time tag
//
// The constructor asserts on axes_match_tags in debug builds; the predicate is
// exercised directly here so the check is observable without aborting the test
// process.
// ============================================================================

TEST_CASE("static_chain rejects axis contradicting its joint tag", "[static_chain][validation]")
{
    using chain_t = spp::static_chain<double, spp::revolute_z>;

    // A revolute screw about +y classifies as revolute_y, contradicting the
    // revolute_z tag.
    std::array<spp::screw_axis<double>, 1> bad_axes{
        spp::screw_axis<double>::revolute({0.0, 1.0, 0.0}, {0.0, 0.0, 0.0})};
    // A revolute screw about +z (or -z) agrees with the revolute_z tag.
    std::array<spp::screw_axis<double>, 1> good_axes{
        spp::screw_axis<double>::revolute({0.0, 0.0, 1.0}, {0.0, 0.0, 0.0})};

    REQUIRE_FALSE(chain_t::axes_match_tags(bad_axes));
    REQUIRE(chain_t::axes_match_tags(good_axes));
}
