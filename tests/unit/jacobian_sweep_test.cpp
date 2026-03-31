#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include "../test_utils.h"

#include <liepp/types.h>
#include <liepp/lie/se3.h>
#include <liepp/chain/kinematic_chain.h>
#include <liepp/kinematics/forward_kinematics.h>
#include <liepp/kinematics/jacobian.h>

#include <numbers>
#include <type_traits>

// ============================================================================
// Finite-difference space Jacobian helper
//
// Column i = log(FK(q + h*e_i) * FK(q - h*e_i)^{-1}) / (2h)
// Central difference, space-frame convention.
// ============================================================================

template <int N, typename Scalar>
liepp::jacobian_matrix<Scalar, N> fd_space_jacobian(
    const liepp::kinematic_chain<Scalar, N>& chain,
    const typename liepp::joint_state<Scalar, N>::position_type& q,
    Scalar h)
{
    int n = chain.num_joints();
    liepp::jacobian_matrix<Scalar, N> J;
    if constexpr (N == liepp::dynamic)
    {
        J.resize(6, n);
    }

    for (int i = 0; i < n; ++i)
    {
        auto q_plus = q;
        auto q_minus = q;
        q_plus(i) += h;
        q_minus(i) -= h;

        auto fk_plus = liepp::forward_kinematics(chain, q_plus);
        auto fk_minus = liepp::forward_kinematics(chain, q_minus);

        // Space-frame: log(T_plus * T_minus^{-1}) / (2h)
        auto delta = (fk_plus.end_effector * fk_minus.end_effector.inverse()).log();
        J.col(i) = delta / (Scalar(2) * h);
    }

    return J;
}

// ============================================================================
// Jacobian sweep: DOF 1-7 x {double, float} x {fixed, dynamic}
//
// For each DOF level:
//   A. Shape correctness (6 x N)
//   B. Space Jacobian vs finite-difference
//   C. Fixed-vs-dynamic comparison
// ============================================================================

TEMPLATE_TEST_CASE("Jacobian sweep: DOF 1-7", "[jacobian][sweep]", double, float)
{
    using Scalar = TestType;
    // Finite-difference tolerance: central-difference O(h^2) truncation error
    // amplified by chain geometry (link lengths ~1m). Expressed as test_eps
    // multiplier scaled by (h/eps)^2 to account for FD error dominance.
    // double: h=1e-6, observed error ~1e-10, multiplier ~1e7 covers with margin
    // float:  h=1e-4, observed error ~1e-3, multiplier ~1e7 covers with margin
    const Scalar fd_tol = Scalar(1e7) * liepp::test::test_eps<Scalar>;
    const Scalar dyn_tol = Scalar(10) * liepp::test::test_eps<Scalar>;
    const Scalar h = std::is_same_v<Scalar, float> ? Scalar(1e-4) : Scalar(1e-6);

    // Helper lambda: run all checks for a given chain
    auto run_checks = [&](auto& chain, int dof, auto& q_fixed)
    {
        auto fk = liepp::forward_kinematics(chain, q_fixed);
        auto Js = liepp::space_jacobian(chain, fk);
        auto Jb = liepp::body_jacobian(chain, fk);

        // A. Shape correctness
        REQUIRE(Js.rows() == 6);
        REQUIRE(Js.cols() == dof);
        REQUIRE(Jb.rows() == 6);
        REQUIRE(Jb.cols() == dof);

        // B. Space Jacobian vs finite-difference
        auto Js_fd = fd_space_jacobian(chain, q_fixed, h);
        for (int col = 0; col < dof; ++col)
        {
            Scalar col_err = (Js.col(col) - Js_fd.col(col)).norm();
            REQUIRE(col_err < fd_tol);
        }
    };

    // ------------------------------------------------------------------
    // DOF 1
    // ------------------------------------------------------------------
    SECTION("DOF 1")
    {
        auto chain = liepp::test::make_1r_chain<Scalar>();

        SECTION("shape and finite-difference")
        {
            Eigen::Vector<Scalar, 1> q;
            q << Scalar(0.1);
            run_checks(chain, 1, q);
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 1> q_fixed;
            q_fixed << Scalar(0.3);
            auto fk_f = liepp::forward_kinematics(chain, q_fixed);
            auto Js_f = liepp::space_jacobian(chain, fk_f);
            auto Jb_f = liepp::body_jacobian(chain, fk_f);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(1);
            q_dyn << Scalar(0.3);
            auto fk_d = liepp::forward_kinematics(dyn_chain, q_dyn);
            auto Js_d = liepp::space_jacobian(dyn_chain, fk_d);
            auto Jb_d = liepp::body_jacobian(dyn_chain, fk_d);

            REQUIRE((Js_f - Js_d).norm() < dyn_tol);
            REQUIRE((Jb_f - Jb_d).norm() < dyn_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 2
    // ------------------------------------------------------------------
    SECTION("DOF 2")
    {
        auto chain = liepp::test::make_2r_planar_chain<Scalar>();

        SECTION("shape and finite-difference")
        {
            Eigen::Vector<Scalar, 2> q;
            q << Scalar(0.1), Scalar(0.1);
            run_checks(chain, 2, q);
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 2> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2);
            auto fk_f = liepp::forward_kinematics(chain, q_fixed);
            auto Js_f = liepp::space_jacobian(chain, fk_f);
            auto Jb_f = liepp::body_jacobian(chain, fk_f);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(2);
            q_dyn << Scalar(0.3), Scalar(-0.2);
            auto fk_d = liepp::forward_kinematics(dyn_chain, q_dyn);
            auto Js_d = liepp::space_jacobian(dyn_chain, fk_d);
            auto Jb_d = liepp::body_jacobian(dyn_chain, fk_d);

            REQUIRE((Js_f - Js_d).norm() < dyn_tol);
            REQUIRE((Jb_f - Jb_d).norm() < dyn_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 3
    // ------------------------------------------------------------------
    SECTION("DOF 3")
    {
        auto chain = liepp::test::make_3r_planar_chain<Scalar>();

        SECTION("shape and finite-difference")
        {
            Eigen::Vector<Scalar, 3> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1);
            run_checks(chain, 3, q);
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 3> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5);
            auto fk_f = liepp::forward_kinematics(chain, q_fixed);
            auto Js_f = liepp::space_jacobian(chain, fk_f);
            auto Jb_f = liepp::body_jacobian(chain, fk_f);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(3);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5);
            auto fk_d = liepp::forward_kinematics(dyn_chain, q_dyn);
            auto Js_d = liepp::space_jacobian(dyn_chain, fk_d);
            auto Jb_d = liepp::body_jacobian(dyn_chain, fk_d);

            REQUIRE((Js_f - Js_d).norm() < dyn_tol);
            REQUIRE((Jb_f - Jb_d).norm() < dyn_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 4
    // ------------------------------------------------------------------
    SECTION("DOF 4")
    {
        auto chain = liepp::test::make_4r_spatial_chain<Scalar>();

        SECTION("shape and finite-difference")
        {
            Eigen::Vector<Scalar, 4> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1);
            run_checks(chain, 4, q);
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 4> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1);
            auto fk_f = liepp::forward_kinematics(chain, q_fixed);
            auto Js_f = liepp::space_jacobian(chain, fk_f);
            auto Jb_f = liepp::body_jacobian(chain, fk_f);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(4);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1);
            auto fk_d = liepp::forward_kinematics(dyn_chain, q_dyn);
            auto Js_d = liepp::space_jacobian(dyn_chain, fk_d);
            auto Jb_d = liepp::body_jacobian(dyn_chain, fk_d);

            REQUIRE((Js_f - Js_d).norm() < dyn_tol);
            REQUIRE((Jb_f - Jb_d).norm() < dyn_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 5
    // ------------------------------------------------------------------
    SECTION("DOF 5")
    {
        auto chain = liepp::test::make_puma560_5dof_chain<Scalar>();

        SECTION("shape and finite-difference")
        {
            Eigen::Vector<Scalar, 5> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1);
            run_checks(chain, 5, q);
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 5> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1), Scalar(0.4);
            auto fk_f = liepp::forward_kinematics(chain, q_fixed);
            auto Js_f = liepp::space_jacobian(chain, fk_f);
            auto Jb_f = liepp::body_jacobian(chain, fk_f);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(5);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1), Scalar(0.4);
            auto fk_d = liepp::forward_kinematics(dyn_chain, q_dyn);
            auto Js_d = liepp::space_jacobian(dyn_chain, fk_d);
            auto Jb_d = liepp::body_jacobian(dyn_chain, fk_d);

            REQUIRE((Js_f - Js_d).norm() < dyn_tol);
            REQUIRE((Jb_f - Jb_d).norm() < dyn_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 6 (UR3e)
    // ------------------------------------------------------------------
    SECTION("DOF 6 (UR3e)")
    {
        auto chain = liepp::test::make_ur3e_chain<Scalar>();

        SECTION("shape and finite-difference")
        {
            Eigen::Vector<Scalar, 6> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1),
                 Scalar(0.1), Scalar(0.1), Scalar(0.1);
            run_checks(chain, 6, q);
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 6> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5),
                       Scalar(-0.1), Scalar(0.4), Scalar(-0.3);
            auto fk_f = liepp::forward_kinematics(chain, q_fixed);
            auto Js_f = liepp::space_jacobian(chain, fk_f);
            auto Jb_f = liepp::body_jacobian(chain, fk_f);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(6);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5),
                     Scalar(-0.1), Scalar(0.4), Scalar(-0.3);
            auto fk_d = liepp::forward_kinematics(dyn_chain, q_dyn);
            auto Js_d = liepp::space_jacobian(dyn_chain, fk_d);
            auto Jb_d = liepp::body_jacobian(dyn_chain, fk_d);

            REQUIRE((Js_f - Js_d).norm() < dyn_tol);
            REQUIRE((Jb_f - Jb_d).norm() < dyn_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 6 (KR6 SIXX)
    // ------------------------------------------------------------------
    SECTION("DOF 6 (KR6 SIXX)")
    {
        auto chain = liepp::test::make_kr6_sixx_chain<Scalar>();

        SECTION("shape and finite-difference")
        {
            Eigen::Vector<Scalar, 6> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1),
                 Scalar(0.1), Scalar(0.1), Scalar(0.1);
            run_checks(chain, 6, q);
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 6> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5),
                       Scalar(-0.1), Scalar(0.4), Scalar(-0.3);
            auto fk_f = liepp::forward_kinematics(chain, q_fixed);
            auto Js_f = liepp::space_jacobian(chain, fk_f);
            auto Jb_f = liepp::body_jacobian(chain, fk_f);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(6);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5),
                     Scalar(-0.1), Scalar(0.4), Scalar(-0.3);
            auto fk_d = liepp::forward_kinematics(dyn_chain, q_dyn);
            auto Js_d = liepp::space_jacobian(dyn_chain, fk_d);
            auto Jb_d = liepp::body_jacobian(dyn_chain, fk_d);

            REQUIRE((Js_f - Js_d).norm() < dyn_tol);
            REQUIRE((Jb_f - Jb_d).norm() < dyn_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 7
    // ------------------------------------------------------------------
    SECTION("DOF 7")
    {
        auto chain = liepp::test::make_lbr_iiwa_chain<Scalar>();

        SECTION("shape and finite-difference")
        {
            Eigen::Vector<Scalar, 7> q;
            q << Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1),
                 Scalar(0.1), Scalar(0.1), Scalar(0.1);
            run_checks(chain, 7, q);
        }

        SECTION("fixed vs dynamic")
        {
            Eigen::Vector<Scalar, 7> q_fixed;
            q_fixed << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1),
                       Scalar(0.4), Scalar(-0.3), Scalar(0.2);
            auto fk_f = liepp::forward_kinematics(chain, q_fixed);
            auto Js_f = liepp::space_jacobian(chain, fk_f);
            auto Jb_f = liepp::body_jacobian(chain, fk_f);

            auto dyn_chain = chain.to_dynamic();
            Eigen::VectorX<Scalar> q_dyn(7);
            q_dyn << Scalar(0.3), Scalar(-0.2), Scalar(0.5), Scalar(-0.1),
                     Scalar(0.4), Scalar(-0.3), Scalar(0.2);
            auto fk_d = liepp::forward_kinematics(dyn_chain, q_dyn);
            auto Js_d = liepp::space_jacobian(dyn_chain, fk_d);
            auto Jb_d = liepp::body_jacobian(dyn_chain, fk_d);

            REQUIRE((Js_f - Js_d).norm() < dyn_tol);
            REQUIRE((Jb_f - Jb_d).norm() < dyn_tol);
        }
    }
}
