#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include "../test_utils.h"

#include <cartan/types.h>
#include <cartan/serial/ik/ik.h>
#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <numbers>
#include <type_traits>

// ============================================================================
// IK sweep: DOF 1-7 x {double, float} x {fixed, dynamic}
//
// Strategy: "known-reachable target" approach:
//   1. Pick known joint config q_known
//   2. Compute target = FK(q_known).end_effector
//   3. Solve IK from q0 = zero
//   4. Verify FK(q_solution) matches target
//
// For each DOF:
//   A. Fixed-chain IK convergence
//   B. Dynamic-chain IK convergence
//   C. Fixed-vs-dynamic FK result comparison
//
// Uses LM stepper (not default DLS) because dls_solve_policy has a known Eigen
// SVD limitation for N=1 chains (1-row dynamic-column block expression
// requires RowMajor storage, which Eigen forbids in certain contexts).
// LM stepper uses LDLT decomposition which works for all DOF values.
// ============================================================================

/// LM-based IK solver alias for sweep testing.
template <typename Scalar, int N>
using lm_ik_solver = cartan::basic_ik_runner<cartan::ik::lm<cartan::kinematic_chain<Scalar, N>>>;

TEMPLATE_TEST_CASE("IK sweep: DOF 1-7", "[ik][sweep]", double, float)
{
    using Scalar = TestType;

    // IK convergence tolerance: precision-appropriate
    const Scalar pos_tol = std::is_same_v<Scalar, float>
        ? Scalar(1e-4) : Scalar(1e-6);
    const Scalar orient_tol = std::is_same_v<Scalar, float>
        ? Scalar(1e-4) : Scalar(1e-6);

    // FK comparison tolerance after IK solve: bounded by convergence tolerance,
    // not machine epsilon. IK converges to pos_tol/orient_tol precision, so the
    // FK residual is O(convergence_tol). Use generous multiplier for margin.
    const Scalar ik_fk_tol = Scalar(100) * pos_tol;
    // Fixed-vs-dynamic FK comparison: both converge to same target within
    // convergence tolerance, so their FK results differ by at most 2 * tol.
    const Scalar dyn_cmp_tol = Scalar(100) * pos_tol;

    const int max_iter = 500;

    // ------------------------------------------------------------------
    // DOF 1
    // ------------------------------------------------------------------
    SECTION("DOF 1")
    {
        auto chain = cartan::test::make_1r_chain<Scalar>();
        Eigen::Vector<Scalar, 1> q_known;
        q_known << Scalar(0.5);

        auto fk_target = cartan::forward_kinematics(chain, q_known);
        auto target = fk_target.end_effector;

        SECTION("fixed-chain IK")
        {
            lm_ik_solver<Scalar, 1> solver;
            Eigen::Vector<Scalar, 1> q0 = Eigen::Vector<Scalar, 1>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("dynamic-chain IK")
        {
            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver;
            Eigen::VectorX<Scalar> q0 = Eigen::VectorX<Scalar>::Zero(1);
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(dyn_chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(dyn_chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("fixed vs dynamic FK comparison")
        {
            lm_ik_solver<Scalar, 1> solver_f;
            Eigen::Vector<Scalar, 1> q0_f = Eigen::Vector<Scalar, 1>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver_f.setup(chain, target, q0_f, criteria);
            auto res_f = solver_f.solve();
            REQUIRE(res_f.has_value());
            auto fk_f = cartan::forward_kinematics(chain, res_f.value().solution.position);

            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver_d;
            Eigen::VectorX<Scalar> q0_d = Eigen::VectorX<Scalar>::Zero(1);
            solver_d.setup(dyn_chain, target, q0_d, criteria);
            auto res_d = solver_d.solve();
            REQUIRE(res_d.has_value());
            auto fk_d = cartan::forward_kinematics(dyn_chain, res_d.value().solution.position);

            Scalar err = (fk_f.end_effector.inverse() * fk_d.end_effector).log().norm();
            REQUIRE(err < dyn_cmp_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 2
    // ------------------------------------------------------------------
    SECTION("DOF 2")
    {
        auto chain = cartan::test::make_2r_planar_chain<Scalar>();
        Eigen::Vector<Scalar, 2> q_known;
        q_known << Scalar(0.3), Scalar(0.3);

        auto fk_target = cartan::forward_kinematics(chain, q_known);
        auto target = fk_target.end_effector;

        SECTION("fixed-chain IK")
        {
            lm_ik_solver<Scalar, 2> solver;
            Eigen::Vector<Scalar, 2> q0 = Eigen::Vector<Scalar, 2>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("dynamic-chain IK")
        {
            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver;
            Eigen::VectorX<Scalar> q0 = Eigen::VectorX<Scalar>::Zero(2);
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(dyn_chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(dyn_chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("fixed vs dynamic FK comparison")
        {
            lm_ik_solver<Scalar, 2> solver_f;
            Eigen::Vector<Scalar, 2> q0_f = Eigen::Vector<Scalar, 2>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver_f.setup(chain, target, q0_f, criteria);
            auto res_f = solver_f.solve();
            REQUIRE(res_f.has_value());
            auto fk_f = cartan::forward_kinematics(chain, res_f.value().solution.position);

            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver_d;
            Eigen::VectorX<Scalar> q0_d = Eigen::VectorX<Scalar>::Zero(2);
            solver_d.setup(dyn_chain, target, q0_d, criteria);
            auto res_d = solver_d.solve();
            REQUIRE(res_d.has_value());
            auto fk_d = cartan::forward_kinematics(dyn_chain, res_d.value().solution.position);

            Scalar err = (fk_f.end_effector.inverse() * fk_d.end_effector).log().norm();
            REQUIRE(err < dyn_cmp_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 3
    // ------------------------------------------------------------------
    SECTION("DOF 3")
    {
        auto chain = cartan::test::make_3r_planar_chain<Scalar>();
        Eigen::Vector<Scalar, 3> q_known;
        q_known << Scalar(0.3), Scalar(0.3), Scalar(0.3);

        auto fk_target = cartan::forward_kinematics(chain, q_known);
        auto target = fk_target.end_effector;

        SECTION("fixed-chain IK")
        {
            lm_ik_solver<Scalar, 3> solver;
            Eigen::Vector<Scalar, 3> q0 = Eigen::Vector<Scalar, 3>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("dynamic-chain IK")
        {
            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver;
            Eigen::VectorX<Scalar> q0 = Eigen::VectorX<Scalar>::Zero(3);
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(dyn_chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(dyn_chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("fixed vs dynamic FK comparison")
        {
            lm_ik_solver<Scalar, 3> solver_f;
            Eigen::Vector<Scalar, 3> q0_f = Eigen::Vector<Scalar, 3>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver_f.setup(chain, target, q0_f, criteria);
            auto res_f = solver_f.solve();
            REQUIRE(res_f.has_value());
            auto fk_f = cartan::forward_kinematics(chain, res_f.value().solution.position);

            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver_d;
            Eigen::VectorX<Scalar> q0_d = Eigen::VectorX<Scalar>::Zero(3);
            solver_d.setup(dyn_chain, target, q0_d, criteria);
            auto res_d = solver_d.solve();
            REQUIRE(res_d.has_value());
            auto fk_d = cartan::forward_kinematics(dyn_chain, res_d.value().solution.position);

            Scalar err = (fk_f.end_effector.inverse() * fk_d.end_effector).log().norm();
            REQUIRE(err < dyn_cmp_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 4
    // ------------------------------------------------------------------
    SECTION("DOF 4")
    {
        auto chain = cartan::test::make_4r_spatial_chain<Scalar>();
        Eigen::Vector<Scalar, 4> q_known;
        q_known << Scalar(0.3), Scalar(0.3), Scalar(0.3), Scalar(0.3);

        auto fk_target = cartan::forward_kinematics(chain, q_known);
        auto target = fk_target.end_effector;

        SECTION("fixed-chain IK")
        {
            lm_ik_solver<Scalar, 4> solver;
            Eigen::Vector<Scalar, 4> q0 = Eigen::Vector<Scalar, 4>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("dynamic-chain IK")
        {
            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver;
            Eigen::VectorX<Scalar> q0 = Eigen::VectorX<Scalar>::Zero(4);
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(dyn_chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(dyn_chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("fixed vs dynamic FK comparison")
        {
            lm_ik_solver<Scalar, 4> solver_f;
            Eigen::Vector<Scalar, 4> q0_f = Eigen::Vector<Scalar, 4>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver_f.setup(chain, target, q0_f, criteria);
            auto res_f = solver_f.solve();
            REQUIRE(res_f.has_value());
            auto fk_f = cartan::forward_kinematics(chain, res_f.value().solution.position);

            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver_d;
            Eigen::VectorX<Scalar> q0_d = Eigen::VectorX<Scalar>::Zero(4);
            solver_d.setup(dyn_chain, target, q0_d, criteria);
            auto res_d = solver_d.solve();
            REQUIRE(res_d.has_value());
            auto fk_d = cartan::forward_kinematics(dyn_chain, res_d.value().solution.position);

            Scalar err = (fk_f.end_effector.inverse() * fk_d.end_effector).log().norm();
            REQUIRE(err < dyn_cmp_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 5
    // ------------------------------------------------------------------
    SECTION("DOF 5")
    {
        auto chain = cartan::test::make_puma560_5dof_chain<Scalar>();
        Eigen::Vector<Scalar, 5> q_known;
        q_known << Scalar(0.3), Scalar(0.3), Scalar(0.3), Scalar(0.3), Scalar(0.3);

        auto fk_target = cartan::forward_kinematics(chain, q_known);
        auto target = fk_target.end_effector;

        SECTION("fixed-chain IK")
        {
            lm_ik_solver<Scalar, 5> solver;
            Eigen::Vector<Scalar, 5> q0 = Eigen::Vector<Scalar, 5>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("dynamic-chain IK")
        {
            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver;
            Eigen::VectorX<Scalar> q0 = Eigen::VectorX<Scalar>::Zero(5);
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(dyn_chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(dyn_chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("fixed vs dynamic FK comparison")
        {
            lm_ik_solver<Scalar, 5> solver_f;
            Eigen::Vector<Scalar, 5> q0_f = Eigen::Vector<Scalar, 5>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver_f.setup(chain, target, q0_f, criteria);
            auto res_f = solver_f.solve();
            REQUIRE(res_f.has_value());
            auto fk_f = cartan::forward_kinematics(chain, res_f.value().solution.position);

            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver_d;
            Eigen::VectorX<Scalar> q0_d = Eigen::VectorX<Scalar>::Zero(5);
            solver_d.setup(dyn_chain, target, q0_d, criteria);
            auto res_d = solver_d.solve();
            REQUIRE(res_d.has_value());
            auto fk_d = cartan::forward_kinematics(dyn_chain, res_d.value().solution.position);

            Scalar err = (fk_f.end_effector.inverse() * fk_d.end_effector).log().norm();
            REQUIRE(err < dyn_cmp_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 6 (UR3e)
    // ------------------------------------------------------------------
    SECTION("DOF 6 (UR3e)")
    {
        auto chain = cartan::test::make_ur3e_chain<Scalar>();
        Eigen::Vector<Scalar, 6> q_known;
        q_known << Scalar(0.3), Scalar(-0.2), Scalar(0.4),
                   Scalar(0.1), Scalar(-0.3), Scalar(0.2);

        auto fk_target = cartan::forward_kinematics(chain, q_known);
        auto target = fk_target.end_effector;

        SECTION("fixed-chain IK")
        {
            lm_ik_solver<Scalar, 6> solver;
            Eigen::Vector<Scalar, 6> q0 = Eigen::Vector<Scalar, 6>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("dynamic-chain IK")
        {
            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver;
            Eigen::VectorX<Scalar> q0 = Eigen::VectorX<Scalar>::Zero(6);
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(dyn_chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(dyn_chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("fixed vs dynamic FK comparison")
        {
            lm_ik_solver<Scalar, 6> solver_f;
            Eigen::Vector<Scalar, 6> q0_f = Eigen::Vector<Scalar, 6>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver_f.setup(chain, target, q0_f, criteria);
            auto res_f = solver_f.solve();
            REQUIRE(res_f.has_value());
            auto fk_f = cartan::forward_kinematics(chain, res_f.value().solution.position);

            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver_d;
            Eigen::VectorX<Scalar> q0_d = Eigen::VectorX<Scalar>::Zero(6);
            solver_d.setup(dyn_chain, target, q0_d, criteria);
            auto res_d = solver_d.solve();
            REQUIRE(res_d.has_value());
            auto fk_d = cartan::forward_kinematics(dyn_chain, res_d.value().solution.position);

            Scalar err = (fk_f.end_effector.inverse() * fk_d.end_effector).log().norm();
            REQUIRE(err < dyn_cmp_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 6 (KR6 SIXX)
    // ------------------------------------------------------------------
    SECTION("DOF 6 (KR6 SIXX)")
    {
        auto chain = cartan::test::make_kr6_sixx_chain<Scalar>();
        // Very small joint angles for KR6 SIXX: the KR6 has long links
        // (~0.935m reach) so even small angles produce large displacements.
        // Float IK needs the target within the convergence basin of zero seed.
        Eigen::Vector<Scalar, 6> q_known;
        q_known << Scalar(0.1), Scalar(-0.05), Scalar(0.1),
                   Scalar(0.05), Scalar(-0.05), Scalar(0.05);

        auto fk_target = cartan::forward_kinematics(chain, q_known);
        auto target = fk_target.end_effector;

        // KR6 SIXX has x-axis roll joints (4,6) that create complex coupling.
        // For float, use relaxed convergence and more iterations.
        const Scalar kr6_pos_tol = std::is_same_v<Scalar, float>
            ? Scalar(1e-3) : pos_tol;
        const Scalar kr6_orient_tol = std::is_same_v<Scalar, float>
            ? Scalar(1e-3) : orient_tol;
        const Scalar kr6_fk_tol = Scalar(100) * kr6_pos_tol;
        const int kr6_max_iter = 1000;

        SECTION("fixed-chain IK")
        {
            lm_ik_solver<Scalar, 6> solver;
            Eigen::Vector<Scalar, 6> q0 = Eigen::Vector<Scalar, 6>::Zero();
            cartan::convergence_criteria<Scalar> criteria{kr6_pos_tol, kr6_orient_tol, kr6_max_iter};
            solver.setup(chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < kr6_fk_tol);
        }

        SECTION("dynamic-chain IK")
        {
            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver;
            Eigen::VectorX<Scalar> q0_d = Eigen::VectorX<Scalar>::Zero(6);
            cartan::convergence_criteria<Scalar> criteria{kr6_pos_tol, kr6_orient_tol, kr6_max_iter};
            solver.setup(dyn_chain, target, q0_d, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(dyn_chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < kr6_fk_tol);
        }

        SECTION("fixed vs dynamic FK comparison")
        {
            lm_ik_solver<Scalar, 6> solver_f;
            Eigen::Vector<Scalar, 6> q0_f = Eigen::Vector<Scalar, 6>::Zero();
            cartan::convergence_criteria<Scalar> criteria{kr6_pos_tol, kr6_orient_tol, kr6_max_iter};
            solver_f.setup(chain, target, q0_f, criteria);
            auto res_f = solver_f.solve();
            REQUIRE(res_f.has_value());
            auto fk_f = cartan::forward_kinematics(chain, res_f.value().solution.position);

            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver_d;
            Eigen::VectorX<Scalar> q0_d = Eigen::VectorX<Scalar>::Zero(6);
            solver_d.setup(dyn_chain, target, q0_d, criteria);
            auto res_d = solver_d.solve();
            REQUIRE(res_d.has_value());
            auto fk_d = cartan::forward_kinematics(dyn_chain, res_d.value().solution.position);

            Scalar err = (fk_f.end_effector.inverse() * fk_d.end_effector).log().norm();
            REQUIRE(err < kr6_fk_tol);
        }
    }

    // ------------------------------------------------------------------
    // DOF 7
    // ------------------------------------------------------------------
    SECTION("DOF 7")
    {
        auto chain = cartan::test::make_lbr_iiwa_chain<Scalar>();
        Eigen::Vector<Scalar, 7> q_known;
        q_known << Scalar(0.3), Scalar(-0.2), Scalar(0.4), Scalar(0.1),
                   Scalar(-0.3), Scalar(0.2), Scalar(-0.1);

        auto fk_target = cartan::forward_kinematics(chain, q_known);
        auto target = fk_target.end_effector;

        SECTION("fixed-chain IK")
        {
            lm_ik_solver<Scalar, 7> solver;
            Eigen::Vector<Scalar, 7> q0 = Eigen::Vector<Scalar, 7>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("dynamic-chain IK")
        {
            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver;
            Eigen::VectorX<Scalar> q0 = Eigen::VectorX<Scalar>::Zero(7);
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver.setup(dyn_chain, target, q0, criteria);
            auto result = solver.solve();
            REQUIRE(result.has_value());

            auto fk_sol = cartan::forward_kinematics(dyn_chain, result.value().solution.position);
            Scalar err = (fk_sol.end_effector.inverse() * target).log().norm();
            REQUIRE(err < ik_fk_tol);
        }

        SECTION("fixed vs dynamic FK comparison")
        {
            lm_ik_solver<Scalar, 7> solver_f;
            Eigen::Vector<Scalar, 7> q0_f = Eigen::Vector<Scalar, 7>::Zero();
            cartan::convergence_criteria<Scalar> criteria{pos_tol, orient_tol, max_iter};
            solver_f.setup(chain, target, q0_f, criteria);
            auto res_f = solver_f.solve();
            REQUIRE(res_f.has_value());
            auto fk_f = cartan::forward_kinematics(chain, res_f.value().solution.position);

            auto dyn_chain = chain.to_dynamic();
            lm_ik_solver<Scalar, cartan::dynamic> solver_d;
            Eigen::VectorX<Scalar> q0_d = Eigen::VectorX<Scalar>::Zero(7);
            solver_d.setup(dyn_chain, target, q0_d, criteria);
            auto res_d = solver_d.solve();
            REQUIRE(res_d.has_value());
            auto fk_d = cartan::forward_kinematics(dyn_chain, res_d.value().solution.position);

            Scalar err = (fk_f.end_effector.inverse() * fk_d.end_effector).log().norm();
            REQUIRE(err < dyn_cmp_tol);
        }
    }
}
