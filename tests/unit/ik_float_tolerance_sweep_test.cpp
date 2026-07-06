#ifndef HPP_GUARD_TESTS_UNIT_IK_FLOAT_TOLERANCE_SWEEP_TEST_H
#define HPP_GUARD_TESTS_UNIT_IK_FLOAT_TOLERANCE_SWEEP_TEST_H

#include "../test_utils.h"

#include <cartan/types.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <vector>
#include <random>
#include <cstdio>
#include <numbers>
#include <algorithm>

// ============================================================================
// Float IK convergence tolerance: reach x joint-count round-off sweep.
//
// The IK convergence gate `convergence_criteria<Scalar>` historically defaulted
// its position/orientation tolerance to 1e-6 for every Scalar. Two floors sit
// above that gate in float and are measured here:
//
//   1. FLOAT FK ROUND-OFF FLOOR (fundamental). float epsilon ~1.19e-7; a
//      metre-scale forward-kinematics product accumulates round-off at
//      O(reach * eps * sqrt(N)). Measured against a double FK oracle (same
//      geometry, same joint vector) the 95th-percentile floor is ~1e-6 m /
//      ~2.5e-7 rad -- already at the 1e-6 gate.
//   2. FLOAT SOLVER RESIDUAL FLOOR (binding). The Levenberg-Marquardt stepper's
//      own float residual floors well above the FK round-off floor: on
//      metre-scale chains its achievable pose residual is broadly distributed in
//      the ~1e-5..5e-4 band. THIS is the constraint that actually gates a float
//      solve -- the runner tests the solver's residual against the tolerance, so
//      the tolerance must clear the solver floor, not merely the FK floor.
//
// The float default is set to 1e-4 (position and orientation): ~100x the
// measured FK round-off floor and inside the solver residual band, so a float
// solve can terminate as converged where a 1e-6 gate can never be met. The
// double default is left at exactly 1e-6 (double FK round-off floors near 1e-13,
// far below the gate).
//
// RECORDED FK ROUND-OFF SWEEP (float vs double oracle, 512 samples/cell over the
// reach x joint-count grid; position m, orientation rad). Live figures are
// printed by the test; representative values from this build:
//
//   chain (N, ~reach m)  | pos 95th-pct | pos max  | ori 95th-pct | ori max
//   1r        (1, 1.0)   | 1.8e-07      | 2.6e-07  | 1.1e-07      | 1.5e-07
//   2r planar (2, 2.0)   | 6.5e-07      | 1.0e-06  | 1.7e-07      | 2.2e-07
//   3r planar (3, 3.0)   | 1.5e-06      | 2.3e-06  | 2.0e-07      | 3.6e-07
//   4r spatial(4, 0.8)   | 5.8e-07      | 1.3e-06  | 2.3e-07      | 3.3e-07
//   puma560   (5, 0.9)   | 9.3e-07      | 1.8e-06  | 2.6e-07      | 3.6e-07
//   kr6 sixx  (6, 0.94)  | 1.0e-06      | 1.9e-06  | 2.8e-07      | 4.3e-07
//   lbr iiwa  (7, 1.3)   | 1.3e-06      | 2.2e-06  | 3.1e-07      | 4.1e-07
//
//   grid aggregate: pos p95 1.0e-06 m (max 2.3e-06), ori p95 2.5e-07 rad
//   (max 4.3e-07). Float IK convergence contrast (metre-scale, near-solution
//   seed): a 1e-6 gate terminates ~0% of solves, the 1e-4 preset ~56%.
//
// The live asserts recompute the floor and gate against the measured figures, so
// the table is documentation, not the source of truth.
// ============================================================================

namespace
{

// 95th-percentile of a value list (nearest-rank).
double percentile95(std::vector<double> v)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    auto idx = static_cast<std::size_t>(0.95 * static_cast<double>(v.size() - 1));
    return v[idx];
}

double maximum(const std::vector<double>& v)
{
    return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());
}

// One grid cell: float-vs-double FK round-off over `samples` random joint
// vectors on identical chain geometry. Appends per-sample floors to pos/ori and
// prints the per-chain 95th-percentile/max for the recorded table.
template <int N>
void accumulate_cell(
    const cartan::kinematic_chain<double, N>& chain_d,
    const cartan::kinematic_chain<float, N>& chain_f,
    const char* label,
    std::mt19937_64& rng,
    int samples,
    std::vector<double>& pos,
    std::vector<double>& ori)
{
    std::uniform_real_distribution<double> dist(
        -std::numbers::pi, std::numbers::pi);

    std::vector<double> cell_pos;
    std::vector<double> cell_ori;
    cell_pos.reserve(static_cast<std::size_t>(samples));
    cell_ori.reserve(static_cast<std::size_t>(samples));

    for (int s = 0; s < samples; ++s)
    {
        Eigen::Vector<double, N> q_d;
        for (int i = 0; i < N; ++i)
            q_d(i) = dist(rng);
        Eigen::Vector<float, N> q_f = q_d.template cast<float>();

        auto T_d = cartan::forward_kinematics(chain_d, q_d).end_effector;
        auto T_f = cartan::forward_kinematics(chain_f, q_f).end_effector;

        // Position floor: ||p_float - p_double|| on the common geometry.
        Eigen::Vector3d p_d = T_d.translation();
        Eigen::Vector3d p_f = T_f.translation().template cast<double>();
        cell_pos.push_back((p_f - p_d).norm());

        // Orientation floor: ||log(R_float^T * R_double)||, the float rotation
        // cast up to double so the difference is not itself limited by float eps.
        cartan::so3<double> R_d = T_d.rotation();
        Eigen::Quaterniond q_rot_f =
            T_f.rotation().quaternion_ref().template cast<double>();
        cartan::so3<double> R_f(q_rot_f);
        cell_ori.push_back((R_f.inverse() * R_d).log().norm());
    }

    std::printf(
        "  %-10s (N=%d): pos p95=%.3e max=%.3e | ori p95=%.3e max=%.3e\n",
        label, N,
        percentile95(cell_pos), maximum(cell_pos),
        percentile95(cell_ori), maximum(cell_ori));

    pos.insert(pos.end(), cell_pos.begin(), cell_pos.end());
    ori.insert(ori.end(), cell_ori.begin(), cell_ori.end());
}

}

// ----------------------------------------------------------------------------
// FK round-off sweep + preset sufficiency + double-unchanged pin.
// ----------------------------------------------------------------------------
TEST_CASE("float FK round-off floor bounds the IK tolerance preset",
    "[ik][float][sweep]")
{
    constexpr int samples = 512; // >= 256 per cell

    std::mt19937_64 rng(0xC0FFEEu);
    std::vector<double> pos;
    std::vector<double> ori;

    std::printf("[ik-float-sweep] per-chain float FK round-off vs double oracle:\n");

    // reach x joint-count grid: N = 1..7 via the shared geometry factories.
    {
        auto cd = cartan::test::make_1r_chain<double>();
        auto cf = cartan::test::make_1r_chain<float>();
        accumulate_cell(cd, cf, "1r", rng, samples, pos, ori);
    }
    {
        auto cd = cartan::test::make_2r_planar_chain<double>();
        auto cf = cartan::test::make_2r_planar_chain<float>();
        accumulate_cell(cd, cf, "2r-planar", rng, samples, pos, ori);
    }
    {
        auto cd = cartan::test::make_3r_planar_chain<double>();
        auto cf = cartan::test::make_3r_planar_chain<float>();
        accumulate_cell(cd, cf, "3r-planar", rng, samples, pos, ori);
    }
    {
        auto cd = cartan::test::make_4r_spatial_chain<double>();
        auto cf = cartan::test::make_4r_spatial_chain<float>();
        accumulate_cell(cd, cf, "4r-spatial", rng, samples, pos, ori);
    }
    {
        auto cd = cartan::test::make_puma560_5dof_chain<double>();
        auto cf = cartan::test::make_puma560_5dof_chain<float>();
        accumulate_cell(cd, cf, "puma560", rng, samples, pos, ori);
    }
    {
        auto cd = cartan::test::make_kr6_sixx_chain<double>();
        auto cf = cartan::test::make_kr6_sixx_chain<float>();
        accumulate_cell(cd, cf, "kr6-sixx", rng, samples, pos, ori);
    }
    {
        auto cd = cartan::test::make_lbr_iiwa_chain<double>();
        auto cf = cartan::test::make_lbr_iiwa_chain<float>();
        accumulate_cell(cd, cf, "lbr-iiwa", rng, samples, pos, ori);
    }

    const double pos_p95 = percentile95(pos);
    const double ori_p95 = percentile95(ori);
    const double pos_max = maximum(pos);
    const double ori_max = maximum(ori);

    std::printf(
        "  aggregate (%zu samples): pos p95=%.3e max=%.3e | ori p95=%.3e max=%.3e\n"
        "  float preset: pos=%.3e ori=%.3e\n",
        pos.size(), pos_p95, pos_max, ori_p95, ori_max,
        static_cast<double>(cartan::detail::default_position_tol_v<float>),
        static_cast<double>(cartan::detail::default_orientation_tol_v<float>));

    // (b) PRESET SUFFICIENCY: the float default must sit safely (>= 5x) above
    // the measured 95th-percentile FK round-off floor. This fails on the pre-fix
    // code where the float default is 1e-6 (below the measured float floor).
    const float float_pos_tol = cartan::convergence_criteria<float>{}.position_tol;
    const float float_ori_tol = cartan::convergence_criteria<float>{}.orientation_tol;

    REQUIRE(static_cast<double>(float_pos_tol) >= 5.0 * pos_p95);
    REQUIRE(static_cast<double>(float_ori_tol) >= 5.0 * ori_p95);

    // The preset must also clear the observed maximum floor, otherwise a solve
    // near the worst-case pose could still floor above the gate.
    REQUIRE(static_cast<double>(float_pos_tol) > pos_max);
    REQUIRE(static_cast<double>(float_ori_tol) > ori_max);

    // The traits back the struct member initializers.
    REQUIRE(float_pos_tol == cartan::detail::default_position_tol_v<float>);
    REQUIRE(float_ori_tol == cartan::detail::default_orientation_tol_v<float>);

    // (c) DOUBLE UNCHANGED: the double default must stay exactly 1e-6 -- guard
    // against silently regressing the double convergence gate.
    REQUIRE(cartan::convergence_criteria<double>{}.position_tol == 1e-6);
    REQUIRE(cartan::convergence_criteria<double>{}.orientation_tol == 1e-6);
    REQUIRE(cartan::detail::default_position_tol_v<double> == 1e-6);
    REQUIRE(cartan::detail::default_orientation_tol_v<double> == 1e-6);
}

// ----------------------------------------------------------------------------
// Two-way float IK demonstration: a forced 1e-6 gate cannot converge, the swept
// preset does -- convergence re-verified via FK + twist-error (not the solver's
// self-reported status).
// ----------------------------------------------------------------------------
TEST_CASE("float IK converges at the swept preset but not at a forced 1e-6 gate",
    "[ik][float][convergence]")
{
    using Chain = cartan::kinematic_chain<float, 7>;

    // ~1.3 m reach 7-DOF chain: its float FK round-off dominates the sweep.
    auto chain = cartan::test::make_lbr_iiwa_chain<float>();

    Eigen::Vector<float, 7> q_known;
    q_known << 0.15f, 0.1f, -0.2f, 0.25f, 0.1f, -0.15f, 0.2f;
    auto target = cartan::forward_kinematics(chain, q_known).end_effector;

    Eigen::Vector<float, 7> q0 = Eigen::Vector<float, 7>::Zero();

    // Generous budget so a failure is a floor issue, not an exhausted budget.
    const int max_attempt = 800;
    const int max_units = 1600;

    // RED: force the historical 1e-6 gate. The float residual floors above it,
    // so the runner exhausts its budget without ever reporting convergence.
    {
        cartan::basic_ik_runner<cartan::lm<Chain>> solver;
        cartan::convergence_criteria<float> forced;
        forced.position_tol = 1e-6f;
        forced.orientation_tol = 1e-6f;
        forced.max_iterations_per_attempt = max_attempt;
        forced.max_total_work_units = max_units;

        solver.setup(chain, target, q0, forced);
        auto result = solver.solve();

        REQUIRE_FALSE(result.has_value());
    }

    // GREEN: the DEFAULT float criteria carry the swept preset. The solve
    // converges; re-verify the pose independently via FK + twist error rather
    // than trusting the solver's self-reported status. In se3::log() the head
    // block is angular (orientation) and the tail block is linear (position).
    {
        cartan::basic_ik_runner<cartan::lm<Chain>> solver;
        cartan::convergence_criteria<float> criteria; // swept float preset
        criteria.max_iterations_per_attempt = max_attempt;
        criteria.max_total_work_units = max_units;

        solver.setup(chain, target, q0, criteria);
        auto result = solver.solve();

        REQUIRE(result.has_value());

        auto fk_sol = cartan::forward_kinematics(chain, result->solution.position);
        auto err = (fk_sol.end_effector.inverse() * target).log();
        const double ori_err = err.head<3>().norm();
        const double pos_err = err.tail<3>().norm();

        std::printf(
            "[ik-float-conv] re-verified pose error: pos=%.3e m, ori=%.3e rad "
            "(preset pos=%.3e, ori=%.3e)\n",
            pos_err, ori_err,
            static_cast<double>(criteria.position_tol),
            static_cast<double>(criteria.orientation_tol));

        // Converged within the swept preset...
        REQUIRE(pos_err < static_cast<double>(criteria.position_tol));
        REQUIRE(ori_err < static_cast<double>(criteria.orientation_tol));

        // ...yet the achieved float accuracy is above the historical 1e-6 gate,
        // which is exactly why the forced-1e-6 run above cannot terminate.
        REQUIRE(pos_err > 1e-6);
    }
}

// ----------------------------------------------------------------------------
// Statistical convergence contrast: across many random reachable targets on
// metre-scale float chains, a 1e-6 gate almost never terminates while the swept
// preset terminates on a large fraction -- generalizing the single two-way
// demonstration and recording where the float solver residual floor sits.
//
// Seeds near the known solution to isolate the numerical residual floor from
// Levenberg-Marquardt basin-escape failures. Deterministic (fixed generator).
// ----------------------------------------------------------------------------
namespace
{

template <int N>
void convergence_contrast(
    const cartan::kinematic_chain<float, N>& chain,
    std::mt19937_64& rng,
    int trials,
    int& conv_1e6,
    int& conv_preset,
    int& total)
{
    std::uniform_real_distribution<double> dist(-0.6, 0.6);

    for (int t = 0; t < trials; ++t)
    {
        Eigen::Vector<float, N> q_known;
        for (int i = 0; i < N; ++i)
            q_known(i) = static_cast<float>(dist(rng));
        auto target = cartan::forward_kinematics(chain, q_known).end_effector;

        Eigen::Vector<float, N> q0 = q_known;
        for (int i = 0; i < N; ++i)
            q0(i) += static_cast<float>(0.05 * dist(rng));

        ++total;

        {
            cartan::basic_ik_runner<cartan::lm<cartan::kinematic_chain<float, N>>> s;
            cartan::convergence_criteria<float> c;
            c.position_tol = 1e-6f;
            c.orientation_tol = 1e-6f;
            c.max_iterations_per_attempt = 800;
            c.max_total_work_units = 1600;
            s.setup(chain, target, q0, c);
            if (s.solve().has_value())
                ++conv_1e6;
        }
        {
            cartan::basic_ik_runner<cartan::lm<cartan::kinematic_chain<float, N>>> s;
            cartan::convergence_criteria<float> c; // swept preset
            c.max_iterations_per_attempt = 800;
            c.max_total_work_units = 1600;
            s.setup(chain, target, q0, c);
            if (s.solve().has_value())
                ++conv_preset;
        }
    }
}

}

TEST_CASE("float IK: swept preset terminates where a 1e-6 gate cannot",
    "[ik][float][convergence]")
{
    std::mt19937_64 rng(0xABCDEFu);
    int conv_1e6 = 0;
    int conv_preset = 0;
    int total = 0;

    {
        auto chain = cartan::test::make_lbr_iiwa_chain<float>();
        convergence_contrast(chain, rng, 75, conv_1e6, conv_preset, total);
    }
    {
        auto chain = cartan::test::make_kr6_sixx_chain<float>();
        convergence_contrast(chain, rng, 75, conv_1e6, conv_preset, total);
    }

    std::printf(
        "[ik-float-contrast] over %d metre-scale float targets: "
        "conv@1e-6 = %d (%.1f%%), conv@preset = %d (%.1f%%)\n",
        total, conv_1e6, 100.0 * conv_1e6 / total,
        conv_preset, 100.0 * conv_preset / total);

    // The historical 1e-6 gate is essentially unreachable in float: the solver
    // residual floors above it, so almost nothing terminates as converged.
    REQUIRE(conv_1e6 <= total / 10);

    // The swept preset lifts the gate above the float solver residual floor for
    // a large fraction of targets -- the float solve is genuinely unblocked.
    REQUIRE(conv_preset >= total / 4);
    REQUIRE(conv_preset > 5 * conv_1e6);
}

#endif
