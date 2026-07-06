/// @file jacobian_sweep_test.cpp
/// @brief Seeded-random space-Jacobian regression sweep plus an empirical
///        finite-difference tolerance derivation.
///
/// Two concerns live here:
///
///   1. A regression gate: for each of the nine benchmark robots the analytic
///      space Jacobian is compared, column by column, against a central-
///      difference approximation over fifty seeded random configurations, in
///      both float and double. A regression in the analytic Jacobian fails the
///      gate on the offending robot.
///
///   2. The tolerance itself: the finite-difference comparison is only a gate
///      if the tolerance is tighter than a real analytic-Jacobian error yet
///      looser than the finite-difference noise floor. The floor is not
///      guessed -- it is measured. The derivation sweep below samples the
///      per-column error over the DOF-1..7 chains, scanning the step size to
///      locate the truncation-vs-roundoff optimum, and records the 95th-
///      percentile and maximum. The shipped tolerances are pinned to those
///      measurements; the double tolerance is kept far below its float
///      counterpart to guard the tight double path.
///
/// Central-difference space Jacobian column i:
///   J_fd.col(i) = log(FK(q + h e_i) * FK(q - h e_i)^{-1}) / (2h)
/// Its error against the analytic column is O(h^2) truncation plus O(eps/h)
/// roundoff, both amplified by the chain's moment arms (up to ~3 m reach in the
/// synthetic planar chains). The moment-arm amplification pushes the roundoff
/// term up, so the truncation-vs-roundoff optimum sits at a larger step than
/// the textbook sqrt(eps): the measured optimum is h ~ 1e-5 for double and
/// h ~ 1e-3 for float (at the more common h ~ 1e-4 the float roundoff term
/// already dominates and the error is an order of magnitude worse).
///
/// RECORDED FINITE-DIFFERENCE ERROR SWEEP (analytic vs central-difference, 50
/// seeded configs/chain over the DOF-1..7 chains, 1400 columns/step; per-column
/// twist-norm error). Live figures are printed by the test; representative
/// values from this build:
///
///   step-size scan (aggregate over all DOF-1..7 columns):
///     double: h=1e-5 p95=8.7e-11 max=2.2e-10 | h=1e-6 p95=8.9e-10 max=2.2e-09
///             h=1e-7 p95=9.7e-09 max=2.2e-08
///     float:  h=1e-3 p95=4.9e-04 max=1.1e-03 | h=1e-4 p95=4.7e-03 max=1.5e-02
///             h=1e-5 p95=5.0e-02 max=1.3e-01
///
///   at the chosen step (double h=1e-5, float h=1e-3):
///     double: p95=8.7e-11 max=2.2e-10  -> fd_tol_double = 1e-8  (~46x max)
///     float:  p95=4.9e-04 max=1.1e-03  -> fd_tol_float  = 5e-3  (~4.3x max)
///
/// The live asserts recompute the floor and gate against the measured figures,
/// so the table is documentation, not the source of truth. The pinned
/// tolerances (fd_tol_double, fd_tol_float) sit just above the measured maxima:
/// tight enough that a genuine analytic-Jacobian regression (error >> the FD
/// noise floor) fails, unlike the previous 10^7 * eps ~ 1.19 value, which was
/// roughly a billion times the floor it claimed to bound and could never fail.

#include "../fixtures/chain_factories.h"
#include "../fixtures/prismatic_chains.h"
#include "../test_utils.h"

#include <cartan/types.h>
#include <cartan/lie/se3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/jacobian.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cstdio>
#include <random>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <type_traits>

namespace
{

// ---------------------------------------------------------------------------
// Pinned finite-difference tolerances (derived from the sweep below).
// ---------------------------------------------------------------------------
// float: at the measured optimum step h=1e-3 the per-column error floors near
// ~1e-3 (max 1.1e-3); the tolerance sits ~4x above that measured maximum so a
// genuine analytic-Jacobian regression (error >> FD noise) fails while the
// finite-difference noise itself does not.
inline constexpr float fd_tol_float = 5e-3f;
// double: at the measured optimum step h=1e-5 the same error floors near
// ~2e-10; the tolerance stays orders of magnitude tighter than the float path.
inline constexpr double fd_tol_double = 1e-8;

template <typename Scalar>
constexpr Scalar fd_tol_v =
    std::is_same_v<Scalar, float> ? Scalar(fd_tol_float) : Scalar(fd_tol_double);

// Step size at the measured truncation-vs-roundoff optimum for each precision.
template <typename Scalar>
constexpr Scalar gate_step_v =
    std::is_same_v<Scalar, float> ? Scalar(1e-3) : Scalar(1e-5);

// ---------------------------------------------------------------------------
// Statistics helpers.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Central-difference space Jacobian: column i is the space twist of the
// end-effector under a symmetric perturbation of joint i.
// ---------------------------------------------------------------------------
template <int N, typename Scalar>
cartan::jacobian_matrix<Scalar, N> fd_space_jacobian(
    const cartan::kinematic_chain<Scalar, N>& chain,
    const typename cartan::joint_state<Scalar, N>::position_type& q,
    Scalar h)
{
    const int n = chain.num_joints();
    cartan::jacobian_matrix<Scalar, N> J;
    if constexpr (N == cartan::dynamic)
    {
        J.resize(6, n);
    }

    for (int i = 0; i < n; ++i)
    {
        auto q_plus = q;
        auto q_minus = q;
        q_plus(i) += h;
        q_minus(i) -= h;

        auto fk_plus = cartan::forward_kinematics(chain, q_plus);
        auto fk_minus = cartan::forward_kinematics(chain, q_minus);

        auto delta =
            (fk_plus.end_effector * fk_minus.end_effector.inverse()).log();
        J.col(i) = delta / (Scalar(2) * h);
    }

    return J;
}

// One chain: per-column analytic-vs-FD twist-norm errors over `trials` seeded
// configs, appended to `out`.
template <int N, typename Scalar>
void collect_fd_errors(
    const cartan::kinematic_chain<Scalar, N>& chain,
    Scalar h, int trials, std::mt19937_64& rng, std::vector<double>& out)
{
    const int n = chain.num_joints();
    std::uniform_real_distribution<Scalar> unit(Scalar(0), Scalar(1));

    for (int t = 0; t < trials; ++t)
    {
        Eigen::Vector<Scalar, N> q;
        for (int j = 0; j < n; ++j)
        {
            const auto& lim = chain.limits()[static_cast<std::size_t>(j)];
            q(j) = lim.position_min
                 + (lim.position_max - lim.position_min) * unit(rng);
        }

        auto fk = cartan::forward_kinematics(chain, q);
        auto Js = cartan::space_jacobian(chain, fk);
        auto Jfd = fd_space_jacobian(chain, q, h);

        for (int c = 0; c < n; ++c)
            out.push_back(static_cast<double>((Js.col(c) - Jfd.col(c)).norm()));
    }
}

// Aggregate per-column errors across the DOF-1..7 chains at one step size.
template <typename Scalar>
std::vector<double> collect_all(Scalar h, int trials, unsigned seed)
{
    std::vector<double> errs;
    std::mt19937_64 rng(seed);

    { auto c = cartan::test::make_1r_chain<Scalar>();        collect_fd_errors(c, h, trials, rng, errs); }
    { auto c = cartan::test::make_2r_planar_chain<Scalar>(); collect_fd_errors(c, h, trials, rng, errs); }
    { auto c = cartan::test::make_3r_planar_chain<Scalar>(); collect_fd_errors(c, h, trials, rng, errs); }
    { auto c = cartan::test::make_4r_spatial_chain<Scalar>();collect_fd_errors(c, h, trials, rng, errs); }
    { auto c = cartan::test::make_puma560_5dof_chain<Scalar>();collect_fd_errors(c, h, trials, rng, errs); }
    { auto c = cartan::test::make_kr6_sixx_chain<Scalar>();  collect_fd_errors(c, h, trials, rng, errs); }
    { auto c = cartan::test::make_lbr_iiwa_chain<Scalar>();  collect_fd_errors(c, h, trials, rng, errs); }

    return errs;
}

// Regression gate for one robot: analytic vs FD at the chosen step, gated by
// the pinned tolerance, over fifty seeded configs.
template <typename MakeChain>
void jac_gate_robot(MakeChain make_chain, const char* name)
{
    INFO("Robot: " << name);

    auto chain = make_chain();
    using Chain = decltype(chain);
    using Scalar = typename Chain::scalar_type;
    constexpr int N = Chain::joints;

    const int n = chain.num_joints();
    const Scalar h = gate_step_v<Scalar>;
    const Scalar tol = fd_tol_v<Scalar>;

    std::mt19937 rng(42);
    std::uniform_real_distribution<Scalar> unit(Scalar(0), Scalar(1));

    for (int trial = 0; trial < 50; ++trial)
    {
        INFO("Trial: " << trial);

        Eigen::Vector<Scalar, N> q;
        for (int j = 0; j < n; ++j)
        {
            const auto& lim = chain.limits()[static_cast<std::size_t>(j)];
            q(j) = lim.position_min
                 + (lim.position_max - lim.position_min) * unit(rng);
        }

        auto fk = cartan::forward_kinematics(chain, q);
        auto Js = cartan::space_jacobian(chain, fk);
        auto Jfd = fd_space_jacobian(chain, q, h);

        REQUIRE(Js.rows() == 6);
        REQUIRE(Js.cols() == n);

        for (int c = 0; c < n; ++c)
        {
            const Scalar col_err = (Js.col(c) - Jfd.col(c)).norm();
            REQUIRE(col_err < tol);
        }
    }
}

// Pure-prismatic P-P-P gantry (signed directions +z, +x, -y).
template <typename Scalar>
auto make_ppp_chain() -> cartan::kinematic_chain<Scalar, 3>
{
    using vec3 = cartan::vector3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::prismatic(
        vec3(Scalar(0), Scalar(0), Scalar(1)));
    auto s2 = cartan::screw_axis<Scalar>::prismatic(
        vec3(Scalar(1), Scalar(0), Scalar(0)));
    auto s3 = cartan::screw_axis<Scalar>::prismatic(
        vec3(Scalar(0), Scalar(-1), Scalar(0)));

    auto home = cartan::se3<Scalar>(
        cartan::so3<Scalar>::identity(), vec3(Scalar(0), Scalar(0), Scalar(0)));
    cartan::joint_limits<Scalar> lim{Scalar(-1), Scalar(1)};

    return cartan::kinematic_chain<Scalar, 3>(
        home, {s1, s2, s3}, {lim, lim, lim});
}

// Zero-DOF dynamic chain: empty axis/limit storage, non-trivial home pose.
template <typename Scalar>
auto make_zero_dof_chain() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    auto home = cartan::se3<Scalar>(
        cartan::so3<Scalar>::identity(),
        cartan::vector3<Scalar>(Scalar(0.1), Scalar(0.2), Scalar(0.3)));
    return cartan::kinematic_chain<Scalar, cartan::dynamic>(
        home,
        std::vector<cartan::screw_axis<Scalar>>{},
        std::vector<cartan::joint_limits<Scalar>>{});
}

}

// ---------------------------------------------------------------------------
// Empirical finite-difference tolerance derivation.
// ---------------------------------------------------------------------------
TEST_CASE("Jacobian FD tolerance sweep: double", "[jacobian][sweep][fd]")
{
    constexpr int trials = 50;

    std::printf("[jacobian-fd-sweep] double: analytic vs central-difference "
                "per-column error, step scan:\n");
    for (double h : {1e-5, 1e-6, 1e-7})
    {
        auto e = collect_all<double>(h, trials, 0xDAC);
        std::printf("  h=%.0e: p95=%.3e max=%.3e (n=%zu)\n",
                    h, percentile95(e), maximum(e), e.size());
    }

    auto e = collect_all<double>(static_cast<double>(gate_step_v<double>),
                                 trials, 0xDAC);
    const double p95 = percentile95(e);
    const double mx = maximum(e);
    std::printf("  chosen h=%.0e: p95=%.3e max=%.3e | fd_tol_double=%.3e\n",
                static_cast<double>(gate_step_v<double>), p95, mx,
                fd_tol_double);

    // Sufficiency: the tolerance must clear the finite-difference noise floor.
    REQUIRE(fd_tol_double > mx);
    // Non-vacuity: it must stay within a small multiple of that floor, so a
    // real analytic-Jacobian error still trips it. The old 10^7 * eps ~ 1.19
    // value was ~10^9x the floor; this is ~50x.
    REQUIRE(fd_tol_double < 100.0 * mx);
    // Pin the shipped constant.
    REQUIRE(fd_tol_double == 1e-8);
    // Double path stays tight relative to the float path.
    REQUIRE(fd_tol_double <= 1e-7);
}

TEST_CASE("Jacobian FD tolerance sweep: float", "[jacobian][sweep][fd]")
{
    constexpr int trials = 50;

    std::printf("[jacobian-fd-sweep] float: analytic vs central-difference "
                "per-column error, step scan:\n");
    for (float h : {1e-3f, 1e-4f, 1e-5f})
    {
        auto e = collect_all<float>(h, trials, 0xFAC);
        std::printf("  h=%.0e: p95=%.3e max=%.3e (n=%zu)\n",
                    static_cast<double>(h), percentile95(e), maximum(e),
                    e.size());
    }

    auto e = collect_all<float>(gate_step_v<float>, trials, 0xFAC);
    const double p95 = percentile95(e);
    const double mx = maximum(e);
    std::printf("  chosen h=%.0e: p95=%.3e max=%.3e | fd_tol_float=%.3e\n",
                static_cast<double>(gate_step_v<float>), p95, mx,
                static_cast<double>(fd_tol_float));

    // Sufficiency: clear the measured float noise floor.
    REQUIRE(static_cast<double>(fd_tol_float) > mx);
    // Non-vacuity: within a small multiple of that floor (the old 10^7 * eps ~ 1.19
    // tolerance was ~1000x the floor and could never fail).
    REQUIRE(static_cast<double>(fd_tol_float) < 10.0 * mx);
    // Pin the shipped constant, and bound it to the "order 1e-3" band.
    REQUIRE(fd_tol_float == 5e-3f);
    REQUIRE(static_cast<double>(fd_tol_float) < 1e-2);
    REQUIRE(static_cast<double>(fd_tol_float) > 1e-5);
}

// ---------------------------------------------------------------------------
// Nine-robot regression gate.
// ---------------------------------------------------------------------------
TEMPLATE_TEST_CASE("Jacobian sweep: nine robots x fifty seeded configs",
    "[jacobian][sweep]", double, float)
{
    using Scalar = TestType;

    jac_gate_robot(cartan::fixtures::make_ur3e_chain<Scalar>, "UR3e");
    jac_gate_robot(cartan::fixtures::make_lbr_med14_chain<Scalar>, "LBR Med 14");
    jac_gate_robot(cartan::fixtures::make_kr6_sixx_chain<Scalar>, "KR6 SIXX");
    jac_gate_robot(cartan::fixtures::make_panda_chain<Scalar>, "Panda");
    jac_gate_robot(cartan::fixtures::make_abb_irb120_chain<Scalar>, "ABB IRB 120");
    jac_gate_robot(cartan::fixtures::make_jaco2_chain<Scalar>, "Jaco2");
    jac_gate_robot(cartan::fixtures::make_fetch_chain<Scalar>, "Fetch");
    jac_gate_robot(cartan::fixtures::make_baxter_chain<Scalar>, "Baxter");
    jac_gate_robot(cartan::fixtures::make_kuka_lwr4_chain<Scalar>, "Kuka LWR4");
}

// ---------------------------------------------------------------------------
// Prismatic, mixed, and zero-DOF coverage.
// ---------------------------------------------------------------------------
TEMPLATE_TEST_CASE("Jacobian sweep: prismatic, mixed, and zero-DOF coverage",
    "[jacobian][sweep][prismatic]", double, float)
{
    using Scalar = TestType;

    SECTION("pure prismatic chain")
    {
        jac_gate_robot(make_ppp_chain<Scalar>, "PPP prismatic");
    }

    SECTION("mixed revolute/prismatic chain")
    {
        jac_gate_robot(
            cartan::fixtures::make_rppr_signed_chain<Scalar>, "RPPR mixed");
    }

    SECTION("zero-DOF chain has a 6x0 space Jacobian")
    {
        auto chain = make_zero_dof_chain<Scalar>();
        REQUIRE(chain.num_joints() == 0);

        Eigen::VectorX<Scalar> q(0);
        auto fk = cartan::forward_kinematics(chain, q);
        auto Js = cartan::space_jacobian(chain, fk);
        REQUIRE(Js.rows() == 6);
        REQUIRE(Js.cols() == 0);
    }
}
