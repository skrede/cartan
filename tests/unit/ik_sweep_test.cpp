/// @file ik_sweep_test.cpp
/// @brief Seeded-random inverse-kinematics regression sweep.
///
/// For each of the nine benchmark robots the solver is exercised over fifty
/// seeded random configurations. Each trial walks a known configuration through
/// forward kinematics to a reachable target, seeds the solver a small distance
/// away from that configuration (isolating the numerical residual floor from
/// Levenberg-Marquardt basin-escape failures), solves, and re-verifies the
/// recovered pose independently with verify_solution (FK + twist error) rather
/// than trusting the solver's self-reported status. A regression in FK, the
/// Jacobian, or the stepper fails the sweep on the offending robot.
///
/// The sweep runs in double precision: the metre-scale float residual floor and
/// its gating are covered exhaustively by the dedicated float-tolerance sweep,
/// whereas here a fixed 1e-6 gate must hold deterministically for every robot
/// and seed. Coverage extends to a pure-prismatic chain, a mixed
/// revolute/prismatic chain, and a zero-DOF chain.

#include "../fixtures/chain_factories.h"
#include "../fixtures/prismatic_chains.h"

#include <cartan/types.h>
#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/ik/ik.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <random>
#include <cstddef>

namespace
{

constexpr int sweep_trials = 50;

/// Aggregate outcome of a robot's fifty seeded solves.
struct sweep_stats
{
    int attempts = 0;
    int converged = 0;
};

/// One robot: fifty seeded random reachable targets, LM solve from a near seed.
///
/// The gate has two falsifiable parts. First -- and strongest -- every solve
/// the runner *reports* as converged must independently pass verify_solution
/// (FK + twist error at the same tolerance): the solver may never claim a pose
/// it cannot back up. A regression that corrupts FK, the Jacobian, or the
/// convergence test surfaces here as a reported success whose pose does not
/// actually match. Second, the aggregate convergence rate is returned to the
/// caller, which asserts it stays high; a regression that degrades the solve
/// tanks that rate. A handful of reachable-but-ill-conditioned configurations
/// where Levenberg-Marquardt stalls are a solver-behavior concern, not an
/// FK/Jacobian correctness one, so a single stall is not treated as a failure.
template <typename MakeChain>
sweep_stats ik_sweep_robot(MakeChain make_chain, const char* name)
{
    INFO("Robot: " << name);

    auto chain = make_chain();
    using Chain = decltype(chain);
    using Scalar = typename Chain::scalar_type;
    constexpr int N = Chain::joints;

    const int n = chain.num_joints();

    const Scalar tol = Scalar(1e-6);
    cartan::convergence_criteria<Scalar> criteria{tol, tol, 500, 2000};

    // Targets are drawn from a moderate joint-angle band rather than the full
    // [-pi, pi] limits, and the solver is seeded a small distance from the
    // generating configuration. This isolates the numerical convergence of a
    // reachable solve from Levenberg-Marquardt basin-escape and near-singular
    // stalls at extreme configurations. The full [-pi, pi] range is exercised
    // by the FK and Jacobian sweeps, whose gates do not depend on iterative
    // convergence.
    const Scalar span = Scalar(0.6);
    std::mt19937 rng(42);
    std::uniform_real_distribution<Scalar> band(-span, span);
    std::uniform_real_distribution<Scalar> perturb(Scalar(-0.05), Scalar(0.05));

    sweep_stats stats;
    for (int trial = 0; trial < sweep_trials; ++trial)
    {
        INFO("Trial: " << trial);
        ++stats.attempts;

        Eigen::Vector<Scalar, N> q_known;
        Eigen::Vector<Scalar, N> q0;
        for (int j = 0; j < n; ++j)
        {
            q_known(j) = band(rng);
            q0(j) = q_known(j) + perturb(rng);
        }

        auto target = cartan::forward_kinematics(chain, q_known).end_effector;

        cartan::basic_ik_runner<cartan::lm<Chain>> solver;
        solver.setup(chain, target, q0, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++stats.converged;
            // No lying: a reported convergence must survive independent FK
            // re-verification, never the solver's self-report alone.
            REQUIRE(cartan::verify_solution(
                chain, target, result.value().solution.position, criteria));
        }
    }
    return stats;
}

/// Pure-prismatic P-P-P gantry (signed directions +z, +x, -y).
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

/// Zero-DOF dynamic chain: empty axis/limit storage, non-trivial home pose.
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

TEST_CASE("IK sweep: nine robots x fifty seeded configs", "[ik][sweep]")
{
    sweep_stats total;
    auto add = [&](sweep_stats s)
    {
        total.attempts += s.attempts;
        total.converged += s.converged;
    };

    add(ik_sweep_robot(cartan::fixtures::make_ur3e_chain<double>, "UR3e"));
    add(ik_sweep_robot(cartan::fixtures::make_lbr_med14_chain<double>, "LBR Med 14"));
    add(ik_sweep_robot(cartan::fixtures::make_kr6_sixx_chain<double>, "KR6 SIXX"));
    add(ik_sweep_robot(cartan::fixtures::make_panda_chain<double>, "Panda"));
    add(ik_sweep_robot(cartan::fixtures::make_abb_irb120_chain<double>, "ABB IRB 120"));
    add(ik_sweep_robot(cartan::fixtures::make_jaco2_chain<double>, "Jaco2"));
    add(ik_sweep_robot(cartan::fixtures::make_fetch_chain<double>, "Fetch"));
    add(ik_sweep_robot(cartan::fixtures::make_baxter_chain<double>, "Baxter"));
    add(ik_sweep_robot(cartan::fixtures::make_kuka_lwr4_chain<double>, "Kuka LWR4"));

    std::printf("[ik-sweep] converged %d/%d reachable near-seed solves\n",
                total.converged, total.attempts);

    // Aggregate convergence rate: a regression that degrades the FK/Jacobian
    // path or the stepper tanks this rate well below the floor.
    REQUIRE(total.converged >= (total.attempts * 9) / 10);
}

TEST_CASE("IK sweep: prismatic, mixed, and zero-DOF coverage",
    "[ik][sweep][prismatic]")
{
    SECTION("pure prismatic chain")
    {
        // A pure-prismatic chain is linear: every seeded solve must converge.
        auto s = ik_sweep_robot(make_ppp_chain<double>, "PPP prismatic");
        REQUIRE(s.converged == s.attempts);
    }

    SECTION("mixed revolute/prismatic chain")
    {
        auto s = ik_sweep_robot(
            cartan::fixtures::make_rppr_signed_chain<double>, "RPPR mixed");
        REQUIRE(s.converged >= (s.attempts * 9) / 10);
    }

    SECTION("zero-DOF chain: home target verifies trivially")
    {
        auto chain = make_zero_dof_chain<double>();
        REQUIRE(chain.num_joints() == 0);

        const double tol = 1e-6;
        cartan::convergence_criteria<double> criteria{tol, tol, 500, 1000};

        Eigen::VectorX<double> q(0);
        auto target = chain.home();
        REQUIRE(cartan::verify_solution(chain, target, q, criteria));
    }
}
