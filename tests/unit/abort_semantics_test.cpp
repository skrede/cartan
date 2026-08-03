/// @file abort_semantics_test.cpp
/// @brief An abort is terminal for the solve it interrupts, is observed at the
///        step boundary, and is reported as one state by every policy shape.
///
/// Measured before the latch existed, on this fixture. The runner called every
/// policy's abort() and then assigned itself running, so status() reported
/// running in every case. Behind that, two things happened: a policy whose
/// abort() body was empty ignored the call outright and the whole solve
/// afterwards returned a solution, while a policy that did latch reported the
/// abort as a stall, so a caller could not tell a solve it had stopped from one
/// that stopped making progress.

#include "../support/kinematics_helpers.h"

#include "../fixtures/chain_factories.h"

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/solver/projected_lm.h>
#include <cartan/serial/ik/solver/newton_raphson.h>
#include <cartan/serial/ik/wrapper/restart_wrapper.h>

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/kinematic_chain.h>

#ifdef CARTAN_HAS_ARGMIN
#include <cartan/serial/ik/solver/argmin_lbfgsb.h>
#include <cartan/serial/ik/solver/argmin_projected_gn.h>
#endif

#include <catch2/catch_test_macros.hpp>

namespace spp = cartan;

namespace
{

using chain_t = spp::kinematic_chain<double, 6>;
using position_t = Eigen::Vector<double, 6>;

chain_t fixture_chain()
{
    return spp::fixtures::make_ur3e_chain<double>();
}

position_t seed()
{
    return position_t::Zero();
}

spp::convergence_criteria<double> criteria()
{
    return {1e-6, 1e-6, 200, 400};
}

spp::se3<double> reachable_target(const chain_t& chain)
{
    position_t q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    return spp::testing::fk_at(chain, q_known).end_effector;
}

/// A latch that reports the terminal state while still billing work is a half
/// latch, so both halves are asserted together for every policy shape.
template <typename Policy>
void step_after_abort_is_terminal_and_free(Policy& policy)
{
    auto chain = fixture_chain();
    policy.setup(chain, reachable_target(chain), seed(), criteria());
    policy.abort();

    auto stepped = policy.step(chain, 5);
    REQUIRE(stepped.status == spp::ik_status::aborted);
    REQUIRE(stepped.metrics.units_consumed == 0);
}

}

TEST_CASE("an aborted runner is terminal and its solve reports the abort", "[ik][abort]")
{
    auto chain = fixture_chain();
    spp::basic_ik_runner<spp::lm<chain_t>> runner;
    runner.setup(chain, reachable_target(chain), seed(), criteria());

    runner.abort();
    REQUIRE(runner.status() == spp::ik_status::aborted);

    auto result = runner.solve();
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error().reason == spp::ik_failure::aborted);
}

TEST_CASE("a fresh setup clears an aborted runner", "[ik][abort]")
{
    auto chain = fixture_chain();
    auto target = reachable_target(chain);
    spp::basic_ik_runner<spp::lm<chain_t>> runner;

    runner.setup(chain, target, seed(), criteria());
    runner.abort();
    REQUIRE(runner.status() == spp::ik_status::aborted);

    runner.setup(chain, target, seed(), criteria());
    REQUIRE(runner.status() == spp::ik_status::running);
    REQUIRE(runner.solve().has_value());
}

// Abort interrupts a solve that is running. It used to latch over any status but
// a refused setup, so a call after a solve had already converged replaced a
// successful outcome with a failure indistinguishable from a genuine mid-solve
// abort -- and there was no solve to interrupt.
TEST_CASE("abort does not overwrite a solve that already finished", "[ik][abort]")
{
    auto chain = fixture_chain();
    auto target = reachable_target(chain);
    spp::basic_ik_runner<spp::lm<chain_t>> runner;

    runner.setup(chain, target, seed(), criteria());
    REQUIRE(runner.solve().has_value());
    REQUIRE(runner.converged());

    runner.abort();
    CHECK(runner.status() == spp::ik_status::converged);
    CHECK(runner.solve().has_value());
}

TEST_CASE("an aborted Newton-Raphson policy is terminal at the step boundary", "[ik][abort]")
{
    spp::newton_raphson<chain_t> policy;
    step_after_abort_is_terminal_and_free(policy);
    REQUIRE(policy.status() == spp::ik_status::aborted);
}

TEST_CASE("an aborted restarting Levenberg-Marquardt policy is terminal", "[ik][abort]")
{
    spp::projected_lm<chain_t> policy;
    step_after_abort_is_terminal_and_free(policy);
    REQUIRE(policy.status() == spp::ik_status::aborted);
}

#ifdef CARTAN_HAS_ARGMIN

TEST_CASE("an aborted L-BFGS-B policy reports the abort and its finer reason", "[ik][abort]")
{
    spp::argmin_lbfgsb<chain_t> policy;
    step_after_abort_is_terminal_and_free(policy);
    REQUIRE(policy.status() == spp::ik_status::aborted);
    REQUIRE(policy.termination_reason() == spp::ik_termination_reason::solver_aborted);
}

TEST_CASE("an aborted projected Gauss-Newton policy reports the abort and its finer reason",
    "[ik][abort]")
{
    spp::argmin_projected_gn<chain_t> policy;
    step_after_abort_is_terminal_and_free(policy);
    REQUIRE(policy.status() == spp::ik_status::aborted);
    REQUIRE(policy.termination_reason() == spp::ik_termination_reason::solver_aborted);
}

#endif

TEST_CASE("aborting the restart wrapper propagates to the wrapped policy", "[ik][abort]")
{
    auto chain = fixture_chain();
    spp::restart_wrapper<chain_t, spp::projected_lm<chain_t>> wrapper;
    wrapper.setup(chain, reachable_target(chain), seed(), criteria());

    wrapper.abort();

    auto stepped = wrapper.step(chain, 5);
    REQUIRE(stepped.status == spp::ik_status::aborted);
    REQUIRE(stepped.metrics.units_consumed == 0);
}
