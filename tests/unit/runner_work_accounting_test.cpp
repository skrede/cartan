/// @file runner_work_accounting_test.cpp
/// @brief The runner's three stepping entry points share one charging path, so
///        they agree on the total work budget, on the terminal status they
///        latch, and on the metrics they report.
///
/// Measured before the shared path existed, on this fixture with a five-unit
/// budget: step() driven five thousand times reported iterations() == 0 and
/// left status() == running; step_n(5000) reported the same; solve() reported
/// 5 iterations and also returned with status() == running, so its caller
/// received an error whose reason came from the reason switch's fall-through
/// arm rather than from a latched terminal status.

#include "../support/kinematics_helpers.h"

#include "../fixtures/chain_factories.h"

#include <cartan/serial/ik/solvers.h>
#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/basic_ik_runner.h>

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>

namespace spp = cartan;

namespace
{

using chain_t = spp::kinematic_chain<double, 6>;
using runner_t = spp::basic_ik_runner<spp::dls<chain_t>>;
using position_t = Eigen::Vector<double, 6>;

constexpr int budget = 5;
constexpr int drive_count = 5000;

chain_t fixture_chain()
{
    return spp::fixtures::make_ur3e_chain<double>();
}

position_t seed()
{
    return position_t::Zero();
}

/// A pose the solver needs far more than `budget` work units to reach, so the
/// budget rather than convergence is what ends the solve.
spp::se3<double> distant_target(const chain_t& chain)
{
    position_t q_known;
    q_known << 2.5, -1.8, 1.2, -2.0, 1.5, -1.0;
    return spp::testing::fk_at(chain, q_known).end_effector;
}

spp::convergence_criteria<double> criteria_with(int total_units)
{
    return {.position_tol = 1e-6,
            .orientation_tol = 1e-6,
            .max_iterations_per_attempt = 200,
            .max_total_work_units = total_units};
}

/// The non-speed objective is what keeps the runner alive past a convergence:
/// it re-seeds the inner policy at the converged iterate and keeps going, which
/// is the path on which the unaccounted work accumulated.
spp::solver_options<double> continuing_objective()
{
    return {.objective = spp::ik_objective::min_error_norm};
}

bool terminal(spp::ik_status status)
{
    return status != spp::ik_status::running
        && status != spp::ik_status::not_initialized;
}

}

TEST_CASE("step() charges its work and stops at the total budget",
    "[ik][runner][work-accounting]")
{
    auto chain = fixture_chain();
    runner_t runner;
    runner.setup(chain, distant_target(chain), seed(), criteria_with(budget),
        continuing_objective());

    for (int i = 0; i < drive_count && runner.step() == spp::ik_status::running; ++i) {}

    CHECK(runner.iterations() == budget);
    CHECK(terminal(runner.status()));

    // A terminal runner charges nothing further.
    runner.step();
    CHECK(runner.iterations() == budget);
}

TEST_CASE("step_n() charges its work and stops at the total budget",
    "[ik][runner][work-accounting]")
{
    auto chain = fixture_chain();
    runner_t runner;
    runner.setup(chain, distant_target(chain), seed(), criteria_with(budget),
        continuing_objective());

    runner.step_n(drive_count);

    CHECK(runner.iterations() == budget);
    CHECK(terminal(runner.status()));
}

TEST_CASE("step(), step_n() and solve() agree from one setup",
    "[ik][runner][work-accounting]")
{
    auto chain = fixture_chain();
    auto target = distant_target(chain);

    runner_t stepped;
    stepped.setup(chain, target, seed(), criteria_with(budget), continuing_objective());
    for (int i = 0; i < drive_count && stepped.step() == spp::ik_status::running; ++i) {}

    runner_t batched;
    batched.setup(chain, target, seed(), criteria_with(budget), continuing_objective());
    batched.step_n(drive_count);

    runner_t solved;
    solved.setup(chain, target, seed(), criteria_with(budget), continuing_objective());
    (void)solved.solve();

    CHECK(stepped.iterations() == batched.iterations());
    CHECK(batched.iterations() == solved.iterations());
    CHECK(stepped.status() == batched.status());
    CHECK(batched.status() == solved.status());
    CHECK(stepped.error_norm() == batched.error_norm());
    CHECK(batched.error_norm() == solved.error_norm());
}

TEST_CASE("solve() assigns a terminal status before it returns",
    "[ik][runner][work-accounting]")
{
    auto chain = fixture_chain();
    runner_t runner;
    runner.setup(chain, distant_target(chain), seed(), criteria_with(budget),
        continuing_objective());

    auto result = runner.solve();

    CHECK(terminal(runner.status()));
    CHECK_FALSE(result.has_value());
    CHECK(runner.status() == spp::ik_status::iteration_limit);
}

TEST_CASE("a budget spent after a convergence reports converged",
    "[ik][runner][work-accounting]")
{
    auto chain = fixture_chain();
    position_t q_near;
    q_near << 0.2, -0.3, 0.4, -0.2, 0.3, 0.1;
    auto target = spp::testing::fk_at(chain, q_near).end_effector;

    runner_t runner;
    runner.setup(chain, target, seed(), criteria_with(200), continuing_objective());

    for (int i = 0; i < drive_count && runner.step() == spp::ik_status::running; ++i) {}

    CHECK(runner.status() == spp::ik_status::converged);
    CHECK(runner.iterations() <= 200);
}

TEST_CASE("a zero total budget terminates before any work is charged",
    "[ik][runner][work-accounting]")
{
    auto chain = fixture_chain();
    runner_t runner;
    runner.setup(chain, distant_target(chain), seed(), criteria_with(0),
        continuing_objective());

    CHECK(terminal(runner.step()));
    CHECK(runner.iterations() == 0);
}

TEST_CASE("a racing solve charges its policies' work against the same budget",
    "[ik][runner][work-accounting]")
{
    auto chain = fixture_chain();
    spp::dual_ik_runner<chain_t> runner;
    runner.setup(chain, distant_target(chain), seed(), criteria_with(budget));

    (void)runner.solve();

    CHECK(terminal(runner.status()));
    // A round-robin tick is atomic, so the last one may carry the accumulator
    // past the budget by at most one unit per still-active policy.
    CHECK(runner.iterations() >= budget);
    CHECK(runner.iterations() <= budget + 1);
}
