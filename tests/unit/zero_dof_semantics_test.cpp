#include "../support/joint_limits_helpers.h"

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/basic_ik_runner.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>

#include <cartan/serial/chain/kinematic_chain.h>

#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/fk/singularity_analysis.h>

#include <catch2/catch_test_macros.hpp>

namespace spp = cartan;

using chain_dyn = spp::kinematic_chain<double, spp::dynamic>;
using runner = spp::basic_ik_runner<spp::lm<chain_dyn>>;

static chain_dyn jointless()
{
    return chain_dyn(spp::se3<double>::identity(), {}, {});
}

static spp::se3<double> displaced(double x)
{
    spp::vector3<double> t;
    t << x, 0, 0;
    return spp::se3<double>(spp::so3<double>::identity(), t);
}

static spp::expected<spp::ik_result<double, spp::dynamic>, spp::ik_error<double, spp::dynamic>>
solve_jointless(const chain_dyn& chain, const spp::se3<double>& target, spp::ik_objective objective)
{
    runner solver;
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 100, 50};
    spp::solver_options<double> options{.objective = objective};
    solver.setup(chain, target, Eigen::VectorXd::Zero(0), criteria, options);
    return solver.solve();
}

// ============================================================================
// The one-point workspace
// ============================================================================

TEST_CASE("a chain with no joints converges on its home target", "[ik][zero_dof]")
{
    auto chain = jointless();
    auto home = spp::forward_kinematics(chain, Eigen::VectorXd::Zero(0));
    REQUIRE(home.has_value());

    for (auto objective : {spp::ik_objective::speed, spp::ik_objective::min_error_norm})
    {
        auto result = solve_jointless(chain, home->end_effector, objective);
        REQUIRE(result.has_value());
        CHECK(result->final_error_norm == 0.0);
        CHECK(result->solution.position.size() == 0);
    }
}

// The target is 0.5 m from the only pose the chain can hold, and no joint can
// close any part of that gap. Reported as `diverged` before this was defined,
// which claims a search went wrong rather than that no solution exists.
TEST_CASE("a chain with no joints reports a non-home target as unreachable", "[ik][zero_dof]")
{
    auto chain = jointless();

    for (auto objective : {spp::ik_objective::speed, spp::ik_objective::min_error_norm})
    {
        auto result = solve_jointless(chain, displaced(0.5), objective);
        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().reason == spp::ik_failure::unreachable);
        CHECK_FALSE(result.error().reason == spp::ik_failure::diverged);
    }
}

TEST_CASE("a chain with no joints latches unreachable at setup", "[ik][zero_dof]")
{
    auto chain = jointless();

    runner solver;
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 100, 50};
    solver.setup(chain, displaced(0.5), Eigen::VectorXd::Zero(0), criteria);

    CHECK(solver.status() == spp::ik_status::unreachable);
    CHECK(solver.iterations() == 0);
    CHECK_FALSE(solver.converged());
}

// Certified infeasibility is a statement about the workspace, so it survives a
// budget large enough that an iteration limit could not be the explanation.
TEST_CASE("unreachability on a chain with no joints is not a budget outcome", "[ik][zero_dof]")
{
    auto chain = jointless();

    runner solver;
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 100, 100000};
    solver.setup(chain, displaced(0.5), Eigen::VectorXd::Zero(0), criteria);

    auto result = solver.solve();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == spp::ik_failure::unreachable);
    CHECK(solver.iterations() == 0);
}

// The refusal and the empty-decomposition guard both landed with the selection
// objectives; this pins that the unreachable path did not displace either.
TEST_CASE("a chain with no joints is still refused under a Jacobian objective", "[ik][zero_dof]")
{
    auto chain = jointless();
    auto home = spp::forward_kinematics(chain, Eigen::VectorXd::Zero(0));

    for (auto objective : {spp::ik_objective::max_manipulability, spp::ik_objective::max_isotropy})
    {
        auto result = solve_jointless(chain, home->end_effector, objective);
        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().reason == spp::ik_failure::unsupported_configuration);
    }

    auto sigma = spp::singular_values(chain, Eigen::VectorXd::Zero(0));
    CHECK(sigma.size() == 0);
    CHECK_FALSE(spp::manipulability(sigma));
    CHECK_FALSE(spp::isotropy(sigma));
    CHECK_FALSE(spp::condition_number(sigma));
}
