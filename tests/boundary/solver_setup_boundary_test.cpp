#include "boundary_fixtures.h"

#include <cartan/analytical/detail/fk_verification.h>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>

namespace spp = cartan;

using spp::fixtures::make_dynamic_chain;
using spp::fixtures::make_six_joint_dynamic_chain;
using spp::fixtures::joint_vector;

template <typename Scalar>
using dyn_chain = spp::kinematic_chain<Scalar, spp::dynamic>;

template <typename Scalar>
static spp::se3<Scalar> reachable_target(const dyn_chain<Scalar>& chain)
{
    return spp::forward_kinematics(chain, joint_vector<Scalar>(chain.num_joints(), Scalar(0.2)))
        ->end_effector;
}

template <typename Scalar>
static spp::se3<Scalar> poisoned_target(const dyn_chain<Scalar>& chain, Scalar poison)
{
    auto target = reachable_target<Scalar>(chain);
    spp::vector3<Scalar> translation = target.translation();
    translation(1) = poison;
    return spp::se3<Scalar>(target.rotation(), translation);
}

/// Every solver in the corpus is driven through the same three calls, so a
/// structural difference between them shows up as a differing outcome rather
/// than as a case somebody forgot to write.
template <typename Solver, typename Scalar>
static spp::step_result<Scalar> setup_then_step(
    Solver& solver,
    const dyn_chain<Scalar>& chain,
    const spp::se3<Scalar>& target,
    const Eigen::VectorX<Scalar>& seed)
{
    solver.setup(chain, target, seed, spp::convergence_criteria<Scalar>{});
    return solver.step(chain, 4);
}

template <typename Solver, typename Scalar>
static void expect_rejected_seed_does_no_work(spp::ik_status expected)
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = reachable_target<Scalar>(chain);
    auto seed = (expected == spp::ik_status::dimension_mismatch)
        ? joint_vector<Scalar>(chain.num_joints() - 1, Scalar(0.1))
        : joint_vector<Scalar>(chain.num_joints(), std::numeric_limits<Scalar>::quiet_NaN());

    Solver solver;
    auto stepped = setup_then_step(solver, chain, target, seed);

    REQUIRE(stepped.status == expected);
    REQUIRE(stepped.metrics.units_consumed == 0);
    REQUIRE(solver.iterations() == 0);
}

template <typename Solver, typename Scalar>
static void expect_rejected_target_does_no_work(Scalar poison)
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = poisoned_target<Scalar>(chain, poison);
    auto seed = joint_vector<Scalar>(chain.num_joints(), Scalar(0.1));

    Solver solver;
    auto stepped = setup_then_step(solver, chain, target, seed);

    REQUIRE(stepped.status == spp::ik_status::non_finite_input);
    REQUIRE(stepped.metrics.units_consumed == 0);
}

template <typename Solver, typename Scalar>
static void expect_never_setup_does_no_work()
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();

    Solver solver;
    auto stepped = solver.step(chain, 4);

    REQUIRE(stepped.status == spp::ik_status::not_initialized);
    REQUIRE(stepped.metrics.units_consumed == 0);
}

template <typename Solver, typename Scalar>
static void expect_valid_setup_unaffected()
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = reachable_target<Scalar>(chain);
    auto seed = joint_vector<Scalar>(chain.num_joints(), Scalar(0.15));

    Solver solver;
    auto stepped = setup_then_step(solver, chain, target, seed);

    REQUIRE(stepped.status != spp::ik_status::dimension_mismatch);
    REQUIRE(stepped.status != spp::ik_status::non_finite_input);
    REQUIRE(stepped.status != spp::ik_status::not_initialized);
    REQUIRE(stepped.metrics.units_consumed > 0);
}

// Three structurally different solvers: a trust-region one whose loop guards on
// the running status, the projected trust-region one whose loop guards on "not
// converged" and therefore needed an explicit return, and a damped
// least-squares one with a different work-unit accounting.
template <typename Scalar>
using lm_solver = spp::builtin_lm<dyn_chain<Scalar>>;
template <typename Scalar>
using projected_solver = spp::projected_lm<dyn_chain<Scalar>>;
template <typename Scalar>
using dls_solver = spp::dls<dyn_chain<Scalar>>;

TEMPLATE_TEST_CASE("a rejected seed leaves a solver terminal and consumes no work",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    for (auto expected : {spp::ik_status::dimension_mismatch, spp::ik_status::non_finite_input})
    {
        expect_rejected_seed_does_no_work<lm_solver<Scalar>, Scalar>(expected);
        expect_rejected_seed_does_no_work<projected_solver<Scalar>, Scalar>(expected);
        expect_rejected_seed_does_no_work<dls_solver<Scalar>, Scalar>(expected);
    }
}

TEMPLATE_TEST_CASE("a nonfinite target leaves a solver terminal and consumes no work",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    for (Scalar poison : {std::numeric_limits<Scalar>::quiet_NaN(),
             std::numeric_limits<Scalar>::infinity(),
             -std::numeric_limits<Scalar>::infinity()})
    {
        expect_rejected_target_does_no_work<lm_solver<Scalar>, Scalar>(poison);
        expect_rejected_target_does_no_work<projected_solver<Scalar>, Scalar>(poison);
        expect_rejected_target_does_no_work<dls_solver<Scalar>, Scalar>(poison);
    }
}

TEMPLATE_TEST_CASE("a solver stepped without setup performs no iteration",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    expect_never_setup_does_no_work<lm_solver<Scalar>, Scalar>();
    expect_never_setup_does_no_work<projected_solver<Scalar>, Scalar>();
    expect_never_setup_does_no_work<dls_solver<Scalar>, Scalar>();
}

TEMPLATE_TEST_CASE("a well-formed setup still runs", "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    expect_valid_setup_unaffected<lm_solver<Scalar>, Scalar>();
    expect_valid_setup_unaffected<projected_solver<Scalar>, Scalar>();
    expect_valid_setup_unaffected<dls_solver<Scalar>, Scalar>();
}

TEMPLATE_TEST_CASE("the runner reports a failed setup as a typed reason",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = reachable_target<Scalar>(chain);
    const int n = chain.num_joints();

    for (int size : {0, n - 1, n + 1})
    {
        spp::basic_ik_runner<lm_solver<Scalar>> runner;
        runner.setup(chain, target, joint_vector<Scalar>(size, Scalar(0.1)),
            spp::convergence_criteria<Scalar>{});
        auto result = runner.solve();
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error().reason == spp::ik_failure::dimension_mismatch);
        REQUIRE(runner.iterations() == 0);
    }

    for (Scalar poison : {std::numeric_limits<Scalar>::quiet_NaN(),
             std::numeric_limits<Scalar>::infinity(),
             -std::numeric_limits<Scalar>::infinity()})
    {
        for (int i = 0; i < n; ++i)
        {
            auto seed = joint_vector<Scalar>(n, Scalar(0.1));
            seed(i) = poison;
            spp::basic_ik_runner<lm_solver<Scalar>> runner;
            runner.setup(chain, target, seed, spp::convergence_criteria<Scalar>{});
            auto result = runner.solve();
            REQUIRE_FALSE(result.has_value());
            REQUIRE(result.error().reason == spp::ik_failure::non_finite_input);
        }

        spp::basic_ik_runner<lm_solver<Scalar>> runner;
        runner.setup(chain, poisoned_target<Scalar>(chain, poison),
            joint_vector<Scalar>(n, Scalar(0.1)), spp::convergence_criteria<Scalar>{});
        auto result = runner.solve();
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error().reason == spp::ik_failure::non_finite_input);
    }
}

TEMPLATE_TEST_CASE("a runner stepped without setup reports not_initialized",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    spp::basic_ik_runner<lm_solver<Scalar>> runner;
    REQUIRE(runner.status() == spp::ik_status::not_initialized);
    auto result = runner.solve();
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error().reason == spp::ik_failure::not_initialized);
}

TEMPLATE_TEST_CASE("a restart-wrapped solver returns the failure and does not restart",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    using wrapper_type = spp::restart_wrapper<dyn_chain<Scalar>, projected_solver<Scalar>>;

    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = reachable_target<Scalar>(chain);
    const int n = chain.num_joints();

    {
        wrapper_type wrapper;
        auto stepped = setup_then_step(
            wrapper, chain, target, joint_vector<Scalar>(n - 1, Scalar(0.1)));
        REQUIRE(stepped.status == spp::ik_status::dimension_mismatch);
        REQUIRE(stepped.metrics.units_consumed == 0);
        REQUIRE(wrapper.restarts() == 0);
    }

    {
        wrapper_type wrapper;
        auto stepped = setup_then_step(wrapper, chain,
            poisoned_target<Scalar>(chain, std::numeric_limits<Scalar>::quiet_NaN()),
            joint_vector<Scalar>(n, Scalar(0.1)));
        REQUIRE(stepped.status == spp::ik_status::non_finite_input);
        REQUIRE(wrapper.restarts() == 0);
    }

    {
        wrapper_type wrapper;
        REQUIRE(wrapper.step(chain, 4).status == spp::ik_status::not_initialized);
        REQUIRE(wrapper.restarts() == 0);
    }
}

TEMPLATE_TEST_CASE("a restart-wrapped solver is reusable after a rejected setup",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    using wrapper_type = spp::restart_wrapper<dyn_chain<Scalar>, projected_solver<Scalar>>;

    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = reachable_target<Scalar>(chain);
    const int n = chain.num_joints();

    wrapper_type wrapper;
    auto rejected = setup_then_step(
        wrapper, chain, target, joint_vector<Scalar>(n - 1, Scalar(0.1)));
    REQUIRE(rejected.status == spp::ik_status::dimension_mismatch);

    auto accepted = setup_then_step(
        wrapper, chain, target, joint_vector<Scalar>(n, Scalar(0.15)));
    REQUIRE(accepted.status != spp::ik_status::dimension_mismatch);
    REQUIRE(accepted.status != spp::ik_status::non_finite_input);
    REQUIRE(accepted.status != spp::ik_status::not_initialized);
    REQUIRE(accepted.metrics.units_consumed > 0);
}

// The corpus's only reuse case drove the projected solver *through* the restart
// wrapper, which returns before the inner setup() ever sees the bad seed. The
// bare solver is exercised here so the wrapper cannot mask an inner latch that
// no later setup clears.
template <typename Solver, typename Scalar>
static void expect_reusable_after_rejected_setup()
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = reachable_target<Scalar>(chain);
    const int n = chain.num_joints();

    Solver solver;
    auto rejected = setup_then_step(
        solver, chain, target, joint_vector<Scalar>(n - 1, Scalar(0.1)));
    REQUIRE(rejected.status == spp::ik_status::dimension_mismatch);

    auto accepted = setup_then_step(
        solver, chain, target, joint_vector<Scalar>(n, Scalar(0.15)));
    REQUIRE(accepted.status != spp::ik_status::dimension_mismatch);
    REQUIRE(accepted.status != spp::ik_status::non_finite_input);
    REQUIRE(accepted.status != spp::ik_status::not_initialized);
    REQUIRE(accepted.metrics.units_consumed > 0);
}

TEMPLATE_TEST_CASE("a solver used directly is reusable after a rejected setup",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    expect_reusable_after_rejected_setup<lm_solver<Scalar>, Scalar>();
    expect_reusable_after_rejected_setup<projected_solver<Scalar>, Scalar>();
    expect_reusable_after_rejected_setup<dls_solver<Scalar>, Scalar>();
}

// setup() validates the chain it is handed, but every policy takes the chain as
// a parameter of step() as well, so nothing but this check binds the two.
template <typename Solver, typename Scalar>
static void expect_foreign_chain_refused()
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto wider = make_dynamic_chain<Scalar>(12);
    auto target = reachable_target<Scalar>(chain);

    Solver solver;
    solver.setup(chain, target, joint_vector<Scalar>(chain.num_joints(), Scalar(0.15)),
        spp::convergence_criteria<Scalar>{});

    auto stepped = solver.step(wider, 4);
    REQUIRE(stepped.status == spp::ik_status::dimension_mismatch);
    REQUIRE(stepped.metrics.units_consumed == 0);
}

TEMPLATE_TEST_CASE("a solver refuses a step against a chain setup never saw",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    expect_foreign_chain_refused<lm_solver<Scalar>, Scalar>();
    expect_foreign_chain_refused<projected_solver<Scalar>, Scalar>();
    expect_foreign_chain_refused<dls_solver<Scalar>, Scalar>();
    expect_foreign_chain_refused<
        spp::restart_wrapper<dyn_chain<Scalar>, projected_solver<Scalar>>, Scalar>();
}

TEMPLATE_TEST_CASE("aborting a runner does not clear a refused setup",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = reachable_target<Scalar>(chain);
    const int n = chain.num_joints();

    spp::basic_ik_runner<lm_solver<Scalar>> runner;
    runner.setup(chain, target, joint_vector<Scalar>(n, Scalar(0.15)),
        spp::convergence_criteria<Scalar>{});
    REQUIRE(runner.solve().has_value());

    runner.setup(chain, target, joint_vector<Scalar>(n - 1, Scalar(0.1)),
        spp::convergence_criteria<Scalar>{});
    REQUIRE(runner.status() == spp::ik_status::dimension_mismatch);

    runner.abort();
    REQUIRE(runner.status() == spp::ik_status::dimension_mismatch);

    auto result = runner.solve();
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error().reason == spp::ik_failure::dimension_mismatch);
}

TEMPLATE_TEST_CASE("a refused setup leaves nothing of the previous solve readable",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    using wrapper_type = spp::restart_wrapper<dyn_chain<Scalar>, projected_solver<Scalar>>;

    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = reachable_target<Scalar>(chain);
    const int n = chain.num_joints();
    const spp::convergence_criteria<Scalar> criteria{};

    spp::basic_ik_runner<lm_solver<Scalar>> runner;
    runner.setup(chain, target, joint_vector<Scalar>(n, Scalar(0.15)), criteria);
    REQUIRE(runner.solve().has_value());

    runner.setup(chain, target, joint_vector<Scalar>(n - 1, Scalar(0.1)), criteria);
    REQUIRE_FALSE(runner.converged());
    REQUIRE(runner.iterations() == 0);
    auto result = runner.solve();
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error().last_error_norm == std::numeric_limits<Scalar>::max());
    REQUIRE(result.error().last_q.size() == n);
    REQUIRE(result.error().last_q.isZero());

    wrapper_type wrapper;
    wrapper.setup(chain, target, joint_vector<Scalar>(n, Scalar(0.15)), criteria);
    while (wrapper.step(chain, 8).status == spp::ik_status::running) {}
    REQUIRE(wrapper.converged());

    wrapper.setup(chain, target, joint_vector<Scalar>(n - 1, Scalar(0.1)), criteria);
    REQUIRE_FALSE(wrapper.converged());
    REQUIRE(wrapper.error_norm() == std::numeric_limits<Scalar>::max());
    REQUIRE(wrapper.iterations() == 0);
    REQUIRE(wrapper.solution().isZero());
}

TEMPLATE_TEST_CASE("the exhaustive runner evaluates one seed and returns the failure",
    "[ik][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = reachable_target<Scalar>(chain);
    const int n = chain.num_joints();

    spp::exhaustive_options<Scalar> options;
    options.max_restarts = 32;

    {
        spp::exhaustive_ik_runner<dyn_chain<Scalar>, projected_solver<Scalar>> runner;
        auto result = runner.solve(chain, target, joint_vector<Scalar>(n - 1, Scalar(0.1)),
            spp::convergence_criteria<Scalar>{}, options);
        REQUIRE(result.failure.has_value());
        REQUIRE(*result.failure == spp::ik_failure::dimension_mismatch);
        REQUIRE(result.restarts_attempted == 1);
        REQUIRE(result.solutions.empty());
    }

    {
        spp::exhaustive_ik_runner<dyn_chain<Scalar>, projected_solver<Scalar>> runner;
        auto result = runner.solve(chain,
            poisoned_target<Scalar>(chain, std::numeric_limits<Scalar>::infinity()),
            joint_vector<Scalar>(n, Scalar(0.1)), spp::convergence_criteria<Scalar>{}, options);
        REQUIRE(result.failure.has_value());
        REQUIRE(*result.failure == spp::ik_failure::non_finite_input);
        REQUIRE(result.restarts_attempted == 1);
    }
}

// The folded planar two-joint configuration is where a closed-form derivation's
// acos leaves its domain and hands the verifier a NaN candidate.
TEMPLATE_TEST_CASE("the analytical verifier rejects a nonfinite candidate",
    "[analytical][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_dynamic_chain<Scalar>(2);
    auto target = reachable_target<Scalar>(chain);

    for (Scalar poison : {std::numeric_limits<Scalar>::quiet_NaN(),
             std::numeric_limits<Scalar>::infinity(),
             -std::numeric_limits<Scalar>::infinity()})
    {
        for (int i = 0; i < 2; ++i)
        {
            Eigen::Vector<Scalar, 2> q;
            q << Scalar(0.2), Scalar(0.2);
            q(i) = poison;
            // Both flag settings: the defect was live only with the orientation
            // check off, so a run only with it on would pass for the wrong
            // reason.
            for (bool check_orientation : {false, true})
            {
                REQUIRE_FALSE(spp::detail::verify_analytical_solution<dyn_chain<Scalar>, 2>(
                    chain, q, target, check_orientation,
                    spp::default_verification_tolerance_v<Scalar>));
            }
        }
    }

    Eigen::Vector<Scalar, 2> finite;
    finite << Scalar(0.2), Scalar(0.2);
    REQUIRE(spp::detail::verify_analytical_solution<dyn_chain<Scalar>, 2>(
        chain, finite, target, false, spp::default_verification_tolerance_v<Scalar>));
    REQUIRE(spp::detail::verify_analytical_solution<dyn_chain<Scalar>, 2>(
        chain, finite, target, true, spp::default_verification_tolerance_v<Scalar>));
}

TEMPLATE_TEST_CASE("the analytical verifier refuses a candidate of the wrong length",
    "[analytical][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_dynamic_chain<Scalar>(3);
    auto target = reachable_target<Scalar>(chain);

    Eigen::Vector<Scalar, 2> too_short;
    too_short << Scalar(0.2), Scalar(0.2);
    REQUIRE_FALSE(spp::detail::verify_analytical_solution<dyn_chain<Scalar>, 2>(
        chain, too_short, target, false,
        spp::default_verification_tolerance_v<Scalar>));
}
