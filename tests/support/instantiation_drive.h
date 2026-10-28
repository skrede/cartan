#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_INSTANTIATION_DRIVE_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_INSTANTIATION_DRIVE_H

// The bounded drives the instantiation targets share. A combination is
// constructed, set up and stepped once, which is what instantiates the member
// bodies that including a class template leaves unchecked; where one entry into
// the loop is not enough, the second drive runs a runner until it terminates,
// and the third reaches the forms whose only entry point is a one-shot solve.
//
// The two-iteration, four-work-unit bound is what keeps any of the three from
// turning into a solve, and the assertions read a status rather than a solution
// because an under-actuated chain legitimately fails to reach a general pose.

#include "kinematics_helpers.h"

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/basic_ik_runner.h>

#include <cartan/serial/ik/detail/convergence.h>
#include <cartan/serial/ik/detail/setup_validation.h>

#include <cartan/serial/ik/solver/exhaustive_ik_runner.h>

#include <cartan/lie/se3.h>

#include <cartan/serial/chain/joint_state.h>

#include <catch2/catch_test_macros.hpp>

namespace cartan::testing
{

template <typename Scalar>
convergence_criteria<Scalar> bounded_criteria()
{
    convergence_criteria<Scalar> criteria;
    criteria.max_iterations_per_attempt = 2;
    criteria.max_total_work_units = 4;
    return criteria;
}

template <typename Chain>
using drive_configuration =
    typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type;

template <typename Chain>
drive_configuration<Chain> spread_configuration(
    const Chain& chain, typename Chain::scalar_type step)
{
    using scalar_type = typename Chain::scalar_type;

    drive_configuration<Chain> q = drive_configuration<Chain>::Zero(chain.num_joints());
    for (int i = 0; i < chain.num_joints(); ++i)
    {
        q(i) = step * scalar_type(i + 1);
    }
    return q;
}

template <typename Chain>
drive_configuration<Chain> seeded_configuration(const Chain& chain)
{
    using scalar_type = typename Chain::scalar_type;
    return spread_configuration(chain, scalar_type(0.1));
}

template <typename Chain>
se3<typename Chain::scalar_type> reachable_target(const Chain& chain)
{
    return fk_at(chain, seeded_configuration(chain)).end_effector;
}

/// A policy handed the pose it already stands at converges on entry and returns
/// before it runs its step body, so a drive that means to reach that body takes
/// its seed and its target at different configurations.
template <typename Chain>
se3<typename Chain::scalar_type> distant_target(const Chain& chain)
{
    using scalar_type = typename Chain::scalar_type;
    return fk_at(chain, spread_configuration(chain, scalar_type(0.3))).end_effector;
}

inline bool terminated(ik_status status)
{
    return status != ik_status::not_initialized && status != ik_status::running;
}

/// Only what solve_policy guarantees is read. A concrete solver also carries a
/// status() accessor, but the wrappers composed from one do not, so a drive that
/// reads it covers the policies and refuses the forms built on them.
///
/// A policy whose setup() refused latches the refusal and returns from step()
/// without entering its body; the three reads below all pass in that state, the
/// error norm starting at the scalar's maximum and the iteration count at zero.
/// Naming the statuses no fresh seed could repair is what makes a regression in
/// input validation red here rather than green at every caller of this drive.
template <typename Policy, typename Chain>
void drive_stepped(const Chain& chain, const se3<typename Chain::scalar_type>& target)
{
    using scalar_type = typename Chain::scalar_type;

    Policy policy;
    policy.setup(chain, target, seeded_configuration(chain), bounded_criteria<scalar_type>());
    step_result<scalar_type> stepped = policy.step(chain, 1);

    CHECK(stepped.status != ik_status::not_initialized);
    CHECK_FALSE(cartan::detail::is_precondition_failure(stepped.status));
    CHECK(policy.error_norm() >= scalar_type(0));
    CHECK(policy.iterations() >= 0);
}

/// The first two checks are what keep this from degenerating into the stepped
/// drive: the seed must fail the very convergence test the run will apply, and
/// the run must bill work, or the policy answered where it started and its
/// iteration body was never entered. The outcome is read against the terminal
/// status rather than for a value, because an under-actuated chain terminates
/// without one.
template <typename Runner, typename Chain>
void drive_solved(const Chain& chain, const se3<typename Chain::scalar_type>& target)
{
    using scalar_type = typename Chain::scalar_type;

    const convergence_criteria<scalar_type> criteria = bounded_criteria<scalar_type>();
    const drive_configuration<Chain> seed = seeded_configuration(chain);
    const vector6<scalar_type> entry_error =
        (fk_at(chain, seed).end_effector.inverse() * target).log();

    Runner runner;
    runner.setup(chain, target, seed, criteria);
    auto outcome = runner.solve();

    CHECK_FALSE(cartan::detail::is_converged_unweighted(entry_error, criteria));
    CHECK(runner.iterations() > 0);
    CHECK(terminated(runner.status()));
    CHECK(outcome.has_value() == (runner.status() == ik_status::converged));
}

template <typename Policy, typename Chain>
void drive_to_completion(const Chain& chain, const se3<typename Chain::scalar_type>& target)
{
    drive_solved<basic_ik_runner<Policy>>(chain, target);
}

/// The enumeration exposes neither an iteration count nor a status, so the stand-in
/// for the work guard above is its own accounting. Its only early return engages
/// the failure, so a disengaged failure is exactly the statement that every restart
/// set a policy up and entered its step loop. The restart count is read beside it to
/// pin the loop length independently of that reasoning; it does not discriminate on
/// its own, because a refusal on the last restart reports the full bound too.
template <typename Runner, typename Chain>
void drive_exhaustive(const Chain& chain, const se3<typename Chain::scalar_type>& target)
{
    using scalar_type = typename Chain::scalar_type;

    const convergence_criteria<scalar_type> criteria = bounded_criteria<scalar_type>();
    const exhaustive_options<scalar_type> options{.max_restarts = 3};

    Runner runner;
    const exhaustive_result<scalar_type, Chain::joints> enumerated =
        runner.solve(chain, target, seeded_configuration(chain), criteria, options);

    CHECK_FALSE(enumerated.failure.has_value());
    CHECK(enumerated.restarts_attempted == options.max_restarts);
}

}

#endif
