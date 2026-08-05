#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_PARTICIPANTS_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_PARTICIPANTS_H

/// @file participants.h
/// @brief The one call signature every solver adapter satisfies.
///
/// An adapter is handed the feasible set and nothing else that describes
/// bounds. The audit's defect was a default template argument selecting a
/// limits policy, which left every cartan cell solving an unconstrained problem
/// while the comparator cells were handed explicit arrays; with no such
/// parameter anywhere in the signature there is nowhere to write it.

#include "verdict.h"
#include "feasible_set.h"
#include "counting_chain.h"
#include "solve_outcome.h"
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
#include "pinocchio_participant.h"
#endif

#include <cartan/lie/se3.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/ik/basic_ik_runner.h>

#include <concepts>

namespace cartan::bench
{

template <typename P, int N>
concept participant = requires(
    P& solver,
    const feasible_set<N>& feasible,
    const cartan::se3<double>& target,
    const typename cartan::joint_state<double, N>::position_type& seed,
    const solve_budget& budget)
{
    { solver(feasible, target, seed, budget) } -> std::same_as<solve_outcome<N>>;
};

namespace detail
{

template <int N, typename Chain>
solve_outcome<N> run_cartan_lm(
    const Chain& chain,
    const cartan::se3<double>& target,
    const typename cartan::joint_state<double, N>::position_type& seed,
    const solve_budget& budget,
    const kernel_counts& counts)
{
    const cartan::convergence_criteria<double> criteria{
        .position_tol = budget.tolerance,
        .orientation_tol = budget.tolerance,
        .max_iterations_per_attempt = budget.units,
        .max_total_work_units = budget.units};

    cartan::basic_ik_runner<cartan::lm<Chain>> solver;
    solver.setup(chain, target, seed, criteria);
    const auto result = solver.solve();
    if (result.has_value())
    {
        return solve_outcome<N>{result->iterations, true, counts, result->solution.position};
    }
    return solve_outcome<N>{solver.iterations(), false, counts, result.error().last_q};
}

}

struct cartan_lm_solver
{
    template <int N>
    solve_outcome<N> operator()(
        const feasible_set<N>& feasible,
        const cartan::se3<double>& target,
        const typename cartan::joint_state<double, N>::position_type& seed,
        const solve_budget& budget)
    {
        kernel_counts counts{0, 0};
        const counting_chain<typename feasible_set<N>::chain_type> counted(
            feasible.chain(), counts);
        return detail::run_cartan_lm<N>(counted, target, seed, budget, counts);
    }
};

/// The timed pass calls this instead: same solve, same criteria, against the
/// bare chain, so nothing the capture pass adds is inside a measured block.
template <int N>
solve_outcome<N> timed_cartan_lm(
    const feasible_set<N>& feasible,
    const cartan::se3<double>& target,
    const typename cartan::joint_state<double, N>::position_type& seed,
    const solve_budget& budget)
{
    const kernel_counts counts{0, 0};
    return detail::run_cartan_lm<N>(feasible.chain(), target, seed, budget, counts);
}

static_assert(participant<cartan_lm_solver, 6>,
    "the cartan adapter must satisfy the one call signature every participant has");

#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
static_assert(participant<pinocchio_lm_solver<6>, 6>,
    "the peer adapter must satisfy the one call signature every participant has");
#endif

}

#endif
