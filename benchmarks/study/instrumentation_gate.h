#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_INSTRUMENTATION_GATE_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_INSTRUMENTATION_GATE_H

/// @file instrumentation_gate.h
/// @brief The harness's checks on its own instrumentation.
///
/// A counting adaptor that changed the answer would still produce counts, and a
/// count of zero reads exactly like a solver that never evaluated anything. So
/// the counts are asserted strictly positive where the harness drives the
/// iteration, and the adaptor is proven to return bit-identical solutions to the
/// bare chain rather than assumed to.

#include "feasible_set.h"
#include "target_pool.h"
#include "participants.h"
#include "solve_outcome.h"
#include "participant_list.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace cartan::bench
{

namespace detail
{

template <int N>
bool identical(
    const typename cartan::joint_state<double, N>::position_type& left,
    const typename cartan::joint_state<double, N>::position_type& right,
    int& first_difference)
{
    for (int i = 0; i < N; ++i)
    {
        const bool same =
            left(i) == right(i) || (std::isnan(left(i)) && std::isnan(right(i)));
        if (!same)
        {
            first_difference = i;
            return false;
        }
    }
    return true;
}

template <int N>
bool counts_are_positive(
    const std::vector<participant_entry<N>>& resolved, const target_pool<N>& pool)
{
    bool held = true;
    for (const auto& entry : resolved)
    {
        if (!entry.kernel_countable)
        {
            std::printf("%s: kernel evaluations are not countable, and are never inferred\n",
                entry.name.c_str());
            continue;
        }
        kernel_counts total{0, 0};
        for (int i = 0; i < pool.size(); ++i)
        {
            const auto outcome = entry.solve(pool.target(i), pool.seed(i));
            total.fk += outcome.counts.fk;
            total.jac += outcome.counts.jac;
        }
        std::printf("%s: %lld forward-kinematics evaluations, %lld Jacobian evaluations\n",
            entry.name.c_str(), static_cast<long long>(total.fk),
            static_cast<long long>(total.jac));
        held = held && total.fk > 0 && total.jac > 0;
    }
    return held;
}

template <int N>
bool adaptor_preserves_solutions(
    const feasible_set<N>& feasible, const target_pool<N>& pool, const solve_budget& budget)
{
    cartan_lm_solver counted;
    for (int i = 0; i < pool.size(); ++i)
    {
        const auto through = counted(feasible, pool.target(i), pool.seed(i), budget);
        const auto bare = timed_cartan_lm<N>(feasible, pool.target(i), pool.seed(i), budget);
        int joint = 0;
        if (!identical<N>(through.q, bare.q, joint))
        {
            std::printf("counting adaptor: target %d joint %d differs -- %.17g through the "
                        "adaptor against %.17g through the bare chain\n",
                i, joint, through.q(joint), bare.q(joint));
            return false;
        }
    }
    std::printf("counting adaptor: %d solves bit-identical to the bare chain\n", pool.size());
    return true;
}

}

template <int N>
int run_instrumentation_gate(
    const feasible_set<N>& feasible,
    const std::vector<participant_entry<N>>& resolved,
    const target_pool<N>& pool,
    const solve_budget& budget)
{
    const bool positive = detail::counts_are_positive<N>(resolved, pool);
    return positive && detail::adaptor_preserves_solutions<N>(feasible, pool, budget) ? 0 : 1;
}

}

#endif
