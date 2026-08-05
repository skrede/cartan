#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_CAPTURE_RUN_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_CAPTURE_RUN_H

/// @file capture_run.h
/// @brief One rung's solving, adjudication and recording.
///
/// The warm-start stratum is the one place the seed does not come from the
/// target's own record: each solve is started from the previously accepted
/// solution, and the row says which target that solution came from. A pass that
/// reordered the stratum, or seeded it fresh, would measure a different thing
/// under the same name.

#include "verdict.h"
#include "budget.h"
#include "manifest.h"
#include "target_pool.h"
#include "solver_group.h"
#include "record_writer.h"
#include "target_record.h"
#include "capture_options.h"

#include <chrono>
#include <string>
#include <cstdint>
#include <utility>

namespace cartan::bench
{

namespace detail
{

/// The achieved value on the row's own axis. A kernel count is what the harness
/// charged; a duration is what the clock read. Neither is ever computed from the
/// other, and a participant the harness does not drive has no kernel value at
/// all rather than a zero one.
inline std::int64_t achieved_on_axis(
    const kernel_counts& counts, std::int64_t wall_ns, bool countable)
{
    return countable ? counts.fk + counts.jac : wall_ns;
}

template <int N>
std::pair<target_record, typename target_entry<N>::position_type> solve_one(
    const feasible_set<N>& feasible,
    const capture_options& options,
    const participant_entry<N>& entry,
    const solve_budget& budget,
    const target_entry<N>& target,
    const typename target_entry<N>::position_type& seed,
    int seed_id)
{
    const auto started = std::chrono::steady_clock::now();
    const auto outcome = entry.solve(target.pose, seed);
    const auto elapsed = std::chrono::steady_clock::now() - started;
    const auto wall =
        std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
    const auto seen = cartan::bench::adjudicate<N>(
        feasible, outcome.q, target.pose, outcome.self_reported, budget.tolerance);
    return {target_record{options.table, options.robot, options.provenance, options.stratum,
        entry.name, budget.axis, budget.index, budget.requested,
        achieved_on_axis(outcome.counts, wall, entry.kernel_countable), target.target_id, seed_id,
        outcome.iterations, budget.tolerance, wall, outcome.counts, seen, entry.kernel_countable,
        stratum_from_name(options.stratum) != stratum::unreachable},
        outcome.q};
}

}

template <int N>
void capture_rung(
    const feasible_set<N>& feasible,
    const capture_options& options,
    const solver_group<N>& group,
    const target_pool<N>& pool,
    record_writer& writer)
{
    for (const auto& entry : group.entries())
    {
        const auto& budget = group.budget_for(entry.kernel_countable);
        auto warm = target_entry<N>::position_type::Zero().eval();
        int warm_from = -1;
        for (int i = 0; i < pool.size(); ++i)
        {
            const bool warmed = pool.ordered() && warm_from >= 0;
            const auto [row, solution] = detail::solve_one<N>(feasible, options, entry, budget,
                pool.entry(i), warmed ? warm : pool.seed(i), warmed ? warm_from : i);
            writer.write(row);
            if (pool.ordered() && row.adjudication.accepted)
            {
                warm = solution;
                warm_from = i;
            }
        }
    }
}

inline capture_parameters study_parameters(const capture_options& options)
{
    std::vector<named_count> targets;
    std::vector<named_count> caps;
    std::vector<int> rungs;
    for (int ordinal = 0; ordinal < k_stratum_count; ++ordinal)
    {
        const auto which = static_cast<stratum>(ordinal);
        targets.push_back({std::string{stratum_name(which)}, targets_for(which)});
        caps.push_back({std::string{stratum_name(which)}, comparator_cap_for(which)});
    }
    for (const auto& rung : budget_ladder())
    {
        rungs.push_back(rung.kernel_evaluations);
    }
    return capture_parameters{targets, caps, rungs, {}, k_repetitions, options.targets,
        options.command};
}

template <int N>
study_notes study_notes_for(
    const feasible_set<N>& feasible, stratum which, const std::vector<excluded_cell>& excluded)
{
    return study_notes{feasible.qualifying_joints().empty(), feasible.qualifying_joints(),
        excluded, std::string{stratum_notes(which).population}};
}

}

#endif
