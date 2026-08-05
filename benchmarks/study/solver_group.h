#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_SOLVER_GROUP_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_SOLVER_GROUP_H

/// @file solver_group.h
/// @brief The participants resolved for one rung of the ladder.
///
/// A group is built per rung because the wall-clock-budgeted comparator takes
/// its cap at construction: a ladder that reused one solver object would ask
/// every rung the same question and report six answers to it.
///
/// The two budgets are the same rung expressed on the two axes. A participant
/// whose iteration the harness drives reads the work-unit one and reports
/// achieved kernel evaluations; a participant that owns its loop reads the cap
/// and reports achieved wall time. Neither number is ever derived from the
/// other.

#include "budget.h"
#include "feasible_set.h"
#include "participants.h"
#include "participant_list.h"

#include <cartan/lie/se3.h>

#include <vector>

namespace cartan::bench
{

template <int N>
class solver_group
{
public:
    using position_type = typename target_entry<N>::position_type;

    solver_group(
        const feasible_set<N>& feasible, const budget& rung, stratum which, double tolerance)
        : m_driven(solve_budget_for(rung, which, tolerance, true))
        , m_clocked(solve_budget_for(rung, which, tolerance, false))
        , m_cartan()
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
        , m_peer(feasible)
#endif
#ifdef CARTAN_BENCH_STUDY_HAS_TRAC_IK
        , m_comparator(feasible, tolerance, m_clocked.time_cap_ms)
#endif
        , m_entries()
    {
        m_entries.push_back({"cartan_lm", true,
            [this, &feasible](const cartan::se3<double>& target, const position_type& seed)
            { return m_cartan(feasible, target, seed, m_driven); }});
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
        m_entries.push_back({"pinocchio_lm", true,
            [this, &feasible](const cartan::se3<double>& target, const position_type& seed)
            { return m_peer(feasible, target, seed, m_driven); }});
#endif
#ifdef CARTAN_BENCH_STUDY_HAS_TRAC_IK
        m_entries.push_back({"trac_ik", false,
            [this, &feasible](const cartan::se3<double>& target, const position_type& seed)
            { return m_comparator(feasible, target, seed, m_clocked); }});
#endif
    }

    solver_group(const solver_group&) = delete;

    solver_group& operator=(const solver_group&) = delete;

    const std::vector<participant_entry<N>>& entries() const { return m_entries; }

    const solve_budget& budget_for(bool countable) const
    {
        return countable ? m_driven : m_clocked;
    }

private:
    solve_budget m_driven;
    solve_budget m_clocked;
    cartan_lm_solver m_cartan;
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
    pinocchio_lm_solver<N> m_peer;
#endif
#ifdef CARTAN_BENCH_STUDY_HAS_TRAC_IK
    trac_ik_solver<N> m_comparator;
#endif
    std::vector<participant_entry<N>> m_entries;
};

}

#endif
