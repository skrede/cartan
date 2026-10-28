#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_SOLVER_GROUP_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_SOLVER_GROUP_H

/// @file solver_group.h
/// @brief The participants resolved for one rung of the ladder.
///
/// A group is built per rung because the wall-clock-budgeted comparator takes
/// its cap at construction: a ladder that reused one solver object would ask
/// every rung the same question and report six answers to it.
///
/// Each participant carries its own budget, which is the same rung expressed on
/// that participant's axis and at that participant's tolerance. A participant
/// whose iteration the harness drives reads the work-unit figure and reports
/// achieved kernel evaluations; a participant that owns its loop reads the cap
/// and reports achieved wall time. Neither number is ever derived from the
/// other, and under the accuracy mode no two of the tolerances agree.

#include "budget.h"
#include "feasible_set.h"
#include "participants.h"
#include "participant_list.h"
#include "tolerance_policy.h"

#include <cartan/lie/se3.h>

#include <vector>

namespace cartan::bench
{

template <int N>
class solver_group
{
public:
    using position_type = typename target_entry<N>::position_type;

    solver_group(const feasible_set<N>& feasible, const budget& rung, stratum which,
        const tolerance_policy& tolerances)
        : m_driven(solve_budget_for(rung, which, tolerances.tolerance_for("cartan_lm"), true))
        , m_restarting(
              solve_budget_for(rung, which, tolerances.tolerance_for("cartan_restart_lm"), true))
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
        , m_peer_budget(
              solve_budget_for(rung, which, tolerances.tolerance_for("pinocchio_lm"), true))
        , m_peer_restarting(solve_budget_for(
              rung, which, tolerances.tolerance_for("pinocchio_restart_lm"), true))
#endif
#ifdef CARTAN_BENCH_STUDY_HAS_TRAC_IK
        , m_clocked(solve_budget_for(rung, which, tolerances.tolerance_for("trac_ik"), false))
#endif
        , m_cartan()
        , m_cartan_restart()
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
        , m_peer(feasible)
        , m_peer_restart(feasible)
#endif
#ifdef CARTAN_BENCH_STUDY_HAS_TRAC_IK
        , m_comparator(feasible, m_clocked.tolerance, m_clocked.time_cap_ms)
#endif
        , m_entries()
    {
        m_entries.push_back({"cartan_lm", true,
            [this, &feasible](const cartan::se3<double>& target, const position_type& seed)
            { return m_cartan(feasible, target, seed, m_driven); },
            m_driven, tolerances.claim_for("cartan_lm")});
        m_entries.push_back({"cartan_restart_lm", true,
            [this, &feasible](const cartan::se3<double>& target, const position_type& seed)
            { return m_cartan_restart(feasible, target, seed, m_restarting); },
            m_restarting, tolerances.claim_for("cartan_restart_lm")});
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
        m_entries.push_back({"pinocchio_lm", true,
            [this, &feasible](const cartan::se3<double>& target, const position_type& seed)
            { return m_peer(feasible, target, seed, m_peer_budget); },
            m_peer_budget, tolerances.claim_for("pinocchio_lm")});
        m_entries.push_back({"pinocchio_restart_lm", true,
            [this, &feasible](const cartan::se3<double>& target, const position_type& seed)
            { return m_peer_restart(feasible, target, seed, m_peer_restarting); },
            m_peer_restarting, tolerances.claim_for("pinocchio_restart_lm")});
#endif
#ifdef CARTAN_BENCH_STUDY_HAS_TRAC_IK
        m_entries.push_back({"trac_ik", false,
            [this, &feasible](const cartan::se3<double>& target, const position_type& seed)
            { return m_comparator(feasible, target, seed, m_clocked); },
            m_clocked, tolerances.claim_for("trac_ik")});
#endif
    }

    solver_group(const solver_group&) = delete;

    solver_group& operator=(const solver_group&) = delete;

    const std::vector<participant_entry<N>>& entries() const { return m_entries; }

    /// The instrumentation gate builds its own cartan solver and needs the
    /// budget that one runs at, which no entry of a partially resolved group is
    /// guaranteed to be.
    const solve_budget& cartan_budget() const { return m_driven; }

private:
    solve_budget m_driven;
    solve_budget m_restarting;
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
    solve_budget m_peer_budget;
    solve_budget m_peer_restarting;
#endif
#ifdef CARTAN_BENCH_STUDY_HAS_TRAC_IK
    solve_budget m_clocked;
#endif
    cartan_lm_solver m_cartan;
    cartan_restart_lm_solver m_cartan_restart;
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
    pinocchio_lm_solver<N> m_peer;
    pinocchio_restart_lm_solver<N> m_peer_restart;
#endif
#ifdef CARTAN_BENCH_STUDY_HAS_TRAC_IK
    trac_ik_solver<N> m_comparator;
#endif
    std::vector<participant_entry<N>> m_entries;
};

}

#endif
