#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_BUDGET_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_BUDGET_H

/// @file budget.h
/// @brief The ladder success is reported against, and the sweep sizes it costs.
///
/// One timeout produces one number, and a solver that fails at that timeout is
/// indistinguishable from a solver that would never have succeeded. The
/// previously published comparison read the first as the second. A ladder is
/// what tells them apart, so success is reported as a curve over the budget
/// rather than as a figure at one point on it.
///
/// The kernel rungs and the time-cap rungs are geometric in different ratios on
/// purpose. A kernel count and a duration are not commensurable -- one is work
/// the harness charges, the other is a wall clock a contended machine moves --
/// and a single ratio spanning both would assert a correspondence between them
/// that this study exists to refuse.
///
/// The sizes below are the lever the whole cost model turns on. The
/// wall-clock-budgeted comparator, at the default cap, on a stratum where it
/// fails, at the reachable stratum's target count, spends a hundred seconds per
/// cell per repetition; the difference between these values and uniform ones is
/// the difference between a study that runs in an hour and one that runs in a
/// working day.

#include "target_strata.h"
#include "solve_outcome.h"

#include <array>
#include <cstdint>
#include <algorithm>

namespace cartan::bench
{

constexpr int k_targets_reachable = 2000;
constexpr int k_targets_secondary = 500;
constexpr int k_repetitions = 3;
constexpr int k_budget_points = 6;
constexpr int k_comparator_cap_default_ms = 50;
constexpr int k_comparator_cap_failure_strata_ms = 10;
constexpr int k_calibration_targets = 200;

/// One rung, expressed once per participant kind. A solver whose iteration the
/// harness drives reads the work-unit and iteration fields; the one that owns
/// its loop reads the cap. The kernel field is the rung's target, never a
/// measurement -- what a run reports on that axis is the count it achieved.
struct budget
{
    int index;
    int kernel_evaluations;
    int work_units;
    int iterations;
    double time_cap_ms;
};

inline const std::array<budget, k_budget_points>& budget_ladder()
{
    constexpr double cap = static_cast<double>(k_comparator_cap_default_ms);
    static const std::array<budget, k_budget_points> ladder{
        budget{0, 9, 3, 3, cap / 32.0},
        budget{1, 27, 9, 9, cap / 16.0},
        budget{2, 81, 27, 27, cap / 8.0},
        budget{3, 243, 81, 81, cap / 4.0},
        budget{4, 729, 243, 243, cap / 2.0},
        budget{5, 2187, 729, 729, cap}};
    return ladder;
}

inline int targets_for(stratum which)
{
    return which == stratum::reachable ? k_targets_reachable : k_targets_secondary;
}

/// The two strata a solver is expected to fail on carry a tighter cap, because
/// a wall-clock-budgeted participant spends its whole cap on every target it
/// cannot solve and those two strata are where it cannot solve most of them.
inline int comparator_cap_for(stratum which)
{
    const bool failure_stratum =
        which == stratum::unreachable || which == stratum::limit_adjacent;
    return failure_stratum ? k_comparator_cap_failure_strata_ms : k_comparator_cap_default_ms;
}

/// The upper rungs of the time ladder coincide on a failure stratum, since the
/// stratum's own cap is below them. Coinciding rungs are recorded rather than
/// dropped: a reader comparing two rungs has to be able to see that they asked
/// the same question.
inline double capped_time(const budget& rung, stratum which)
{
    return std::min(rung.time_cap_ms, static_cast<double>(comparator_cap_for(which)));
}

inline solve_budget solve_budget_for(
    const budget& rung, stratum which, double tolerance, bool countable)
{
    const double capped = capped_time(rung, which);
    if (countable)
    {
        return solve_budget{rung.index, rung.work_units, tolerance, "kernel_evaluations",
            capped, rung.kernel_evaluations};
    }
    return solve_budget{rung.index, rung.work_units, tolerance, "wall_clock", capped,
        static_cast<std::int64_t>(capped * 1e6)};
}

}

#endif
