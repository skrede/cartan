#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_CALIBRATION_SEARCH_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_CALIBRATION_SEARCH_H

/// @file calibration_search.h
/// @brief The bisection that finds what one solver must be asked for.
///
/// What it minimizes against is the harness's own recomputed error, taken
/// through the same adjudication every published figure goes through: a
/// solver's own account of its accuracy would calibrate it against its own
/// definition of error, which is the failure this mode exists to remove.

#include "budget.h"
#include "verdict.h"
#include "target_pool.h"
#include "iso_accuracy.h"
#include "solver_group.h"
#include "feasible_set.h"
#include "cell_aggregate.h"
#include "tolerance_policy.h"
#include "participant_list.h"
#include "calibration_options.h"

#include <cmath>
#include <vector>
#include <cstddef>

namespace cartan::bench
{

struct achieved_accuracy
{
    double median;
    double p95;
    int n;
};

struct calibration_step
{
    double tolerance;
    achieved_accuracy seen;
};

/// One robot, one table, one target subset and one rung of the ladder: what the
/// search varies is the requested tolerance and nothing else.
template <int N>
struct calibration_run
{
    const feasible_set<N>& feasible;
    const target_pool<N>& pool;
    const budget& rung;
    const calibration_options& options;
};

template <int N>
std::vector<double> accepted_errors(
    const calibration_run<N>& run, const participant_entry<N>& entry)
{
    std::vector<double> errors;
    for (int i = 0; i < run.pool.size(); ++i)
    {
        const auto outcome = entry.solve(run.pool.target(i), run.pool.seed(i));
        const auto seen = adjudicate<N>(run.feasible, outcome.q, run.pool.target(i),
            outcome.self_reported, entry.budget.tolerance);
        if (seen.accepted)
        {
            errors.push_back(seen.pos_err);
        }
    }
    return errors;
}

template <int N>
achieved_accuracy measure_at(const calibration_run<N>& run, std::size_t solver, double tolerance)
{
    const solver_group<N> group(
        run.feasible, run.rung, run.pool.which(), tolerance_policy(tolerance));
    const auto errors = accepted_errors<N>(run, group.entries()[solver]);
    return achieved_accuracy{detail::order_statistic(errors, 0.5),
        detail::order_statistic(errors, 0.95), static_cast<int>(errors.size())};
}

inline bool closer(
    const calibration_step& candidate, const calibration_step& best, double target)
{
    if (candidate.seen.n == 0)
    {
        return false;
    }
    return best.seen.n == 0
        || std::abs(candidate.seen.median - target) < std::abs(best.seen.median - target);
}

/// Only interior points of the interval are ever evaluated, so the tolerance a
/// non-converged row carries is a measured point rather than the endpoint the
/// search ran out of room at. A step at which no solve met its own requested
/// tolerance says the solver needs a looser one.
template <int N>
calibration_step search(const calibration_run<N>& run, std::size_t solver, double target)
{
    double lo = target / 100.0;
    double hi = target * 10.0;
    calibration_step best{std::sqrt(lo * hi), achieved_accuracy{0.0, 0.0, 0}};
    for (int step = 0; step < k_bisection_steps; ++step)
    {
        const double mid = std::sqrt(lo * hi);
        const calibration_step here{mid, measure_at<N>(run, solver, mid)};
        if (closer(here, best, target))
        {
            best = here;
        }
        if (here.seen.n > 0 && here.seen.median > target) { hi = mid; } else { lo = mid; }
    }
    return best;
}

}

#endif
