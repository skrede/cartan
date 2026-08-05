/// @file ik_study_calibrate.cpp
/// @brief The offline pass that measures what each solver must be asked for.
///
/// A search loop inside the study's timed pass would time the search, and its
/// answer would exist only inside the run that used it. This program is the
/// search, run once over a fixed subset, writing a table the capture reads.
///
/// What it minimizes against is the harness's own recomputed error, taken
/// through the same adjudication every published figure goes through: a solver's
/// own account of its accuracy would calibrate it against its own definition of
/// error, which is the failure this mode exists to remove.

#include "study/budget.h"
#include "study/verdict.h"
#include "study/target_pool.h"
#include "study/iso_accuracy.h"
#include "study/solver_group.h"
#include "study/cell_aggregate.h"
#include "study/record_writer.h"
#include "study/description_chain.h"
#include "study/participant_list.h"
#include "study/calibration_options.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <exception>

namespace
{

constexpr std::uint64_t pool_seed = 42;

/// The group built only to enumerate resolved participants solves nothing, so
/// what it is asked for cannot reach a measurement.
constexpr double announcement_tolerance = 1e-5;

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
    const cartan::bench::feasible_set<N>& feasible;
    const cartan::bench::target_pool<N>& pool;
    const cartan::bench::budget& rung;
    const cartan::bench::calibration_options& options;
};

template <int N>
std::vector<double> accepted_errors(
    const calibration_run<N>& run, const cartan::bench::participant_entry<N>& entry)
{
    std::vector<double> errors;
    for (int i = 0; i < run.pool.size(); ++i)
    {
        const auto outcome = entry.solve(run.pool.target(i), run.pool.seed(i));
        const auto seen = cartan::bench::adjudicate<N>(run.feasible, outcome.q,
            run.pool.target(i), outcome.self_reported, entry.budget.tolerance);
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
    const cartan::bench::solver_group<N> group(run.feasible, run.rung, run.pool.which(),
        cartan::bench::tolerance_policy(tolerance));
    const auto errors = accepted_errors<N>(run, group.entries()[solver]);
    return achieved_accuracy{cartan::bench::detail::order_statistic(errors, 0.5),
        cartan::bench::detail::order_statistic(errors, 0.95), static_cast<int>(errors.size())};
}

bool closer(const calibration_step& candidate, const calibration_step& best, double target)
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
    for (int step = 0; step < cartan::bench::k_bisection_steps; ++step)
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

template <int N>
cartan::bench::calibration_row calibrate_one(
    const calibration_run<N>& run, std::size_t solver, const std::string& name, double target)
{
    const auto best = search<N>(run, solver, target);
    return cartan::bench::calibration_row{run.options.table, run.options.robot, name, target,
        best.tolerance, best.seen.median, best.seen.p95, best.seen.n,
        cartan::bench::target_reached(best.seen.median, best.seen.n, target, run.pool.size())};
}

void report(const cartan::bench::calibration_row& row)
{
    std::printf("%s at %g m: tolerance %.6g, achieved %s over %d accepted solves, converged %d\n",
        row.solver.c_str(), row.accuracy_target, row.calibrated_tolerance,
        cartan::bench::achieved_field(row.achieved_median, row.n).c_str(), row.n,
        row.converged ? 1 : 0);
}

template <int N>
void calibrate_every_solver(const calibration_run<N>& run, std::ofstream& out)
{
    const cartan::bench::solver_group<N> group(
        run.feasible, run.rung, run.pool.which(),
        cartan::bench::tolerance_policy(announcement_tolerance));
    cartan::bench::print_participants<N>(group.entries());
    for (const double target : run.options.accuracy_targets())
    {
        for (std::size_t i = 0; i < group.entries().size(); ++i)
        {
            const auto row = calibrate_one<N>(run, i, group.entries()[i].name, target);
            out << cartan::bench::calibration_csv_row(row) << '\n';
            report(row);
        }
    }
}

template <int N>
int run_for(
    const cartan::bench::calibration_options& options,
    const cartan::bench::description_spec& spec)
{
    const auto feasible = cartan::bench::load_feasible_set<N>(spec,
        cartan::bench::rule_for_table(options.table),
        cartan::bench::limits_provenance::description);
    const cartan::bench::target_pool<N> pool(
        feasible, cartan::bench::stratum::reachable, options.targets, pool_seed);
    auto out = cartan::bench::detail::open_with_header(
        options.out, cartan::bench::calibration_header());
    const calibration_run<N> run{feasible, pool, cartan::bench::budget_ladder().back(), options};
    calibrate_every_solver<N>(run, out);
    return 0;
}

int run(const cartan::bench::calibration_options& options)
{
    const auto& spec = cartan::bench::description_for(options.robot);
    return cartan::bench::dispatch_on_joints(spec, [&spec, &options](auto joints)
        { return run_for<decltype(joints)::value>(options, spec); });
}

}

int main(int argc, char** argv)
{
    try
    {
        return run(cartan::bench::parse_calibration_options(argc, argv));
    }
    catch (const std::exception& refused)
    {
        std::fprintf(stderr, "ik_study_calibrate: %s\n", refused.what());
        return 1;
    }
}
