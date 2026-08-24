/// @file ik_study_calibrate.cpp
/// @brief The offline pass that measures what each solver must be asked for.
///
/// A search loop inside the study's timed pass would time the search, and its
/// answer would exist only inside the run that used it. This program is the
/// search, run once over a fixed subset, writing a table the capture reads.
///

#include "study/record_writer.h"
#include "study/description_chain.h"
#include "study/calibration_search.h"

#include <cstdio>
#include <string>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <exception>

namespace
{

constexpr std::uint64_t pool_seed = cartan::bench::k_target_pool_seed;

/// The group built only to enumerate resolved participants solves nothing, so
/// what it is asked for cannot reach a measurement.
constexpr double announcement_tolerance = 1e-5;

template <int N>
cartan::bench::calibration_row calibrate_one(
    const cartan::bench::calibration_run<N>& run, std::size_t solver, const std::string& name,
    double target)
{
    const auto best = cartan::bench::search<N>(run, solver, target);
    return cartan::bench::calibration_row{run.options.table, run.options.robot,
        run.options.provenance, name, target, best.tolerance, best.seen.median, best.seen.p95,
        best.seen.n,
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
void calibrate_every_solver(const cartan::bench::calibration_run<N>& run, std::ofstream& out)
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
        options.provenance == "synthetic" ? cartan::bench::limits_provenance::synthetic
                                          : cartan::bench::limits_provenance::description);
    const cartan::bench::target_pool<N> pool(
        feasible, cartan::bench::stratum::reachable, options.targets, pool_seed);
    auto out = cartan::bench::detail::open_with_header(
        options.out, cartan::bench::calibration_header());
    const cartan::bench::calibration_run<N> run{
        feasible, pool, cartan::bench::budget_ladder().back(), options};
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
