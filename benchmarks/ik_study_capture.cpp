/// @file ik_study_capture.cpp
/// @brief The untimed instrumented pass: solve, adjudicate, record.
///
/// A timing harness aggregates over its iteration loop by construction, and a
/// counter or a file write inside a measured block would be timed along with
/// the solve, so the study's per-target evidence comes from here rather than
/// from the timed pass. The wall time on each row is this pass's own, taken
/// with the counting adaptor in the path: a diagnostic, not a timing figure.

#include "study/budget.h"
#include "study/manifest.h"
#include "study/exclusions.h"
#include "study/target_pool.h"
#include "study/capture_run.h"
#include "study/feasible_set.h"
#include "study/fk_agreement.h"
#include "study/solver_group.h"
#include "study/record_writer.h"
#include "study/target_fixture.h"
#include "study/capture_options.h"
#include "study/participant_list.h"
#include "study/strata_selfcheck.h"
#include "study/instrumentation_gate.h"

#include <cstdio>
#include <vector>
#include <cstdint>
#include <exception>

namespace
{

constexpr std::uint64_t pool_seed = 42;
constexpr double budget_tolerance = 1e-5;

std::vector<cartan::bench::budget> rungs_for(const cartan::bench::capture_options& options)
{
    const auto& ladder = cartan::bench::budget_ladder();
    if (options.all_budgets)
    {
        return {ladder.begin(), ladder.end()};
    }
    return {ladder[static_cast<std::size_t>(options.budget_index)]};
}

/// Measured for the robot this capture ran on, not asserted: the deviation
/// between the description-loaded chain and the hand-coded chain it replaced is
/// the evidence that the robot measured is the robot named.
template <int N>
cartan::bench::description_deviation loaded_chain_deviation(
    const cartan::bench::feasible_set<N>& feasible, const cartan::bench::description_spec& spec)
{
    const auto& truth = cartan::bench::truth_for(spec.robot_key);
    const auto worst = cartan::bench::worst_fk_deviation<N>(
        feasible.declared(), truth.chain, truth.seed);
    return cartan::bench::description_deviation{std::string{spec.robot_key}, worst.first};
}

template <int N>
void capture_every_rung(
    const cartan::bench::feasible_set<N>& feasible,
    const cartan::bench::capture_options& options,
    const cartan::bench::target_pool<N>& pool)
{
    cartan::bench::write_target_fixture<N>(
        options.sidecar_dir, options.table, options.robot, pool);
    cartan::bench::record_writer writer(options.out_dir, options.sidecar_dir, options.table);
    for (const auto& rung : rungs_for(options))
    {
        const cartan::bench::solver_group<N> group(
            feasible, rung, pool.which(), budget_tolerance);
        cartan::bench::capture_rung<N>(feasible, options, group, pool, writer);
    }
    writer.finish();
}

template <int N>
int run_for(
    const cartan::bench::capture_options& options, const cartan::bench::description_spec& spec)
{
    const auto rule = cartan::bench::rule_for_table(options.table);
    const auto provenance = options.provenance == "synthetic"
        ? cartan::bench::limits_provenance::synthetic
        : cartan::bench::limits_provenance::description;
    const auto feasible = cartan::bench::load_feasible_set<N>(spec, rule, provenance);
    const auto which = cartan::bench::stratum_from_name(options.stratum);
    if (options.selfcheck)
    {
        return cartan::bench::run_strata_selfcheck<N>(feasible, options.targets, pool_seed);
    }

    std::vector<cartan::bench::excluded_cell> excluded;
    if (!cartan::bench::cell_admissible(provenance, which, rule))
    {
        excluded.push_back(cartan::bench::exclusion_for(options.robot, which, rule));
        std::printf("no rows emitted for the %s stratum on table %s: %s\n",
            options.stratum.c_str(), options.table.c_str(), excluded.front().reason.c_str());
    }

    const cartan::bench::solver_group<N> announced(
        feasible, rungs_for(options).front(), which, budget_tolerance);
    cartan::bench::print_participants<N>(announced.entries());
    const auto absent = cartan::bench::absent_participants<N>(announced.entries());
    cartan::bench::report_absent_participants(absent);
    cartan::bench::refuse_empty_participants<N>(announced.entries());

    if (excluded.empty())
    {
        const cartan::bench::target_pool<N> pool(feasible, which, options.targets, pool_seed);
        if (options.gate)
        {
            return cartan::bench::run_instrumentation_gate<N>(
                feasible, announced.entries(), pool, announced.budget_for(true));
        }
        capture_every_rung<N>(feasible, options, pool);
    }
    cartan::bench::write_manifest(options.out_dir, cartan::bench::study_parameters(options),
        absent, loaded_chain_deviation<N>(feasible, spec),
        cartan::bench::study_notes_for<N>(feasible, which, excluded));
    return 0;
}

int run(const cartan::bench::capture_options& options)
{
    const auto& spec = cartan::bench::description_for(options.robot);
    if (spec.joints == 6)
    {
        return run_for<6>(options, spec);
    }
    if (spec.joints == 7)
    {
        return run_for<7>(options, spec);
    }
    throw std::runtime_error(std::string{spec.robot_key} + ": the study measures six- and "
        "seven-axis arms, and this description declares "
        + std::to_string(spec.joints) + " joints");
}

}

int main(int argc, char** argv)
{
    try
    {
        return run(cartan::bench::parse_capture_options(argc, argv));
    }
    catch (const std::exception& refused)
    {
        std::fprintf(stderr, "ik_study_capture: %s\n", refused.what());
        return 1;
    }
}
