/// @file ik_study_capture.cpp
/// @brief The untimed instrumented pass: solve, adjudicate, record.
///
/// A timing harness aggregates over its iteration loop by construction, and a
/// counter or a file write inside a measured block would be timed along with
/// the solve, so the study's per-target evidence comes from here rather than
/// from the timed pass. The wall time on each row is this pass's own, taken
/// with the counting adaptor in the path: a diagnostic, not a timing figure.

#include "study/verdict.h"
#include "study/manifest.h"
#include "study/instrumentation_gate.h"
#include "study/target_pool.h"
#include "study/feasible_set.h"
#include "study/fk_agreement.h"
#include "study/participants.h"
#include "study/record_writer.h"
#include "study/target_record.h"
#include "study/capture_options.h"
#include "study/participant_list.h"

#include <cartan/lie/se3.h>

#include <cmath>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>
#include <exception>
#include <string_view>

namespace
{

using cartan::bench::study_joints;
using feasible_type = cartan::bench::feasible_set<study_joints>;
using position_type = typename cartan::bench::target_pool::position_type;

constexpr unsigned int pool_seed = 42;
constexpr int budget_units = 800;
constexpr double budget_tolerance = 1e-5;

std::string_view provenance_name(cartan::bench::limits_provenance provenance)
{
    return provenance == cartan::bench::limits_provenance::description ? "description"
                                                                      : "synthetic";
}

cartan::bench::target_record solve_one(
    const feasible_type& feasible,
    const cartan::bench::capture_options& options,
    const cartan::bench::participant_entry& entry,
    const cartan::bench::target_pool& pool,
    const cartan::bench::solve_budget& budget,
    int index)
{
    const auto started = std::chrono::steady_clock::now();
    const auto outcome = entry.solve(pool.target(index), pool.seed(index));
    const auto elapsed = std::chrono::steady_clock::now() - started;
    const auto seen = cartan::bench::adjudicate<study_joints>(
        feasible, outcome.q, pool.target(index), outcome.self_reported, budget.tolerance);
    return cartan::bench::target_record{
        options.table, options.robot, provenance_name(feasible.provenance()), options.stratum,
        entry.name, budget.axis, budget.index, budget.units, index, index, outcome.iterations,
        budget.tolerance,
        std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count(),
        outcome.counts, seen, entry.kernel_countable};
}

cartan::bench::capture_parameters study_parameters(
    const cartan::bench::capture_options& options, const cartan::bench::solve_budget& budget)
{
    return cartan::bench::capture_parameters{
        options.targets, 1, {budget.units}, {budget.tolerance},
        cartan::bench::comparator_time_cap_ms, options.command};
}

/// Measured for the robot this capture ran on, not asserted: the deviation
/// between the description-loaded chain and the hand-coded chain it replaced is
/// the evidence that the robot measured is the robot named.
cartan::bench::description_deviation loaded_chain_deviation(
    const feasible_type& feasible, const cartan::bench::description_spec& spec)
{
    const auto& truth = cartan::bench::truth_for(spec.robot_key);
    const auto worst =
        cartan::bench::worst_fk_deviation<study_joints>(feasible.chain(), truth.chain, truth.seed);
    return cartan::bench::description_deviation{std::string{spec.robot_key}, worst.first};
}

void capture(
    const feasible_type& feasible,
    const cartan::bench::capture_options& options,
    const std::vector<cartan::bench::participant_entry>& resolved,
    const cartan::bench::target_pool& pool,
    const cartan::bench::solve_budget& budget)
{
    cartan::bench::record_writer writer(options.out_dir, options.sidecar_dir, options.table);
    for (const auto& entry : resolved)
    {
        for (int i = 0; i < pool.size(); ++i)
        {
            writer.write(solve_one(feasible, options, entry, pool, budget, i));
        }
    }
    writer.finish();
    cartan::bench::write_manifest(options.out_dir, study_parameters(options, budget),
        cartan::bench::absent_participants(resolved),
        loaded_chain_deviation(feasible, cartan::bench::description_for(options.robot)));
}

int run(const cartan::bench::capture_options& options)
{
    const feasible_type feasible = cartan::bench::load_feasible_set<study_joints>(
        cartan::bench::description_for(options.robot),
        cartan::bench::rule_for_table(options.table));
    const cartan::bench::solve_budget budget{0, budget_units, budget_tolerance, "work_units"};

    cartan::bench::cartan_lm_solver cartan_solver;
    std::vector<cartan::bench::participant_entry> resolved;
    resolved.push_back({"cartan_lm", true,
        [&](const cartan::se3<double>& target, const position_type& seed)
        { return cartan_solver(feasible, target, seed, budget); }});
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
    cartan::bench::pinocchio_lm_solver<study_joints> peer(feasible);
    resolved.push_back({"pinocchio_lm", true,
        [&](const cartan::se3<double>& target, const position_type& seed)
        { return peer(feasible, target, seed, budget); }});
#endif
#ifdef CARTAN_BENCH_STUDY_HAS_TRAC_IK
    cartan::bench::trac_ik_solver<study_joints> comparator(feasible, budget.tolerance);
    resolved.push_back({"trac_ik", false,
        [&](const cartan::se3<double>& target, const position_type& seed)
        { return comparator(feasible, target, seed, budget); }});
#endif

    cartan::bench::print_participants(resolved);
    cartan::bench::report_absent_participants(cartan::bench::absent_participants(resolved));
    cartan::bench::refuse_empty_participants(resolved);

    const cartan::bench::target_pool pool(feasible, options.stratum, options.targets, pool_seed);
    if (options.gate)
    {
        return cartan::bench::run_instrumentation_gate<study_joints>(
            feasible, resolved, pool, budget);
    }
    capture(feasible, options, resolved, pool, budget);
    return 0;
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
