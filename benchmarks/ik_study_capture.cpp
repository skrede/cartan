/// @file ik_study_capture.cpp
/// @brief The untimed instrumented pass: solve, adjudicate, record.
///
/// A timing harness aggregates over its iteration loop by construction, and a
/// counter or a file write inside a measured block would be timed along with
/// the solve, so the study's per-target evidence comes from here rather than
/// from the timed pass. The wall time on each row is this pass's own, taken
/// with the counting adaptor in the path: a diagnostic, not a timing figure.

#include "study/verdict.h"
#include "study/target_pool.h"
#include "study/feasible_set.h"
#include "study/participants.h"
#include "study/record_writer.h"
#include "study/target_record.h"
#include "study/capture_options.h"
#include "study/participant_list.h"

#include <cartan/lie/se3.h>

#include <cmath>
#include <chrono>
#include <cstdio>
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

bool identical(const position_type& left, const position_type& right, int& first_difference)
{
    for (int i = 0; i < study_joints; ++i)
    {
        const bool same = left(i) == right(i)
            || (std::isnan(left(i)) && std::isnan(right(i)));
        if (!same)
        {
            first_difference = i;
            return false;
        }
    }
    return true;
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
        outcome.counts, seen};
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
}

bool counts_are_positive(
    const std::vector<cartan::bench::participant_entry>& resolved,
    const cartan::bench::target_pool& pool)
{
    bool held = true;
    for (const auto& entry : resolved)
    {
        cartan::bench::kernel_counts total{0, 0};
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

bool adaptor_preserves_solutions(
    const feasible_type& feasible,
    const cartan::bench::target_pool& pool,
    const cartan::bench::solve_budget& budget)
{
    cartan::bench::cartan_lm_solver counted;
    for (int i = 0; i < pool.size(); ++i)
    {
        const auto through = counted(feasible, pool.target(i), pool.seed(i), budget);
        const auto bare = cartan::bench::timed_cartan_lm<study_joints>(
            feasible, pool.target(i), pool.seed(i), budget);
        int joint = 0;
        if (!identical(through.q, bare.q, joint))
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

int gate(
    const feasible_type& feasible,
    const std::vector<cartan::bench::participant_entry>& resolved,
    const cartan::bench::target_pool& pool,
    const cartan::bench::solve_budget& budget)
{
    const bool positive = counts_are_positive(resolved, pool);
    return positive && adaptor_preserves_solutions(feasible, pool, budget) ? 0 : 1;
}

int run(const cartan::bench::capture_options& options)
{
    const feasible_type feasible = cartan::bench::load_feasible_set<study_joints>(
        cartan::bench::description_for(options.robot),
        cartan::bench::rule_for_table(options.table));
    const cartan::bench::solve_budget budget{0, budget_units, budget_tolerance, "work_units"};

    cartan::bench::cartan_lm_solver cartan_solver;
    std::vector<cartan::bench::participant_entry> resolved;
    resolved.push_back({"cartan_lm",
        [&](const cartan::se3<double>& target, const position_type& seed)
        { return cartan_solver(feasible, target, seed, budget); }});
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
    cartan::bench::pinocchio_lm_solver<study_joints> peer(feasible);
    resolved.push_back({"pinocchio_lm",
        [&](const cartan::se3<double>& target, const position_type& seed)
        { return peer(feasible, target, seed, budget); }});
#endif

    cartan::bench::print_participants(resolved);
    cartan::bench::refuse_incomplete_participants(resolved);

    const cartan::bench::target_pool pool(feasible, options.stratum, options.targets, pool_seed);
    if (options.gate)
    {
        return gate(feasible, resolved, pool, budget);
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
