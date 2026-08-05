/// @file ik_study_benchmarks.cpp
/// @brief The study's timed pass: registration, and nothing else.
///
/// Inside a measured block there is the solve alone. Every counter, every
/// adjudication and every file write belongs to the untimed capture pass, which
/// is the whole reason the study runs two passes over the same target pool.
/// The configure-time gate over this directory catches a checked kinematics
/// entry point in a measured block; it cannot catch a counter, so that half of
/// the discipline has to live in the shape of this file.

#include "study/target_pool.h"
#include "study/feasible_set.h"
#include "study/participants.h"

#include <cartan/lie/se3.h>

#include <benchmark/benchmark.h>

namespace
{

using cartan::bench::study_joints;
using feasible_type = cartan::bench::feasible_set<study_joints>;

constexpr unsigned int pool_seed = 42;
constexpr int pool_targets = 200;
constexpr int budget_units = 800;
constexpr double budget_tolerance = 1e-5;

const feasible_type& study_feasible_set()
{
    static const feasible_type feasible = cartan::bench::load_feasible_set<study_joints>(
        cartan::bench::description_for("irb120"), cartan::bench::periodic_rule::canonical);
    return feasible;
}

const cartan::bench::target_pool& study_target_pool()
{
    static const cartan::bench::target_pool pool(
        study_feasible_set(), "reachable", pool_targets, pool_seed);
    return pool;
}

cartan::bench::solve_budget study_budget()
{
    return cartan::bench::solve_budget{0, budget_units, budget_tolerance, "work_units"};
}

void bm_study_cartan_lm(benchmark::State& state)
{
    const auto& feasible = study_feasible_set();
    const auto& pool = study_target_pool();
    const auto budget = study_budget();
    int index = 0;

    for (auto _ : state)
    {
        auto outcome = cartan::bench::timed_cartan_lm<study_joints>(
            feasible, pool.target(index), pool.seed(index), budget);
        benchmark::DoNotOptimize(outcome);
        index = (index + 1 == pool_targets) ? 0 : index + 1;
    }
}

#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
void bm_study_pinocchio_lm(benchmark::State& state)
{
    const auto& feasible = study_feasible_set();
    const auto& pool = study_target_pool();
    const auto budget = study_budget();
    cartan::bench::pinocchio_lm_solver<study_joints> peer(feasible);
    int index = 0;

    for (auto _ : state)
    {
        auto outcome = peer(feasible, pool.target(index), pool.seed(index), budget);
        benchmark::DoNotOptimize(outcome);
        index = (index + 1 == pool_targets) ? 0 : index + 1;
    }
}
#endif

}

BENCHMARK(bm_study_cartan_lm);
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
BENCHMARK(bm_study_pinocchio_lm);
#endif
