/// @file ik_study_benchmarks.cpp
/// @brief The study's timed pass: registration, and nothing else.
///
/// Inside a measured block there is the solve alone. Every counter, every
/// adjudication and every file write belongs to the untimed capture pass, which
/// is the whole reason the study runs two passes over the same target pool.
/// The configure-time gate over this directory catches a checked kinematics
/// entry point in a measured block; it cannot catch a counter, so that half of
/// the discipline has to live in the shape of this file.
///
/// One cell per rung of the ladder. A single cell would time one budget and
/// leave a reader to assume the curve through it.

#include "study/budget.h"
#include "study/target_pool.h"
#include "study/feasible_set.h"
#include "study/participants.h"

#include <cartan/lie/se3.h>

#include <benchmark/benchmark.h>

#include <cstddef>

namespace
{

constexpr int study_joints = 6;
using feasible_type = cartan::bench::feasible_set<study_joints>;

constexpr std::uint64_t pool_seed = cartan::bench::k_target_pool_seed;
constexpr int pool_targets = 200;
constexpr double budget_tolerance = 1e-5;

const feasible_type& study_feasible_set()
{
    static const feasible_type feasible = cartan::bench::load_feasible_set<study_joints>(
        cartan::bench::description_for("irb120"), cartan::bench::periodic_rule::canonical,
        cartan::bench::limits_provenance::description);
    return feasible;
}

const cartan::bench::target_pool<study_joints>& study_target_pool()
{
    static const cartan::bench::target_pool<study_joints> pool(
        study_feasible_set(), cartan::bench::stratum::reachable, pool_targets, pool_seed);
    return pool;
}

cartan::bench::solve_budget rung_budget(int index)
{
    return cartan::bench::solve_budget_for(
        cartan::bench::budget_ladder()[static_cast<std::size_t>(index)],
        cartan::bench::stratum::reachable, budget_tolerance, true);
}

void bm_study_cartan_lm(benchmark::State& state)
{
    const auto& feasible = study_feasible_set();
    const auto& pool = study_target_pool();
    const auto budget = rung_budget(static_cast<int>(state.range(0)));
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
    const auto budget = rung_budget(static_cast<int>(state.range(0)));
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

BENCHMARK(bm_study_cartan_lm)->DenseRange(0, cartan::bench::k_budget_points - 1);
#ifdef CARTAN_BENCH_STUDY_HAS_PINOCCHIO
BENCHMARK(bm_study_pinocchio_lm)->DenseRange(0, cartan::bench::k_budget_points - 1);
#endif
