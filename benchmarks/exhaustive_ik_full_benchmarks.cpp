/// @file exhaustive_ik_full_benchmarks.cpp
/// @brief Full matrix benchmarks for exhaustive_ik_runner across all robots.
///
/// Runs the exhaustive solver with projected_lm on all 9 robots with 100
/// FK-generated targets per robot (seed=42). Reports solution counts,
/// solve rate, FK validation rate, dedup reduction, and ranking behavior.
///
/// Fewer targets than basic_ik_full_benchmarks (100 vs 1000) because the
/// exhaustive solver runs all restarts per target — each solve is ~50-100x
/// more expensive than a single basic_ik_runner solve.

#include "../profiling/chain_factories.h"

#include <cartan/serial/ik/ik_validation.h>
#include <cartan/serial/ik/solver/exhaustive_ik_runner.h>
#include <cartan/serial/ik/solver/projected_lm.h>

#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <benchmark/benchmark.h>

#include <vector>
#include <random>
#include <cmath>
#include <numbers>
#include <algorithm>

namespace
{

constexpr int num_targets = 100;
constexpr double wrap_tol = 0.1;

template <typename VecA, typename VecB>
bool is_wrap_equivalent(const VecA& a, const VecB& b, int n)
{
    constexpr double two_pi = 2.0 * std::numbers::pi;
    for (int j = 0; j < n; ++j)
    {
        double diff = std::fmod(std::abs(a[j] - b[j]), two_pi);
        if (diff > two_pi - wrap_tol)
            diff = two_pi - diff;
        if (diff > wrap_tol)
            return false;
    }
    return true;
}

template <typename Scalar, int N>
int count_unwrapped(
    const std::vector<cartan::ik_result<Scalar, N>>& solutions,
    int n_joints)
{
    if (solutions.empty()) return 0;

    std::vector<bool> is_wrap(solutions.size(), false);
    for (std::size_t i = 1; i < solutions.size(); ++i)
    {
        for (std::size_t j = 0; j < i; ++j)
        {
            if (!is_wrap[j] && is_wrap_equivalent(
                    solutions[i].solution.position,
                    solutions[j].solution.position, n_joints))
            {
                is_wrap[i] = true;
                break;
            }
        }
    }

    int unique = 0;
    for (bool w : is_wrap)
        if (!w) ++unique;
    return unique;
}

template <typename Scalar, int N>
struct target_set
{
    using position_type = typename cartan::joint_state<Scalar, N>::position_type;

    std::vector<cartan::se3<Scalar>> targets;
    std::vector<position_type> seeds;

    target_set(const cartan::kinematic_chain<Scalar, N>& chain, int count, unsigned seed = 42)
    {
        std::mt19937 rng(seed);
        targets.reserve(static_cast<std::size_t>(count));
        seeds.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i)
        {
            targets.push_back(cartan::benchmarks::random_reachable_target(chain, rng));
            seeds.push_back(cartan::benchmarks::random_joint_config(chain, rng));
        }
    }
};

// ============================================================================
// Generic exhaustive benchmark
// ============================================================================

template <int N, typename Policy>
void bm_exhaustive(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts,
    const cartan::convergence_criteria<double>& criteria,
    const cartan::exhaustive_options<double>& options)
{
    using runner_type = cartan::exhaustive_ik_runner<cartan::kinematic_chain<double, N>, Policy>;

    int n_joints = chain.num_joints();

    std::size_t idx = 0;
    int total_solutions = 0;
    int total_unwrapped = 0;
    int min_solutions = options.max_restarts + 1;
    int max_solutions = 0;
    int min_unwrapped = options.max_restarts + 1;
    int max_unwrapped = 0;
    int total_before_dedup = 0;
    int total_fk_failed = 0;
    int targets_with_solutions = 0;
    int total_restarts = 0;
    double total_best_error = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        runner_type runner;
        auto result = runner.solve(chain, target, seed, criteria, options);

        int n_sol = static_cast<int>(result.solutions.size());
        int n_unwrapped = count_unwrapped(result.solutions, n_joints);

        total_solutions += n_sol;
        total_unwrapped += n_unwrapped;
        total_before_dedup += result.solutions_before_dedup;
        total_fk_failed += result.fk_validations_failed;
        total_restarts += result.restarts_attempted;

        if (n_sol > 0)
        {
            ++targets_with_solutions;
            total_best_error += result.solutions[0].final_error_norm;
        }

        min_solutions = std::min(min_solutions, n_sol);
        max_solutions = std::max(max_solutions, n_sol);
        min_unwrapped = std::min(min_unwrapped, n_unwrapped);
        max_unwrapped = std::max(max_unwrapped, n_unwrapped);

        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);

    state.counters["Solve_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(targets_with_solutions) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);

    state.counters["Converge_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(total_before_dedup) / std::max(total_restarts, 1),
        benchmark::Counter::kAvgThreads);

    state.counters["FK_reject_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(total_fk_failed)
            / std::max(total_before_dedup + total_fk_failed, 1),
        benchmark::Counter::kAvgThreads);

    state.counters["Dedup_reduction"] = benchmark::Counter(
        100.0 * static_cast<double>(total_before_dedup - total_solutions)
            / std::max(total_before_dedup, 1),
        benchmark::Counter::kAvgThreads);

    state.counters["Avg_solutions"] = benchmark::Counter(
        static_cast<double>(total_solutions) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);

    state.counters["Min_solutions"] = benchmark::Counter(
        static_cast<double>(min_solutions),
        benchmark::Counter::kAvgThreads);

    state.counters["Max_solutions"] = benchmark::Counter(
        static_cast<double>(max_solutions),
        benchmark::Counter::kAvgThreads);

    state.counters["Avg_best_error"] = benchmark::Counter(
        total_best_error / std::max(targets_with_solutions, 1),
        benchmark::Counter::kAvgThreads);

    state.counters["Avg_unwrapped"] = benchmark::Counter(
        static_cast<double>(total_unwrapped) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);

    state.counters["Min_unwrapped"] = benchmark::Counter(
        static_cast<double>(min_unwrapped),
        benchmark::Counter::kAvgThreads);

    state.counters["Max_unwrapped"] = benchmark::Counter(
        static_cast<double>(max_unwrapped),
        benchmark::Counter::kAvgThreads);

    state.counters["Wrap_ratio"] = benchmark::Counter(
        100.0 * static_cast<double>(total_solutions - total_unwrapped)
            / std::max(total_solutions, 1),
        benchmark::Counter::kAvgThreads);
}

// ============================================================================
// Solver type aliases
// ============================================================================

template <int N>
using chain_t = cartan::kinematic_chain<double, N>;

template <int N>
using plm_policy = cartan::ik::projected_lm<chain_t<N>>;

inline cartan::convergence_criteria<double> exhaustive_criteria()
{
    return {1e-5, 1e-5, 200};
}

inline cartan::exhaustive_options<double> exhaustive_opts_50()
{
    return {50, 1e-3, cartan::ranking_strategy::distance_to_seed};
}

inline cartan::exhaustive_options<double> exhaustive_opts_100()
{
    return {100, 1e-3, cartan::ranking_strategy::distance_to_seed};
}

inline cartan::exhaustive_options<double> exhaustive_opts_200()
{
    return {200, 1e-3, cartan::ranking_strategy::distance_to_seed};
}

// ============================================================================
// Registration macros — 6-DOF
// ============================================================================

#define REGISTER_EXHAUSTIVE_6DOF(ROBOT, CHAIN_FN)                                                      \
                                                                                                       \
static void bm_exhaustive_##ROBOT##_r50(benchmark::State& state)                                       \
{                                                                                                      \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                     \
    bm_exhaustive<6, plm_policy<6>>(state, chain, ts, exhaustive_criteria(), exhaustive_opts_50());     \
}                                                                                                      \
BENCHMARK(bm_exhaustive_##ROBOT##_r50)->Iterations(num_targets)->Unit(benchmark::kMillisecond);         \
                                                                                                       \
static void bm_exhaustive_##ROBOT##_r100(benchmark::State& state)                                      \
{                                                                                                      \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                     \
    bm_exhaustive<6, plm_policy<6>>(state, chain, ts, exhaustive_criteria(), exhaustive_opts_100());    \
}                                                                                                      \
BENCHMARK(bm_exhaustive_##ROBOT##_r100)->Iterations(num_targets)->Unit(benchmark::kMillisecond);        \
                                                                                                       \
static void bm_exhaustive_##ROBOT##_r200(benchmark::State& state)                                      \
{                                                                                                      \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                     \
    bm_exhaustive<6, plm_policy<6>>(state, chain, ts, exhaustive_criteria(), exhaustive_opts_200());    \
}                                                                                                      \
BENCHMARK(bm_exhaustive_##ROBOT##_r200)->Iterations(num_targets)->Unit(benchmark::kMillisecond);

// ============================================================================
// Registration macros — 7-DOF
// ============================================================================

#define REGISTER_EXHAUSTIVE_7DOF(ROBOT, CHAIN_FN)                                                      \
                                                                                                       \
static void bm_exhaustive_##ROBOT##_r50(benchmark::State& state)                                       \
{                                                                                                      \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                     \
    bm_exhaustive<7, plm_policy<7>>(state, chain, ts, exhaustive_criteria(), exhaustive_opts_50());     \
}                                                                                                      \
BENCHMARK(bm_exhaustive_##ROBOT##_r50)->Iterations(num_targets)->Unit(benchmark::kMillisecond);         \
                                                                                                       \
static void bm_exhaustive_##ROBOT##_r100(benchmark::State& state)                                      \
{                                                                                                      \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                     \
    bm_exhaustive<7, plm_policy<7>>(state, chain, ts, exhaustive_criteria(), exhaustive_opts_100());    \
}                                                                                                      \
BENCHMARK(bm_exhaustive_##ROBOT##_r100)->Iterations(num_targets)->Unit(benchmark::kMillisecond);        \
                                                                                                       \
static void bm_exhaustive_##ROBOT##_r200(benchmark::State& state)                                      \
{                                                                                                      \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                     \
    bm_exhaustive<7, plm_policy<7>>(state, chain, ts, exhaustive_criteria(), exhaustive_opts_200());    \
}                                                                                                      \
BENCHMARK(bm_exhaustive_##ROBOT##_r200)->Iterations(num_targets)->Unit(benchmark::kMillisecond);

// ============================================================================
// 6-DOF robots
// ============================================================================

REGISTER_EXHAUSTIVE_6DOF(ur3e,       make_ur3e_chain)
REGISTER_EXHAUSTIVE_6DOF(kr6_sixx,   make_kr6_sixx_chain)
REGISTER_EXHAUSTIVE_6DOF(abb_irb120, make_abb_irb120_chain)
REGISTER_EXHAUSTIVE_6DOF(jaco2,      make_jaco2_chain)

// ============================================================================
// 7-DOF robots
// ============================================================================

REGISTER_EXHAUSTIVE_7DOF(lbr_med14,  make_lbr_med14_chain)
REGISTER_EXHAUSTIVE_7DOF(panda,      make_panda_chain)
REGISTER_EXHAUSTIVE_7DOF(fetch,      make_fetch_chain)
REGISTER_EXHAUSTIVE_7DOF(baxter,     make_baxter_chain)
REGISTER_EXHAUSTIVE_7DOF(kuka_lwr4,  make_kuka_lwr4_chain)

}
