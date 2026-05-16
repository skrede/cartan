/// @file ik_profiling.cpp
/// @brief Standalone profiling tool for comparing native, argmin, and NLopt IK
///        solvers across all 9 robots. No external robot library dependencies.

#include "../tests/fixtures/chain_factories.h"

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/policy/limits_policy.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/ik/solvers.h>
#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/solver/lbfgsb.h>

#ifdef CARTAN_BUILD_ARGMIN
#include <cartan/serial/ik/solver/argmin_slsqp.h>
#endif
#include <cartan/serial/ik/wrapper/restart_wrapper.h>
#include <cartan/serial/ik/solver/projected_lm.h>
#include <cartan/serial/ik/solver/newton_raphson.h>

#ifdef CARTAN_HAS_NLOPT
#include <cartan/serial/ik/solver/nlopt_slsqp.h>
#endif

#include <benchmark/benchmark.h>

#include <vector>
#include <random>
#include <algorithm>

namespace
{

constexpr int num_targets = 1000;

// ============================================================================
// Pre-generated target/seed sets
// ============================================================================

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
            targets.push_back(cartan::fixtures::random_reachable_target(chain, rng));
            seeds.push_back(cartan::fixtures::random_joint_config(chain, rng));
        }
    }
};

// ============================================================================
// Generic benchmark for basic_ik_runner-wrapped policies
// ============================================================================

template <int N, typename Solver>
void bm_full_solver(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts,
    const cartan::convergence_criteria<double>& criteria)
{
    std::size_t idx = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_position_error = 0.0;
    double total_orientation_error = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        Solver solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iterations += result->iterations;
            auto [position_error, orientation_error] = cartan::fixtures::compute_pose_errors(
                chain, result->solution.position, target);
            total_position_error += position_error;
            total_orientation_error += orientation_error;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_iterations"] = benchmark::Counter(
        static_cast<double>(total_iterations) / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_position_error"] = benchmark::Counter(
        total_position_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_orientation_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

// ============================================================================
// Solver type aliases
// ============================================================================

// Chain type shorthand
template <int N>
using chain_t = cartan::kinematic_chain<double, N>;

// Native family
template <int N>
using speed_ik_solver = cartan::basic_ik_runner<cartan::speed_ik_runner<chain_t<N>>>;

template <int N>
using convergence_ik_solver = cartan::basic_ik_runner<cartan::robust_ik_runner<chain_t<N>>>;

template <int N>
using restart_lm = cartan::ik::restart_wrapper<chain_t<N>, cartan::ik::lm<chain_t<N>, cartan::no_limits>>;

template <int N>
using restart_lm_ik_solver = cartan::basic_ik_runner<restart_lm<N>>;

template <int N>
using racing_solver = cartan::dual_ik_runner<chain_t<N>>;

#ifdef CARTAN_BUILD_ARGMIN
// argmin family (available when argmin is built)
template <int N>
using argmin_slsqp_restart = cartan::ik::restart_wrapper<chain_t<N>, cartan::ik::argmin_slsqp<chain_t<N>>>;

template <int N>
using argmin_slsqp_solver = cartan::basic_ik_runner<argmin_slsqp_restart<N>>;
#endif

// NLopt family (behind CARTAN_HAS_NLOPT)
#ifdef CARTAN_HAS_NLOPT
template <int N>
using nlopt_slsqp_restart = cartan::ik::restart_wrapper<chain_t<N>, cartan::ik::nlopt_slsqp<chain_t<N>>>;

template <int N>
using nlopt_slsqp_solver = cartan::basic_ik_runner<nlopt_slsqp_restart<N>>;
#endif

// ============================================================================
// Convergence criteria per solver family
// ============================================================================

inline cartan::convergence_criteria<double> speed_criteria()                { return {1e-5, 1e-5, 200}; }
inline cartan::convergence_criteria<double> convergence_criteria_tuned()    { return {1e-5, 1e-5, 500}; }
inline cartan::convergence_criteria<double> restart_lm_criteria()           { return {1e-5, 1e-5, 200}; }
#ifdef CARTAN_BUILD_ARGMIN
inline cartan::convergence_criteria<double> argmin_criteria()              { return {1e-5, 1e-5, 500}; }
#endif
inline cartan::convergence_criteria<double> nlopt_criteria()                { return {1e-5, 1e-5, 500}; }

// ============================================================================
// Macro-based benchmark registration
// ============================================================================

// Register native + argmin solver benchmarks for a 6-DOF robot.
#define REGISTER_6DOF_PROFILING(ROBOT, CHAIN_FN)                                                       \
                                                                                                       \
static void bm_profiling_##ROBOT##_cartan_speed(benchmark::State& state)                                      \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                     \
    bm_full_solver<6, speed_ik_solver<6>>(state, chain, ts, speed_criteria());                       \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);               \
                                                                                                       \
static void bm_profiling_##ROBOT##_cartan_convergence(benchmark::State& state)                                \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                     \
    bm_full_solver<6, convergence_ik_solver<6>>(state, chain, ts, convergence_criteria_tuned());     \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_cartan_convergence)->Iterations(1000)->Unit(benchmark::kMicrosecond);         \
                                                                                                       \
static void bm_profiling_##ROBOT##_cartan_restart_lm(benchmark::State& state)                                 \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                     \
    bm_full_solver<6, restart_lm_ik_solver<6>>(state, chain, ts, restart_lm_criteria());             \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);          \
                                                                                                       \
static void bm_profiling_##ROBOT##_cartan_racing(benchmark::State& state)                                     \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                     \
    bm_full_solver<6, racing_solver<6>>(state, chain, ts, convergence_criteria_tuned());             \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

#ifdef CARTAN_BUILD_ARGMIN
#define REGISTER_6DOF_ARGMIN_PROFILING(ROBOT, CHAIN_FN)                                               \
                                                                                                       \
static void bm_profiling_##ROBOT##_argmin_slsqp(benchmark::State& state)                              \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                     \
    bm_full_solver<6, argmin_slsqp_solver<6>>(state, chain, ts, argmin_criteria());                \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_argmin_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);
#else
#define REGISTER_6DOF_ARGMIN_PROFILING(ROBOT, CHAIN_FN)
#endif

// Register NLopt solver benchmarks for a 6-DOF robot.
#ifdef CARTAN_HAS_NLOPT
#define REGISTER_6DOF_NLOPT_PROFILING(ROBOT, CHAIN_FN)                                                 \
                                                                                                       \
static void bm_profiling_##ROBOT##_nlopt_slsqp(benchmark::State& state)                                \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                     \
    bm_full_solver<6, nlopt_slsqp_solver<6>>(state, chain, ts, nlopt_criteria());                    \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_nlopt_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);
#else
#define REGISTER_6DOF_NLOPT_PROFILING(ROBOT, CHAIN_FN)
#endif

// Register native + argmin solver benchmarks for a 7-DOF robot.
#define REGISTER_7DOF_PROFILING(ROBOT, CHAIN_FN)                                                       \
                                                                                                       \
static void bm_profiling_##ROBOT##_cartan_speed(benchmark::State& state)                                      \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                     \
    bm_full_solver<7, speed_ik_solver<7>>(state, chain, ts, speed_criteria());                       \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);               \
                                                                                                       \
static void bm_profiling_##ROBOT##_cartan_convergence(benchmark::State& state)                                \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                     \
    bm_full_solver<7, convergence_ik_solver<7>>(state, chain, ts, convergence_criteria_tuned());     \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_cartan_convergence)->Iterations(1000)->Unit(benchmark::kMicrosecond);         \
                                                                                                       \
static void bm_profiling_##ROBOT##_cartan_restart_lm(benchmark::State& state)                                 \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                     \
    bm_full_solver<7, restart_lm_ik_solver<7>>(state, chain, ts, restart_lm_criteria());             \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);          \
                                                                                                       \
static void bm_profiling_##ROBOT##_cartan_racing(benchmark::State& state)                                     \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                     \
    bm_full_solver<7, racing_solver<7>>(state, chain, ts, convergence_criteria_tuned());             \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

#ifdef CARTAN_BUILD_ARGMIN
#define REGISTER_7DOF_ARGMIN_PROFILING(ROBOT, CHAIN_FN)                                               \
                                                                                                       \
static void bm_profiling_##ROBOT##_argmin_slsqp(benchmark::State& state)                              \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                     \
    bm_full_solver<7, argmin_slsqp_solver<7>>(state, chain, ts, argmin_criteria());                \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_argmin_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);
#else
#define REGISTER_7DOF_ARGMIN_PROFILING(ROBOT, CHAIN_FN)
#endif

// Register NLopt solver benchmarks for a 7-DOF robot.
#ifdef CARTAN_HAS_NLOPT
#define REGISTER_7DOF_NLOPT_PROFILING(ROBOT, CHAIN_FN)                                                 \
                                                                                                       \
static void bm_profiling_##ROBOT##_nlopt_slsqp(benchmark::State& state)                                \
{                                                                                                      \
    auto chain = cartan::fixtures::CHAIN_FN<double>();                                                \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                     \
    bm_full_solver<7, nlopt_slsqp_solver<7>>(state, chain, ts, nlopt_criteria());                    \
}                                                                                                      \
BENCHMARK(bm_profiling_##ROBOT##_nlopt_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);
#else
#define REGISTER_7DOF_NLOPT_PROFILING(ROBOT, CHAIN_FN)
#endif

// ============================================================================
// 6-DOF robots
// ============================================================================

REGISTER_6DOF_PROFILING(ur3e, make_ur3e_chain)
REGISTER_6DOF_ARGMIN_PROFILING(ur3e, make_ur3e_chain)
REGISTER_6DOF_NLOPT_PROFILING(ur3e, make_ur3e_chain)

REGISTER_6DOF_PROFILING(kr6_sixx, make_kr6_sixx_chain)
REGISTER_6DOF_ARGMIN_PROFILING(kr6_sixx, make_kr6_sixx_chain)
REGISTER_6DOF_NLOPT_PROFILING(kr6_sixx, make_kr6_sixx_chain)

REGISTER_6DOF_PROFILING(abb_irb120, make_abb_irb120_chain)
REGISTER_6DOF_ARGMIN_PROFILING(abb_irb120, make_abb_irb120_chain)
REGISTER_6DOF_NLOPT_PROFILING(abb_irb120, make_abb_irb120_chain)

REGISTER_6DOF_PROFILING(jaco2, make_jaco2_chain)
REGISTER_6DOF_ARGMIN_PROFILING(jaco2, make_jaco2_chain)
REGISTER_6DOF_NLOPT_PROFILING(jaco2, make_jaco2_chain)

// ============================================================================
// 7-DOF robots
// ============================================================================

REGISTER_7DOF_PROFILING(lbr_med14, make_lbr_med14_chain)
REGISTER_7DOF_ARGMIN_PROFILING(lbr_med14, make_lbr_med14_chain)
REGISTER_7DOF_NLOPT_PROFILING(lbr_med14, make_lbr_med14_chain)

REGISTER_7DOF_PROFILING(panda, make_panda_chain)
REGISTER_7DOF_ARGMIN_PROFILING(panda, make_panda_chain)
REGISTER_7DOF_NLOPT_PROFILING(panda, make_panda_chain)

REGISTER_7DOF_PROFILING(fetch, make_fetch_chain)
REGISTER_7DOF_ARGMIN_PROFILING(fetch, make_fetch_chain)
REGISTER_7DOF_NLOPT_PROFILING(fetch, make_fetch_chain)

REGISTER_7DOF_PROFILING(baxter, make_baxter_chain)
REGISTER_7DOF_ARGMIN_PROFILING(baxter, make_baxter_chain)
REGISTER_7DOF_NLOPT_PROFILING(baxter, make_baxter_chain)

REGISTER_7DOF_PROFILING(kuka_lwr4, make_kuka_lwr4_chain)
REGISTER_7DOF_ARGMIN_PROFILING(kuka_lwr4, make_kuka_lwr4_chain)
REGISTER_7DOF_NLOPT_PROFILING(kuka_lwr4, make_kuka_lwr4_chain)

}
