/// @file ik_comparison_benchmarks.cpp
/// @brief Head-to-head comparison of cartan IK vs TRAC-IK for ~9 robots.
///
/// Both solvers use the same random seed (42), same 10,000 targets (FK-generated,
/// guaranteed reachable), and same convergence tolerance (eps = 1e-5).
/// TRAC-IK uses maxtime=10.0 for convergence-based termination (Pitfall 3).
/// cartan uses LM stepper (closest algorithmic match to TRAC-IK's Newton methods)
/// and restart+LM (random-restart wrapper, closer to TRAC-IK's actual strategy).
///
/// Reference: Beeson, P., & Ames, B. (2015). TRAC-IK: An open-source library
///            for improved solving of generic inverse kinematics.

#include "benchmark_utils.h"

#include <cartan/serial/ik/ik_types.h>
#include <cartan/serial/ik/limits_policy.h>
#include <cartan/serial/ik/default_solvers.h>
#include <cartan/serial/ik/basic_ik_solver.h>
#include <cartan/serial/ik/lm_solve_policy.h>
#include <cartan/serial/ik/slsqp_solve_policy.h>
#include <cartan/serial/ik/bobyqa_solve_policy.h>
#include <cartan/serial/ik/nw_sqp_solve_policy.h>
#include <cartan/serial/ik/restart_solve_policy.h>
#include <cartan/serial/ik/nablapp_lbfgsb_solve_policy.h>

#ifdef CARTAN_HAS_NLOPT
#include <cartan/serial/ik/nlopt_bobyqa_solve_policy.h>
#include <cartan/serial/ik/nlopt_slsqp_solve_policy.h>
#endif

#include <trac_ik/trac_ik.hpp>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include <benchmark/benchmark.h>

#include <random>
#include <vector>
#include <algorithm>

namespace
{

constexpr int num_targets = 10000;

// ============================================================================
// Template helpers to reduce boilerplate across 9 robots x 2 solvers
// ============================================================================

/// Pre-generate cartan targets and seeds for a given chain.
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

/// cartan LM benchmark for a fixed-N chain.
template <int N>
void bm_cartan_comparison(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts)
{
    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 100};

    std::size_t idx = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_pos_error = 0.0;
    double total_ori_error = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::basic_ik_solver<cartan::lm_solve_policy<cartan::kinematic_chain<double, N>>> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iterations += result->iterations;
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos_error += pos_err;
            total_ori_error += ori_err;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_iterations"] = benchmark::Counter(
        static_cast<double>(total_iterations) / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_position_error"] = benchmark::Counter(
        total_pos_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_ori_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

/// cartan restart+LM benchmark for a fixed-N chain.
template <int N, typename LimitsPolicy = cartan::no_limits>
void bm_cartan_restart_lm_comparison(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts)
{
    using chain_t = cartan::kinematic_chain<double, N>;
    using restart_lm = cartan::restart_solve_policy<
        chain_t, cartan::lm_solve_policy<chain_t, LimitsPolicy>>;

    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 200};

    std::size_t idx = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_pos_error = 0.0;
    double total_ori_error = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::basic_ik_solver<restart_lm> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iterations += result->iterations;
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos_error += pos_err;
            total_ori_error += ori_err;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_iterations"] = benchmark::Counter(
        static_cast<double>(total_iterations) / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_position_error"] = benchmark::Counter(
        total_pos_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_ori_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

/// cartan speed solver (restart+projected LM with joint limits) benchmark.
template <int N>
void bm_cartan_speed_comparison(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts)
{
    using chain_t = cartan::kinematic_chain<double, N>;

    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 200};

    std::size_t idx = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_pos_error = 0.0;
    double total_ori_error = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::basic_ik_solver<cartan::speed_solver<chain_t>> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iterations += result->iterations;
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos_error += pos_err;
            total_ori_error += ori_err;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_iterations"] = benchmark::Counter(
        static_cast<double>(total_iterations) / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_position_error"] = benchmark::Counter(
        total_pos_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_ori_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

/// cartan racing solver (speed + convergence, both limit-respecting) benchmark.
template <int N>
void bm_cartan_racing_comparison(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts)
{
    using chain_t = cartan::kinematic_chain<double, N>;

    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    std::size_t idx = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_pos_error = 0.0;
    double total_ori_error = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::default_solver<chain_t> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iterations += result->iterations;
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos_error += pos_err;
            total_ori_error += ori_err;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_iterations"] = benchmark::Counter(
        static_cast<double>(total_iterations) / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_position_error"] = benchmark::Counter(
        total_pos_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_ori_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

/// TRAC-IK benchmark for a given KDL chain + pre-generated targets.
template <int N>
void bm_trac_ik_comparison(
    benchmark::State& state,
    const KDL::Chain& kdl_chain,
    KDL::JntArray q_min,
    KDL::JntArray q_max,
    const target_set<double, N>& ts)
{
    // Convert cartan targets to KDL frames and seeds to KDL JntArrays
    auto count = static_cast<std::size_t>(num_targets);
    std::vector<KDL::Frame> kdl_targets;
    std::vector<KDL::JntArray> kdl_seeds;
    kdl_targets.reserve(count);
    kdl_seeds.reserve(count);

    for (std::size_t i = 0; i < count; ++i)
    {
        kdl_targets.push_back(cartan::benchmarks::se3_to_kdl_frame(ts.targets[i]));
        KDL::JntArray kdl_seed(static_cast<unsigned int>(N));
        for (unsigned int j = 0; j < static_cast<unsigned int>(N); ++j)
        {
            kdl_seed(j) = ts.seeds[i](static_cast<int>(j));
        }
        kdl_seeds.push_back(kdl_seed);
    }

    // TRAC-IK with large timeout for convergence-based termination (Pitfall 3)
    // eps=1e-5 matches cartan convergence tolerance
    TRAC_IK::TRAC_IK solver(kdl_chain, q_min, q_max,
                             /*maxtime=*/10.0, /*eps=*/1e-5, TRAC_IK::Speed);

    std::size_t idx = 0;
    int successes = 0;

    for (auto _ : state)
    {
        auto& target = kdl_targets[idx % count];
        auto& q_init = kdl_seeds[idx % count];
        ++idx;

        KDL::JntArray q_out(static_cast<unsigned int>(N));
        int rc = solver.CartToJnt(q_init, target, q_out);

        if (rc >= 0)
        {
            ++successes;
        }
        benchmark::DoNotOptimize(q_out);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_iterations"] = benchmark::Counter(0, benchmark::Counter::kAvgThreads);
    state.counters["avg_position_error"] = benchmark::Counter(0, benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(0, benchmark::Counter::kAvgThreads);
}

// ============================================================================
// 6-DOF robots
// ============================================================================

// --- UR3e ---

static void bm_comparison_ur3e_cartan(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_ur3e_cartan)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_trac_ik(benchmark::State& state)
{
    auto cartan_chain = cartan::benchmarks::make_ur3e_chain<double>();
    auto kdl_chain = cartan::benchmarks::make_ur3e_kdl_chain();
    KDL::JntArray q_min(6), q_max(6);
    cartan::benchmarks::make_ur3e_kdl_limits(q_min, q_max);
    static const target_set<double, 6> ts(cartan_chain, num_targets, 42);
    bm_trac_ik_comparison<6>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_ur3e_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_cartan_restart_lm(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_ur3e_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_cartan_restart_lm_clamped(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<6, cartan::clamp_limits>(state, chain, ts);
}
BENCHMARK(bm_comparison_ur3e_cartan_restart_lm_clamped)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_cartan_speed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_speed_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_ur3e_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_cartan_racing(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_racing_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_ur3e_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- KR 6 SIXX ---

static void bm_comparison_kr6_sixx_cartan(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_kr6_sixx_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_kr6_sixx_cartan)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kr6_sixx_trac_ik(benchmark::State& state)
{
    auto cartan_chain = cartan::benchmarks::make_kr6_sixx_chain<double>();
    auto kdl_chain = cartan::benchmarks::make_kr6_sixx_kdl_chain();
    KDL::JntArray q_min(6), q_max(6);
    cartan::benchmarks::make_kr6_sixx_kdl_limits(q_min, q_max);
    static const target_set<double, 6> ts(cartan_chain, num_targets, 42);
    bm_trac_ik_comparison<6>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_kr6_sixx_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kr6_sixx_cartan_restart_lm(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_kr6_sixx_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_kr6_sixx_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kr6_sixx_cartan_restart_lm_clamped(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_kr6_sixx_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<6, cartan::clamp_limits>(state, chain, ts);
}
BENCHMARK(bm_comparison_kr6_sixx_cartan_restart_lm_clamped)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kr6_sixx_cartan_speed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_kr6_sixx_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_speed_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_kr6_sixx_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kr6_sixx_cartan_racing(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_kr6_sixx_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_racing_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_kr6_sixx_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- ABB IRB 120 ---

static void bm_comparison_abb_irb120_cartan(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_abb_irb120_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_abb_irb120_cartan)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_abb_irb120_trac_ik(benchmark::State& state)
{
    auto cartan_chain = cartan::benchmarks::make_abb_irb120_chain<double>();
    auto kdl_chain = cartan::benchmarks::make_abb_irb120_kdl_chain();
    KDL::JntArray q_min(6), q_max(6);
    cartan::benchmarks::make_abb_irb120_kdl_limits(q_min, q_max);
    static const target_set<double, 6> ts(cartan_chain, num_targets, 42);
    bm_trac_ik_comparison<6>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_abb_irb120_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_abb_irb120_cartan_restart_lm(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_abb_irb120_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_abb_irb120_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_abb_irb120_cartan_restart_lm_clamped(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_abb_irb120_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<6, cartan::clamp_limits>(state, chain, ts);
}
BENCHMARK(bm_comparison_abb_irb120_cartan_restart_lm_clamped)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_abb_irb120_cartan_speed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_abb_irb120_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_speed_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_abb_irb120_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_abb_irb120_cartan_racing(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_abb_irb120_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_racing_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_abb_irb120_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- Jaco2 ---

static void bm_comparison_jaco2_cartan(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_jaco2_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_jaco2_cartan)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_jaco2_trac_ik(benchmark::State& state)
{
    auto cartan_chain = cartan::benchmarks::make_jaco2_chain<double>();
    auto kdl_chain = cartan::benchmarks::make_jaco2_kdl_chain();
    KDL::JntArray q_min(6), q_max(6);
    cartan::benchmarks::make_jaco2_kdl_limits(q_min, q_max);
    static const target_set<double, 6> ts(cartan_chain, num_targets, 42);
    bm_trac_ik_comparison<6>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_jaco2_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_jaco2_cartan_restart_lm(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_jaco2_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_jaco2_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_jaco2_cartan_restart_lm_clamped(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_jaco2_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<6, cartan::clamp_limits>(state, chain, ts);
}
BENCHMARK(bm_comparison_jaco2_cartan_restart_lm_clamped)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_jaco2_cartan_speed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_jaco2_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_speed_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_jaco2_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_jaco2_cartan_racing(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_jaco2_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_cartan_racing_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_jaco2_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// ============================================================================
// 7-DOF robots
// ============================================================================

// --- LBR Med 14 ---

static void bm_comparison_lbr_med14_cartan(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_lbr_med14_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_lbr_med14_cartan)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_lbr_med14_trac_ik(benchmark::State& state)
{
    auto cartan_chain = cartan::benchmarks::make_lbr_med14_chain<double>();
    auto kdl_chain = cartan::benchmarks::make_lbr_med14_kdl_chain();
    KDL::JntArray q_min(7), q_max(7);
    cartan::benchmarks::make_lbr_med14_kdl_limits(q_min, q_max);
    static const target_set<double, 7> ts(cartan_chain, num_targets, 42);
    bm_trac_ik_comparison<7>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_lbr_med14_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_lbr_med14_cartan_restart_lm(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_lbr_med14_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_lbr_med14_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_lbr_med14_cartan_restart_lm_clamped(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_lbr_med14_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<7, cartan::clamp_limits>(state, chain, ts);
}
BENCHMARK(bm_comparison_lbr_med14_cartan_restart_lm_clamped)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_lbr_med14_cartan_speed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_lbr_med14_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_speed_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_lbr_med14_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_lbr_med14_cartan_racing(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_lbr_med14_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_racing_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_lbr_med14_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- Panda ---

static void bm_comparison_panda_cartan(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_panda_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_panda_cartan)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_panda_trac_ik(benchmark::State& state)
{
    auto cartan_chain = cartan::benchmarks::make_panda_chain<double>();
    auto kdl_chain = cartan::benchmarks::make_panda_kdl_chain();
    KDL::JntArray q_min(7), q_max(7);
    cartan::benchmarks::make_panda_kdl_limits(q_min, q_max);
    static const target_set<double, 7> ts(cartan_chain, num_targets, 42);
    bm_trac_ik_comparison<7>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_panda_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_panda_cartan_restart_lm(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_panda_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_panda_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_panda_cartan_restart_lm_clamped(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_panda_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<7, cartan::clamp_limits>(state, chain, ts);
}
BENCHMARK(bm_comparison_panda_cartan_restart_lm_clamped)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_panda_cartan_speed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_panda_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_speed_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_panda_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_panda_cartan_racing(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_panda_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_racing_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_panda_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- Fetch ---

static void bm_comparison_fetch_cartan(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_fetch_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_fetch_cartan)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_fetch_trac_ik(benchmark::State& state)
{
    auto cartan_chain = cartan::benchmarks::make_fetch_chain<double>();
    auto kdl_chain = cartan::benchmarks::make_fetch_kdl_chain();
    KDL::JntArray q_min(7), q_max(7);
    cartan::benchmarks::make_fetch_kdl_limits(q_min, q_max);
    static const target_set<double, 7> ts(cartan_chain, num_targets, 42);
    bm_trac_ik_comparison<7>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_fetch_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_fetch_cartan_restart_lm(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_fetch_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_fetch_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_fetch_cartan_restart_lm_clamped(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_fetch_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<7, cartan::clamp_limits>(state, chain, ts);
}
BENCHMARK(bm_comparison_fetch_cartan_restart_lm_clamped)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_fetch_cartan_speed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_fetch_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_speed_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_fetch_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_fetch_cartan_racing(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_fetch_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_racing_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_fetch_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- Baxter ---

static void bm_comparison_baxter_cartan(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_baxter_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_baxter_cartan)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_baxter_trac_ik(benchmark::State& state)
{
    auto cartan_chain = cartan::benchmarks::make_baxter_chain<double>();
    auto kdl_chain = cartan::benchmarks::make_baxter_kdl_chain();
    KDL::JntArray q_min(7), q_max(7);
    cartan::benchmarks::make_baxter_kdl_limits(q_min, q_max);
    static const target_set<double, 7> ts(cartan_chain, num_targets, 42);
    bm_trac_ik_comparison<7>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_baxter_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_baxter_cartan_restart_lm(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_baxter_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_baxter_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_baxter_cartan_restart_lm_clamped(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_baxter_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<7, cartan::clamp_limits>(state, chain, ts);
}
BENCHMARK(bm_comparison_baxter_cartan_restart_lm_clamped)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_baxter_cartan_speed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_baxter_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_speed_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_baxter_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_baxter_cartan_racing(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_baxter_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_racing_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_baxter_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- KUKA LWR 4+ ---

static void bm_comparison_kuka_lwr4_cartan(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_kuka_lwr4_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_kuka_lwr4_cartan)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kuka_lwr4_trac_ik(benchmark::State& state)
{
    auto cartan_chain = cartan::benchmarks::make_kuka_lwr4_chain<double>();
    auto kdl_chain = cartan::benchmarks::make_kuka_lwr4_kdl_chain();
    KDL::JntArray q_min(7), q_max(7);
    cartan::benchmarks::make_kuka_lwr4_kdl_limits(q_min, q_max);
    static const target_set<double, 7> ts(cartan_chain, num_targets, 42);
    bm_trac_ik_comparison<7>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_kuka_lwr4_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kuka_lwr4_cartan_restart_lm(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_kuka_lwr4_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_kuka_lwr4_cartan_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kuka_lwr4_cartan_restart_lm_clamped(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_kuka_lwr4_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_restart_lm_comparison<7, cartan::clamp_limits>(state, chain, ts);
}
BENCHMARK(bm_comparison_kuka_lwr4_cartan_restart_lm_clamped)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kuka_lwr4_cartan_speed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_kuka_lwr4_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_speed_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_kuka_lwr4_cartan_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kuka_lwr4_cartan_racing(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_kuka_lwr4_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_cartan_racing_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_kuka_lwr4_cartan_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// ============================================================================
// nablapp comparison benchmarks (three D-10 axes)
// ============================================================================

/// Generic nablapp-backed solver comparison benchmark.
template <int N, typename Solver>
void bm_nablapp_comparison(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts,
    const cartan::convergence_criteria<double>& criteria)
{
    std::size_t idx = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_pos_error = 0.0;
    double total_ori_error = 0.0;

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
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos_error += pos_err;
            total_ori_error += ori_err;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_iterations"] = benchmark::Counter(
        static_cast<double>(total_iterations) / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_position_error"] = benchmark::Counter(
        total_pos_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_ori_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

// Chain alias for convenience
template <int N>
using chain_t = cartan::kinematic_chain<double, N>;

// nablapp solver type aliases for comparison benchmarks
template <int N>
using nablapp_slsqp_solver = cartan::basic_ik_solver<
    cartan::restart_solve_policy<chain_t<N>, cartan::slsqp_solve_policy<chain_t<N>>>>;

template <int N>
using nablapp_lbfgsb_solver = cartan::basic_ik_solver<
    cartan::restart_solve_policy<chain_t<N>, cartan::nablapp_lbfgsb_solve_policy<chain_t<N>>>>;

template <int N>
using nablapp_nw_sqp_solver = cartan::basic_ik_solver<
    cartan::restart_solve_policy<chain_t<N>, cartan::nw_sqp_solve_policy<chain_t<N>>>>;

// NLopt solver type aliases (gated behind CARTAN_HAS_NLOPT)
#ifdef CARTAN_HAS_NLOPT
template <int N>
using nlopt_slsqp_solver = cartan::basic_ik_solver<
    cartan::restart_solve_policy<chain_t<N>, cartan::nlopt_slsqp_solve_policy<chain_t<N>>>>;

template <int N>
using nlopt_bobyqa_solver = cartan::basic_ik_solver<
    cartan::restart_solve_policy<chain_t<N>, cartan::nlopt_bobyqa_solve_policy<chain_t<N>>>>;
#endif

// Convergence criteria shared across comparison benchmarks
inline cartan::convergence_criteria<double> nablapp_comparison_criteria() { return {1e-5, 1e-5, 500}; }

// ============================================================================
// Axis 1: Formulation comparison (bound-constrained vs inequality-constrained)
// ============================================================================
// UR3e representative: nablapp SLSQP (bound-constrained) vs NW-SQP (inequality)

static void bm_comparison_ur3e_nablapp_slsqp_bounded(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_nablapp_comparison<6, nablapp_slsqp_solver<6>>(state, chain, ts, nablapp_comparison_criteria());
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_bounded)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_nw_sqp_inequality(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_nablapp_comparison<6, nablapp_nw_sqp_solver<6>>(state, chain, ts, nablapp_comparison_criteria());
}
BENCHMARK(bm_comparison_ur3e_nw_sqp_inequality)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// ============================================================================
// Axis 2: Backend head-to-head (nablapp vs NLopt same-algorithm)
// ============================================================================

static void bm_comparison_ur3e_nablapp_slsqp(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_nablapp_comparison<6, nablapp_slsqp_solver<6>>(state, chain, ts, nablapp_comparison_criteria());
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

#ifdef CARTAN_HAS_NLOPT
static void bm_comparison_ur3e_nlopt_slsqp(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_nablapp_comparison<6, nlopt_slsqp_solver<6>>(state, chain, ts, nablapp_comparison_criteria());
}
BENCHMARK(bm_comparison_ur3e_nlopt_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_nlopt_bobyqa(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_nablapp_comparison<6, nlopt_bobyqa_solver<6>>(state, chain, ts, nablapp_comparison_criteria());
}
BENCHMARK(bm_comparison_ur3e_nlopt_bobyqa)->Iterations(1000)->Unit(benchmark::kMicrosecond);
#endif

// ============================================================================
// Axis 3: Headline comparison (best nablapp configs vs TRAC-IK)
// ============================================================================
// Registers nablapp SLSQP, L-BFGS-B, and NW-SQP for all 9 robots alongside
// the existing TRAC-IK entries. Uses same target set (seed 42) for fair comparison.

#define REGISTER_6DOF_NABLAPP_COMPARISON(ROBOT, CHAIN_FN)                                             \
                                                                                                      \
static void bm_comparison_##ROBOT##_nablapp_slsqp(benchmark::State& state)                           \
{                                                                                                     \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                    \
    bm_nablapp_comparison<6, nablapp_slsqp_solver<6>>(state, chain, ts, nablapp_comparison_criteria()); \
}                                                                                                     \
BENCHMARK(bm_comparison_##ROBOT##_nablapp_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);    \
                                                                                                      \
static void bm_comparison_##ROBOT##_nablapp_lbfgsb(benchmark::State& state)                          \
{                                                                                                     \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                    \
    bm_nablapp_comparison<6, nablapp_lbfgsb_solver<6>>(state, chain, ts, nablapp_comparison_criteria()); \
}                                                                                                     \
BENCHMARK(bm_comparison_##ROBOT##_nablapp_lbfgsb)->Iterations(1000)->Unit(benchmark::kMicrosecond);   \
                                                                                                      \
static void bm_comparison_##ROBOT##_nw_sqp(benchmark::State& state)                                  \
{                                                                                                     \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                    \
    bm_nablapp_comparison<6, nablapp_nw_sqp_solver<6>>(state, chain, ts, nablapp_comparison_criteria()); \
}                                                                                                     \
BENCHMARK(bm_comparison_##ROBOT##_nw_sqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

#define REGISTER_7DOF_NABLAPP_COMPARISON(ROBOT, CHAIN_FN)                                             \
                                                                                                      \
static void bm_comparison_##ROBOT##_nablapp_slsqp(benchmark::State& state)                           \
{                                                                                                     \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                    \
    bm_nablapp_comparison<7, nablapp_slsqp_solver<7>>(state, chain, ts, nablapp_comparison_criteria()); \
}                                                                                                     \
BENCHMARK(bm_comparison_##ROBOT##_nablapp_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);    \
                                                                                                      \
static void bm_comparison_##ROBOT##_nablapp_lbfgsb(benchmark::State& state)                          \
{                                                                                                     \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                    \
    bm_nablapp_comparison<7, nablapp_lbfgsb_solver<7>>(state, chain, ts, nablapp_comparison_criteria()); \
}                                                                                                     \
BENCHMARK(bm_comparison_##ROBOT##_nablapp_lbfgsb)->Iterations(1000)->Unit(benchmark::kMicrosecond);   \
                                                                                                      \
static void bm_comparison_##ROBOT##_nw_sqp(benchmark::State& state)                                  \
{                                                                                                     \
    auto chain = cartan::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                    \
    bm_nablapp_comparison<7, nablapp_nw_sqp_solver<7>>(state, chain, ts, nablapp_comparison_criteria()); \
}                                                                                                     \
BENCHMARK(bm_comparison_##ROBOT##_nw_sqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// 6-DOF headline nablapp entries
REGISTER_6DOF_NABLAPP_COMPARISON(kr6_sixx,   make_kr6_sixx_chain)
REGISTER_6DOF_NABLAPP_COMPARISON(abb_irb120, make_abb_irb120_chain)
REGISTER_6DOF_NABLAPP_COMPARISON(jaco2,      make_jaco2_chain)

// 7-DOF headline nablapp entries
REGISTER_7DOF_NABLAPP_COMPARISON(lbr_med14,  make_lbr_med14_chain)
REGISTER_7DOF_NABLAPP_COMPARISON(panda,      make_panda_chain)
REGISTER_7DOF_NABLAPP_COMPARISON(fetch,      make_fetch_chain)
REGISTER_7DOF_NABLAPP_COMPARISON(baxter,     make_baxter_chain)
REGISTER_7DOF_NABLAPP_COMPARISON(kuka_lwr4,  make_kuka_lwr4_chain)

} // anonymous namespace
