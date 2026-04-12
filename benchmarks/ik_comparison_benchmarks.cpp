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

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/policy/limits_policy.h>
#include <cartan/serial/ik/solvers.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/solver/argmin_slsqp.h>
#include <cartan/serial/ik/solver/argmin_bobyqa.h>
#include <cartan/serial/ik/solver/nw_sqp.h>
#include <cartan/serial/ik/wrapper/restart_wrapper.h>
#include <cartan/serial/ik/solver/argmin_lbfgsb.h>
#include <cartan/serial/ik/detail/nablapp_problem.h>
#include <cartan/serial/ik/solver/detail/analytical_gradient.h>
#include <cartan/serial/fk/forward_kinematics.h>

#ifdef CARTAN_HAS_NLOPT
#include <cartan/serial/ik/solver/nlopt_bobyqa.h>
#include <cartan/serial/ik/solver/nlopt_slsqp.h>
#endif

#include <trac_ik/trac_ik.hpp>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include <benchmark/benchmark.h>

#include <random>
#include <vector>
#include <algorithm>
#include <cmath>

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

        cartan::basic_ik_runner<cartan::ik::lm<cartan::kinematic_chain<double, N>>> solver;
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
    using restart_lm = cartan::ik::restart_wrapper<
        chain_t, cartan::ik::lm<chain_t, LimitsPolicy>>;

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

        cartan::basic_ik_runner<restart_lm> solver;
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

        cartan::basic_ik_runner<cartan::speed_ik_runner<chain_t>> solver;
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

        cartan::dual_ik_runner<chain_t> solver;
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
using nablapp_slsqp_solver = cartan::basic_ik_runner<
    cartan::ik::restart_wrapper<chain_t<N>, cartan::ik::argmin_slsqp<chain_t<N>>>>;

// NLopt-compatible convergence variant: same solver, different stopping rules.
// Uses nablapp::slsqp_compatible_convergence (ftol_rel + xtol_rel + stall,
// no gradient-norm check) via argmin_slsqp_nlopt_compat.
template <int N>
using nablapp_slsqp_nlopt_compat_solver = cartan::basic_ik_runner<
    cartan::ik::restart_wrapper<chain_t<N>, cartan::ik::argmin_slsqp_nlopt_compat<chain_t<N>>>>;

template <int N>
using nablapp_lbfgsb_solver = cartan::basic_ik_runner<
    cartan::ik::restart_wrapper<chain_t<N>, cartan::ik::argmin_lbfgsb<chain_t<N>>>>;

template <int N>
using nablapp_nw_sqp_solver = cartan::basic_ik_runner<
    cartan::ik::restart_wrapper<chain_t<N>, cartan::ik::nw_sqp<chain_t<N>>>>;

// NLopt solver type aliases (gated behind CARTAN_HAS_NLOPT)
#ifdef CARTAN_HAS_NLOPT
template <int N>
using nlopt_slsqp_solver = cartan::basic_ik_runner<
    cartan::ik::restart_wrapper<chain_t<N>, cartan::ik::nlopt_slsqp<chain_t<N>>>>;

template <int N>
using nlopt_bobyqa_solver = cartan::basic_ik_runner<
    cartan::ik::restart_wrapper<chain_t<N>, cartan::ik::nlopt_bobyqa<chain_t<N>>>>;
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

// NLopt-compatible convergence variant of the UR3e SLSQP bench.
// Uses argmin_slsqp_nlopt_compat which instantiates argmin_slsqp with
// nablapp::slsqp_compatible_convergence. The default ftol_rel=1e-10 /
// xtol_rel=1e-10 relative thresholds on argmin_slsqp::options match
// NLopt SLSQP's stop-rule behavior. This is the key experiment for the
// "nablapp takes 4x more outer iterations than nlopt" gap.
static void bm_comparison_ur3e_nablapp_slsqp_nlopt_compat(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_nablapp_comparison<6, nablapp_slsqp_nlopt_compat_solver<6>>(
        state, chain, ts, nablapp_comparison_criteria());
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_nlopt_compat)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Armijo c1 sweep for nablapp SLSQP on UR3e IK. Diagnostic benchmark set
// accompanying the phi_ls backtrack investigation: loosening c1 reduces
// Armijo aggressiveness, cutting backtrack calls on expensive-objective
// problems like IK where every phi(alpha) call is a full FK evaluation.
// Reference: N&W Eq. 3.6a, p. 33. Three points bracket the default 1e-4.
template <double C1>
static void bm_comparison_ur3e_nablapp_slsqp_c1_impl(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    cartan::ik::argmin_slsqp<chain_t<6>>::options slsqp_opts{};
    slsqp_opts.line_search_c1 = C1;

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

        cartan::ik::argmin_slsqp<chain_t<6>> inner{slsqp_opts};
        cartan::ik::restart_wrapper<chain_t<6>, cartan::ik::argmin_slsqp<chain_t<6>>> wrapper{std::move(inner)};
        nablapp_slsqp_solver<6> solver{std::move(wrapper)};

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
    state.counters["c1"] = benchmark::Counter(C1, benchmark::Counter::kDefaults);
}

static void bm_comparison_ur3e_nablapp_slsqp_c1_1em5(benchmark::State& state)
{
    bm_comparison_ur3e_nablapp_slsqp_c1_impl<1e-5>(state);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_c1_1em5)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_nablapp_slsqp_c1_1em4(benchmark::State& state)
{
    bm_comparison_ur3e_nablapp_slsqp_c1_impl<1e-4>(state);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_c1_1em4)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_nablapp_slsqp_c1_1em3(benchmark::State& state)
{
    bm_comparison_ur3e_nablapp_slsqp_c1_impl<1e-3>(state);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_c1_1em3)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Standalone phi_ls counter read — diagnostic only.
//
// Constructs argmin_slsqp directly (no restart_wrapper, no
// basic_ik_runner) so the cumulative line_search_calls counter on
// kraft_slsqp_policy::state_type survives across the entire solve
// without being reset on a restart. Reports mean calls/nablapp_step
// across 1000 UR3e poses as a benchmark counter. Answers nablapp
// turn-07 revised hypothesis test: <=1.3 means backtracks are not
// the driver; ~2-2.5 matches HS071 compile-time baseline; >=3 means
// backtracks are elevated and nonmonotone merit / c1 loosening is
// warranted.
//
// Two budget points bracket the answer: one with the production
// budget_per_step=50 (matching the racing-runner configuration) and
// one with 500 (so one cartan::argmin_slsqp::step() runs the
// kraft_slsqp to convergence or termination in isolation).
template <int BudgetPerStep>
static void bm_comparison_ur3e_nablapp_slsqp_phi_ls_calls_impl(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    cartan::ik::argmin_slsqp<chain_t<6>>::options slsqp_opts{};
    slsqp_opts.budget_per_step = BudgetPerStep;

    std::size_t idx = 0;
    std::uint64_t total_ls = 0;
    std::uint64_t total_nablapp_steps = 0;
    int total_cartan_outer = 0;
    int successes = 0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::ik::argmin_slsqp<chain_t<6>> solver{slsqp_opts};
        solver.setup(chain, target, q_seed, criteria);

        while (solver.status() == cartan::ik_status::running)
        {
            solver.step(chain);
        }

        total_ls += solver.line_search_calls();
        total_nablapp_steps += static_cast<std::uint64_t>(solver.nablapp_iterations());
        total_cartan_outer += solver.iterations();
        if (solver.status() == cartan::ik_status::converged)
            ++successes;

        benchmark::DoNotOptimize(solver);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["total_phi_ls_calls"] = benchmark::Counter(
        static_cast<double>(total_ls), benchmark::Counter::kDefaults);
    state.counters["total_nablapp_steps"] = benchmark::Counter(
        static_cast<double>(total_nablapp_steps), benchmark::Counter::kDefaults);
    state.counters["phi_ls_per_nablapp_step"] = benchmark::Counter(
        static_cast<double>(total_ls) / std::max<double>(1.0, static_cast<double>(total_nablapp_steps)),
        benchmark::Counter::kDefaults);
    state.counters["avg_nablapp_step_per_pose"] = benchmark::Counter(
        static_cast<double>(total_nablapp_steps) / std::max(total, 1),
        benchmark::Counter::kDefaults);
    state.counters["avg_cartan_outer_iter"] = benchmark::Counter(
        static_cast<double>(total_cartan_outer) / std::max(total, 1),
        benchmark::Counter::kDefaults);
    state.counters["budget"] = benchmark::Counter(
        static_cast<double>(BudgetPerStep), benchmark::Counter::kDefaults);
}

static void bm_comparison_ur3e_nablapp_slsqp_phi_ls_calls_budget50(benchmark::State& state)
{
    bm_comparison_ur3e_nablapp_slsqp_phi_ls_calls_impl<50>(state);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_phi_ls_calls_budget50)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_nablapp_slsqp_phi_ls_calls_budget500(benchmark::State& state)
{
    bm_comparison_ur3e_nablapp_slsqp_phi_ls_calls_impl<500>(state);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_phi_ls_calls_budget500)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Dynamic-N variant: construct argmin_slsqp against a kinematic_chain
// with runtime-known joint count. This instantiates
// kraft_slsqp_policy<cartan::dynamic> (== nablapp::dynamic_dimension),
// which is the pre-workspace-templating code path that Eigen dispatches
// to its dynamic kernels rather than the fixed-6 specialization. Paired
// with the fixed-6 variant above, this isolates whether the compile-
// time N path adds the ~0.9 backtracks/step delta at N=6 that nablapp
// measured at N=4 on HS071 (2.0 compile-time vs 1.11 dynamic).
// Diagnostic: reads the last_check_results telemetry from nablapp's
// convergence policy on a single direct-drive UR3e solve per iteration.
// Answers the turn-11 falsification test: at cartan-tight tolerances
// (gradient=1e-12, objective=1e-14, step=1e-14), which of the four
// default_convergence criteria actually fires? Expected per nablapp's
// turn-11 corrected mechanism: [0] gradient_tolerance NEVER fires
// (unreachable near singular poses), [2] step_tolerance fires.
//
// Emits one counter per criterion plus the first-firing index. 0 means
// "did not fire" (nullopt), otherwise the nablapp::solver_status enum
// value the criterion would have reported.
static void bm_comparison_ur3e_nablapp_slsqp_last_check_results(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    cartan::ik::argmin_slsqp<chain_t<6>>::options slsqp_opts{};
    slsqp_opts.budget_per_step = 500;

    std::size_t idx = 0;
    std::array<std::uint64_t, 4> criterion_fire_count{};
    std::uint64_t solves = 0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::ik::argmin_slsqp<chain_t<6>> solver{slsqp_opts};
        solver.setup(chain, target, q_seed, criteria);

        while (solver.status() == cartan::ik_status::running)
        {
            solver.step(chain);
        }

        const auto results = solver.last_check_results();
        for (std::size_t i = 0; i < results.size(); ++i)
        {
            if (results[i].has_value())
            {
                ++criterion_fire_count[i];
            }
        }
        ++solves;

        benchmark::DoNotOptimize(solver);
    }

    const auto total = std::max<std::uint64_t>(solves, 1);
    state.counters["grad_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[0]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["obj_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[1]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["step_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[2]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["stall_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[3]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["total_solves"] = benchmark::Counter(
        static_cast<double>(total), benchmark::Counter::kDefaults);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_last_check_results)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Gradient-threshold sweep on UR3e SLSQP with default_convergence, direct-
// drive (no runner, no restart wrapper). Parameterizes cartan's
// `gradient_threshold` via state.range(0) as a negative-log10 exponent:
// Arg(6) -> 1e-6, Arg(12) -> 1e-12. objective_threshold and step_threshold
// are held at cartan-tight 1e-14 so only the gradient knob moves. Emits
// wall per solve, pose-tolerance success rate, avg iterations of converged
// solves, and the full last_check_results firing profile at each point.
// Tests whether relaxing gradient_threshold recovers wall on the 19.2%
// gradient_tolerance-bound UR3e subset without degrading the 80.8% step_
// tolerance-bound subset or losing pose-tolerance success.
static void bm_comparison_ur3e_nablapp_slsqp_grad_sweep(benchmark::State& state)
{
    const int exponent = static_cast<int>(state.range(0));
    const double gradient_threshold = std::pow(10.0, -static_cast<double>(exponent));

    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    cartan::ik::argmin_slsqp<chain_t<6>>::options slsqp_opts{};
    slsqp_opts.budget_per_step = 500;
    slsqp_opts.gradient_threshold = gradient_threshold;

    std::size_t idx = 0;
    std::array<std::uint64_t, 4> criterion_fire_count{};
    std::uint64_t solves = 0;
    std::uint64_t pose_successes = 0;
    std::uint64_t total_iterations = 0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::ik::argmin_slsqp<chain_t<6>> solver{slsqp_opts};
        solver.setup(chain, target, q_seed, criteria);

        while (solver.status() == cartan::ik_status::running)
        {
            solver.step(chain);
        }

        const auto results = solver.last_check_results();
        for (std::size_t i = 0; i < results.size() && i < criterion_fire_count.size(); ++i)
        {
            if (results[i].has_value())
            {
                ++criterion_fire_count[i];
            }
        }
        if (solver.status() == cartan::ik_status::converged)
        {
            ++pose_successes;
            total_iterations += static_cast<std::uint64_t>(solver.iterations());
        }
        ++solves;

        benchmark::DoNotOptimize(solver);
    }

    const auto total = std::max<std::uint64_t>(solves, 1);
    const auto converged = std::max<std::uint64_t>(pose_successes, 1);
    state.counters["grad_threshold"] = benchmark::Counter(
        gradient_threshold, benchmark::Counter::kDefaults);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(pose_successes) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["avg_iterations"] = benchmark::Counter(
        static_cast<double>(total_iterations) / static_cast<double>(converged),
        benchmark::Counter::kDefaults);
    state.counters["grad_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[0]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["obj_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[1]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["step_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[2]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["stall_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[3]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["total_solves"] = benchmark::Counter(
        static_cast<double>(total), benchmark::Counter::kDefaults);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_grad_sweep)
    ->Arg(6)->Arg(8)->Arg(10)->Arg(12)
    ->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Budget-per-step sweep on UR3e SLSQP with cartan-tight default_
// convergence, direct-drive (no runner, no restart wrapper).
// state.range(0) is `budget_per_step` (nablapp `max_iterations` per
// cartan-outer step). Holds gradient_threshold = 1e-12 and cartan-
// tight objective/step thresholds so only the inner iteration budget
// moves. Emits wall per solve, direct-drive pose-tolerance success
// rate, avg nablapp-inner-steps of converged solves, cartan-outer
// iteration count (from solver.iterations()), and the last_check_
// results firing profile at each budget.
//
// Falsification test for the iteration-budget-exhaustion hypothesis:
// if 80%+ of UR3e poses currently fail direct-drive because they are
// not converging to pose tolerance within ~37 inner steps, raising
// budget_per_step from 500 → 1000 → 2000 → 4000 should raise direct-
// drive success rate. If success is flat across the sweep, the
// budget is not the exit reason and H1 is falsified.
static void bm_comparison_ur3e_nablapp_slsqp_budget_sweep(benchmark::State& state)
{
    const int budget_per_step = static_cast<int>(state.range(0));

    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    cartan::ik::argmin_slsqp<chain_t<6>>::options slsqp_opts{};
    slsqp_opts.budget_per_step = budget_per_step;
    slsqp_opts.gradient_threshold = 1e-12;

    std::size_t idx = 0;
    std::array<std::uint64_t, 4> criterion_fire_count{};
    std::uint64_t solves = 0;
    std::uint64_t pose_successes = 0;
    std::uint64_t total_outer_iterations = 0;
    std::uint64_t total_outer_iter_converged = 0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::ik::argmin_slsqp<chain_t<6>> solver{slsqp_opts};
        solver.setup(chain, target, q_seed, criteria);

        while (solver.status() == cartan::ik_status::running)
        {
            solver.step(chain);
        }

        const auto results = solver.last_check_results();
        for (std::size_t i = 0; i < results.size() && i < criterion_fire_count.size(); ++i)
        {
            if (results[i].has_value())
            {
                ++criterion_fire_count[i];
            }
        }
        total_outer_iterations += static_cast<std::uint64_t>(solver.iterations());
        if (solver.status() == cartan::ik_status::converged)
        {
            ++pose_successes;
            total_outer_iter_converged += static_cast<std::uint64_t>(solver.iterations());
        }
        ++solves;

        benchmark::DoNotOptimize(solver);
    }

    const auto total = std::max<std::uint64_t>(solves, 1);
    const auto converged = std::max<std::uint64_t>(pose_successes, 1);
    state.counters["budget_per_step"] = benchmark::Counter(
        static_cast<double>(budget_per_step), benchmark::Counter::kDefaults);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(pose_successes) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["avg_outer_iter_converged"] = benchmark::Counter(
        static_cast<double>(total_outer_iter_converged) / static_cast<double>(converged),
        benchmark::Counter::kDefaults);
    state.counters["avg_outer_iter_all"] = benchmark::Counter(
        static_cast<double>(total_outer_iterations) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["grad_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[0]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["obj_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[1]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["step_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[2]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["stall_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[3]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["total_solves"] = benchmark::Counter(
        static_cast<double>(total), benchmark::Counter::kDefaults);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_budget_sweep)
    ->Arg(500)->Arg(1000)->Arg(2000)->Arg(4000)
    ->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_nablapp_slsqp_retry(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    const auto restart_scale = static_cast<double>(state.range(0)) / 100.0;

    cartan::ik::argmin_slsqp<chain_t<6>>::options slsqp_opts{};
    slsqp_opts.max_restarts = 10;
    slsqp_opts.restart_scale = restart_scale;

    std::size_t idx = 0;
    std::uint64_t solves = 0;
    std::uint64_t pose_successes = 0;
    std::uint64_t total_restarts = 0;
    std::uint64_t total_restarts_converged = 0;
    std::uint64_t total_outer_iterations = 0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::ik::argmin_slsqp<chain_t<6>> solver{slsqp_opts};
        solver.setup(chain, target, q_seed, criteria);

        while (solver.status() == cartan::ik_status::running)
        {
            solver.step(chain);
        }

        total_restarts += static_cast<std::uint64_t>(solver.restart_count());
        total_outer_iterations += static_cast<std::uint64_t>(solver.iterations());

        if (solver.status() == cartan::ik_status::converged)
        {
            ++pose_successes;
            total_restarts_converged += static_cast<std::uint64_t>(solver.restart_count());
        }
        ++solves;

        benchmark::DoNotOptimize(solver);
    }

    const auto total = std::max<std::uint64_t>(solves, 1);
    const auto converged = std::max<std::uint64_t>(pose_successes, 1);
    state.counters["restart_scale"] = benchmark::Counter(
        restart_scale, benchmark::Counter::kDefaults);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(pose_successes) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["avg_restarts_all"] = benchmark::Counter(
        static_cast<double>(total_restarts) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["avg_restarts_converged"] = benchmark::Counter(
        static_cast<double>(total_restarts_converged) / static_cast<double>(converged),
        benchmark::Counter::kDefaults);
    state.counters["avg_outer_iter_all"] = benchmark::Counter(
        static_cast<double>(total_outer_iterations) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["total_solves"] = benchmark::Counter(
        static_cast<double>(total), benchmark::Counter::kDefaults);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_retry)
    ->Arg(10)->Arg(30)->Arg(50)->Arg(70)->Arg(100)
    ->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Alias-path companion to bm_comparison_ur3e_nablapp_slsqp_last_check_results.
// Same direct-drive pattern, but instantiates argmin_slsqp_nlopt_compat
// (nablapp::slsqp_compatible_convergence: 3 criteria — ftol_rel, xtol_rel,
// stall_tolerance). Tests the alias-mapping hypothesis: which default_
// convergence firing slot each alias criterion inherits when the alias
// replaces absolute thresholds with relative ones at 1e-10.
static void bm_comparison_ur3e_nablapp_slsqp_last_check_results_alias(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    cartan::ik::argmin_slsqp_nlopt_compat<chain_t<6>>::options slsqp_opts{};
    slsqp_opts.budget_per_step = 500;

    std::size_t idx = 0;
    std::array<std::uint64_t, 3> criterion_fire_count{};
    std::uint64_t solves = 0;
    std::uint64_t pose_successes = 0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::ik::argmin_slsqp_nlopt_compat<chain_t<6>> solver{slsqp_opts};
        solver.setup(chain, target, q_seed, criteria);

        while (solver.status() == cartan::ik_status::running)
        {
            solver.step(chain);
        }

        const auto results = solver.last_check_results();
        for (std::size_t i = 0; i < results.size() && i < criterion_fire_count.size(); ++i)
        {
            if (results[i].has_value())
            {
                ++criterion_fire_count[i];
            }
        }
        if (solver.status() == cartan::ik_status::converged)
        {
            ++pose_successes;
        }
        ++solves;

        benchmark::DoNotOptimize(solver);
    }

    const auto total = std::max<std::uint64_t>(solves, 1);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(pose_successes) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["ftol_rel_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[0]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["xtol_rel_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[1]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["stall_tol_fire_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(criterion_fire_count[2]) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["total_solves"] = benchmark::Counter(
        static_cast<double>(total), benchmark::Counter::kDefaults);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_last_check_results_alias)
    ->Iterations(1000)->Unit(benchmark::kMicrosecond);

#ifdef CARTAN_HAS_NLOPT
// Direct-drive NLopt SLSQP bench that also captures nlopt's per-pose
// objective function call count (via the nlopt_objective_calls accessor
// on cartan::ik::nlopt_slsqp). Purpose: indirect per-inner-iteration
// wall measurement for nlopt on UR3e, to compare against nablapp's
// ~11.4 us/inner-step figure. Same 1000 UR3e poses, seed 42, cartan-
// tight criteria, direct-drive (no restart_wrapper, no basic_ik_runner)
// to expose the native nlopt counter without wrapper interference.
//
// `nlopt_iter_count` in the counter output is the cumulative count of
// objective callback invocations divided by total solves = average
// nlopt inner iterations per pose. Each invocation corresponds to one
// nlopt SLSQP inner iteration (one value-only or value+gradient call).
// nlopt dispatches value-only and gradient through the same callback
// and branches on `grad.empty()`, so this is a unified counter.
static void bm_comparison_ur3e_nlopt_slsqp_inner_iter_count(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    cartan::ik::nlopt_slsqp<chain_t<6>>::options slsqp_opts{};
    slsqp_opts.budget_per_step = 500;

    std::size_t idx = 0;
    std::uint64_t total_calls = 0;
    std::uint64_t solves = 0;
    std::uint64_t pose_successes = 0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::ik::nlopt_slsqp<chain_t<6>> solver{slsqp_opts};
        solver.setup(chain, target, q_seed, criteria);

        while (solver.status() == cartan::ik_status::running)
        {
            solver.step(chain);
        }

        total_calls += solver.nlopt_objective_calls();
        if (solver.status() == cartan::ik_status::converged)
        {
            ++pose_successes;
        }
        ++solves;

        benchmark::DoNotOptimize(solver);
    }

    const auto total = std::max<std::uint64_t>(solves, 1);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(pose_successes) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["nlopt_iter_count"] = benchmark::Counter(
        static_cast<double>(total_calls) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["total_solves"] = benchmark::Counter(
        static_cast<double>(total), benchmark::Counter::kDefaults);
}
BENCHMARK(bm_comparison_ur3e_nlopt_slsqp_inner_iter_count)
    ->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Direct-drive NLopt SLSQP bench that captures how often the cartan-
// side `needs_restart` perturbation loop fires per pose during a
// single cartan-outer solve. The loop is wrapped around nlopt (see
// nlopt_slsqp::step line 146-161): when nlopt returns SUCCESS/FTOL/
// XTOL without pose-tolerance convergence (or MAXEVAL with stalled
// error), cartan perturbs the seed and re-runs optimize. Capped by
// options::max_restarts = 10.
//
// Falsification test for the hypothesis that nlopt's per-pose direct-
// drive success rate (65.6% on UR3e) is mostly sourced from this
// perturb-and-retry resilience rather than from one-shot convergence.
// Threshold: if the mean restart_count per pose is >= 2.0 the
// hypothesis is live; if it is < 0.5 the hypothesis is falsified and
// nlopt is finding pose-feasible minima on the first optimize() call.
//
// Note on framing: this is a cartan-side perturbation wrapper around
// nlopt, not an nlopt-internal feature. nlopt itself has no
// `max_restarts` parameter — the `cartan::detail::needs_restart`
// predicate and `perturb_nlopt_solution` call together form the retry
// loop.
static void bm_comparison_ur3e_nlopt_slsqp_restart_count(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    cartan::ik::nlopt_slsqp<chain_t<6>>::options slsqp_opts{};
    slsqp_opts.budget_per_step = 500;

    std::size_t idx = 0;
    std::uint64_t total_restarts = 0;
    std::uint64_t solves = 0;
    std::uint64_t pose_successes = 0;
    std::uint64_t poses_with_zero_restarts = 0;
    std::uint64_t poses_with_at_least_one_restart = 0;
    std::uint64_t successes_with_zero_restarts = 0;
    int max_restarts_seen = 0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::ik::nlopt_slsqp<chain_t<6>> solver{slsqp_opts};
        solver.setup(chain, target, q_seed, criteria);

        while (solver.status() == cartan::ik_status::running)
        {
            solver.step(chain);
        }

        const int rc = solver.nlopt_restart_count();
        total_restarts += static_cast<std::uint64_t>(rc);
        if (rc == 0)
        {
            ++poses_with_zero_restarts;
        }
        else
        {
            ++poses_with_at_least_one_restart;
        }
        if (rc > max_restarts_seen)
        {
            max_restarts_seen = rc;
        }
        if (solver.status() == cartan::ik_status::converged)
        {
            ++pose_successes;
            if (rc == 0)
            {
                ++successes_with_zero_restarts;
            }
        }
        ++solves;

        benchmark::DoNotOptimize(solver);
    }

    const auto total = std::max<std::uint64_t>(solves, 1);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(pose_successes) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["avg_restarts_per_pose"] = benchmark::Counter(
        static_cast<double>(total_restarts) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["zero_restart_pose_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(poses_with_zero_restarts) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["any_restart_pose_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(poses_with_at_least_one_restart) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["zero_restart_success_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(successes_with_zero_restarts) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["max_restart_count"] = benchmark::Counter(
        static_cast<double>(max_restarts_seen),
        benchmark::Counter::kDefaults);
    state.counters["total_solves"] = benchmark::Counter(
        static_cast<double>(total), benchmark::Counter::kDefaults);
}
BENCHMARK(bm_comparison_ur3e_nlopt_slsqp_restart_count)
    ->Iterations(1000)->Unit(benchmark::kMicrosecond);
#endif

// Runner-wrapped companion to bm_comparison_ur3e_nablapp_slsqp_grad_sweep.
// Replicates bm_comparison_ur3e_nablapp_slsqp (default_convergence, restart
// wrapper, basic_ik_runner) but preconfigures argmin_slsqp with a custom
// gradient_threshold so wall and success rate are directly comparable to
// the 2554 us / 91.3% baseline at cartan-tight 1e-12. This is the bench
// that answers "does relaxing gradient_threshold recover wall without
// losing runner-wrapped success"; the direct-drive sweep above only
// observes one nablapp inner pass per pose and does not exercise the
// restart retry loop that the published 91.3% success rate relies on.
static void bm_comparison_ur3e_nablapp_slsqp_grad_sweep_runner(benchmark::State& state)
{
    const int exponent = static_cast<int>(state.range(0));
    const double gradient_threshold = std::pow(10.0, -static_cast<double>(exponent));

    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    cartan::ik::argmin_slsqp<chain_t<6>>::options inner_opts{};
    inner_opts.gradient_threshold = gradient_threshold;

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

        cartan::ik::argmin_slsqp<chain_t<6>> inner{inner_opts};
        cartan::ik::restart_wrapper<chain_t<6>, cartan::ik::argmin_slsqp<chain_t<6>>>
            wrapper{std::move(inner)};
        nablapp_slsqp_solver<6> solver{std::move(wrapper)};

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

    const auto total = std::max<int>(static_cast<int>(idx), 1);
    state.counters["grad_threshold"] = benchmark::Counter(
        gradient_threshold, benchmark::Counter::kDefaults);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / static_cast<double>(total),
        benchmark::Counter::kDefaults);
    state.counters["avg_iterations"] = benchmark::Counter(
        static_cast<double>(total_iterations) / std::max(successes, 1),
        benchmark::Counter::kDefaults);
    state.counters["avg_position_error"] = benchmark::Counter(
        total_pos_error / std::max(successes, 1),
        benchmark::Counter::kDefaults);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_ori_error / std::max(successes, 1),
        benchmark::Counter::kDefaults);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_grad_sweep_runner)
    ->Arg(6)->Arg(8)->Arg(10)->Arg(12)
    ->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_nablapp_slsqp_phi_ls_calls_dynamicN(benchmark::State& state)
{
    using dynamic_chain = cartan::kinematic_chain<double, cartan::dynamic>;

    auto chain = cartan::benchmarks::make_ur3e_chain_dynamic<double>();
    static const target_set<double, cartan::dynamic> ts(chain, num_targets, 42);
    const auto criteria = nablapp_comparison_criteria();

    cartan::ik::argmin_slsqp<dynamic_chain>::options slsqp_opts{};
    slsqp_opts.budget_per_step = 500;

    std::size_t idx = 0;
    std::uint64_t total_ls = 0;
    std::uint64_t total_nablapp_steps = 0;
    int total_cartan_outer = 0;
    int successes = 0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::ik::argmin_slsqp<dynamic_chain> solver{slsqp_opts};
        solver.setup(chain, target, q_seed, criteria);

        while (solver.status() == cartan::ik_status::running)
        {
            solver.step(chain);
        }

        total_ls += solver.line_search_calls();
        total_nablapp_steps += static_cast<std::uint64_t>(solver.nablapp_iterations());
        total_cartan_outer += solver.iterations();
        if (solver.status() == cartan::ik_status::converged)
            ++successes;

        benchmark::DoNotOptimize(solver);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["total_phi_ls_calls"] = benchmark::Counter(
        static_cast<double>(total_ls), benchmark::Counter::kDefaults);
    state.counters["total_nablapp_steps"] = benchmark::Counter(
        static_cast<double>(total_nablapp_steps), benchmark::Counter::kDefaults);
    state.counters["phi_ls_per_nablapp_step"] = benchmark::Counter(
        static_cast<double>(total_ls) / std::max<double>(1.0, static_cast<double>(total_nablapp_steps)),
        benchmark::Counter::kDefaults);
    state.counters["avg_nablapp_step_per_pose"] = benchmark::Counter(
        static_cast<double>(total_nablapp_steps) / std::max(total, 1),
        benchmark::Counter::kDefaults);
    state.counters["avg_cartan_outer_iter"] = benchmark::Counter(
        static_cast<double>(total_cartan_outer) / std::max(total, 1),
        benchmark::Counter::kDefaults);
}
BENCHMARK(bm_comparison_ur3e_nablapp_slsqp_phi_ls_calls_dynamicN)->Iterations(1000)->Unit(benchmark::kMicrosecond);

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

// ============================================================================
// Per-call micro benchmarks — interprets the turn-04 perf roll-up
// ============================================================================
//
// Measures the raw per-call cost of the four kernels that together account
// for ~41% of nablapp_slsqp wall in cartan turn-04:
//
//   forward_kinematics(chain, q)
//   ik_se3_objective::evaluate(chain, target, q)
//   ik_se3_objective::evaluate_with_gradient(chain, target, q)
//   nablapp_ik_problem::value(x)            [adapter wrapping evaluate]
//   nablapp_ik_problem::gradient(x, g)      [adapter wrapping evaluate_with_gradient]
//
// Both solvers go through `ik_se3_objective`, so a per-call gap between
// the plain evaluate and the nablapp adapter path indicates pure adapter
// overhead (position_type conversion, etc.). A per-call gap between
// evaluate and evaluate_with_gradient measures the cost of the SE(3)
// log Jacobian contribution on top of the objective.
//
// Target and seed are fixed (the first UR3e target from the same seed=42
// set used by bm_comparison_ur3e_*) so run-to-run variance comes purely
// from the kernel, not the workload.

struct ur3e_per_call_fixture
{
    ur3e_per_call_fixture()
        : chain{cartan::benchmarks::make_ur3e_chain<double>()}
        , ts{chain, num_targets, 42}
        , problem{chain, ts.targets[0], cartan::error_weight<double>{}}
    {
        x = Eigen::Vector<double, 6>{};
        for (int i = 0; i < 6; ++i)
        {
            x[i] = ts.seeds[0][i];
        }
        g = Eigen::Vector<double, 6>::Zero();
    }

    cartan::kinematic_chain<double, 6> chain;
    target_set<double, 6> ts;
    cartan::detail::nablapp_ik_problem<chain_t<6>> problem;
    Eigen::Vector<double, 6> x;
    Eigen::Vector<double, 6> g;
};

inline ur3e_per_call_fixture& ur3e_per_call()
{
    static ur3e_per_call_fixture instance;
    return instance;
}

static void bm_ur3e_per_call_forward_kinematics(benchmark::State& state)
{
    auto& f = ur3e_per_call();
    auto q = f.problem.to_position(f.x);
    for (auto _ : state)
    {
        auto fk = cartan::forward_kinematics(f.chain, q);
        benchmark::DoNotOptimize(fk);
    }
}
BENCHMARK(bm_ur3e_per_call_forward_kinematics)->Unit(benchmark::kNanosecond);

static void bm_ur3e_per_call_ik_se3_objective_evaluate(benchmark::State& state)
{
    auto& f = ur3e_per_call();
    auto q = f.problem.to_position(f.x);
    cartan::error_weight<double> w{};
    for (auto _ : state)
    {
        auto result = cartan::ik_se3_objective<chain_t<6>>::evaluate(
            f.chain, f.ts.targets[0], q, w);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ur3e_per_call_ik_se3_objective_evaluate)->Unit(benchmark::kNanosecond);

static void bm_ur3e_per_call_ik_se3_objective_evaluate_with_gradient(benchmark::State& state)
{
    auto& f = ur3e_per_call();
    auto q = f.problem.to_position(f.x);
    cartan::error_weight<double> w{};
    for (auto _ : state)
    {
        auto result = cartan::ik_se3_objective<chain_t<6>>::evaluate_with_gradient(
            f.chain, f.ts.targets[0], q, w);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ur3e_per_call_ik_se3_objective_evaluate_with_gradient)->Unit(benchmark::kNanosecond);

static void bm_ur3e_per_call_nablapp_adapter_value(benchmark::State& state)
{
    auto& f = ur3e_per_call();
    for (auto _ : state)
    {
        double v = f.problem.value(f.x);
        benchmark::DoNotOptimize(v);
    }
}
BENCHMARK(bm_ur3e_per_call_nablapp_adapter_value)->Unit(benchmark::kNanosecond);

static void bm_ur3e_per_call_nablapp_adapter_gradient(benchmark::State& state)
{
    auto& f = ur3e_per_call();
    for (auto _ : state)
    {
        f.problem.gradient(f.x, f.g);
        benchmark::DoNotOptimize(f.g);
    }
}
BENCHMARK(bm_ur3e_per_call_nablapp_adapter_gradient)->Unit(benchmark::kNanosecond);

} // anonymous namespace
