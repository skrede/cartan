/// @file ik_comparison_benchmarks.cpp
/// @brief Head-to-head comparison of liepp IK vs TRAC-IK for ~9 robots.
///
/// Both solvers use the same random seed (42), same 10,000 targets (FK-generated,
/// guaranteed reachable), and same convergence tolerance (eps = 1e-5).
/// TRAC-IK uses maxtime=10.0 for convergence-based termination (Pitfall 3).
/// liepp uses LM stepper (closest algorithmic match to TRAC-IK's Newton methods).
///
/// Reference: Beeson, P., & Ames, B. (2015). TRAC-IK: An open-source library
///            for improved solving of generic inverse kinematics.

#include "benchmark_utils.h"

#include <liepp/ik/basic_ik_solver.h>
#include <liepp/ik/lm_solve_policy.h>
#include <liepp/ik/ik_types.h>

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

/// Pre-generate liepp targets and seeds for a given chain.
template <typename Scalar, int N>
struct target_set
{
    using position_type = typename liepp::joint_state<Scalar, N>::position_type;

    std::vector<liepp::se3<Scalar>> targets;
    std::vector<position_type> seeds;

    target_set(const liepp::kinematic_chain<Scalar, N>& chain, int count, unsigned seed = 42)
    {
        std::mt19937 rng(seed);
        targets.reserve(static_cast<std::size_t>(count));
        seeds.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i)
        {
            targets.push_back(liepp::benchmarks::random_reachable_target(chain, rng));
            seeds.push_back(liepp::benchmarks::random_joint_config(chain, rng));
        }
    }
};

/// liepp LM benchmark for a fixed-N chain.
template <int N>
void bm_liepp_comparison(
    benchmark::State& state,
    const liepp::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts)
{
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 100};

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

        liepp::basic_ik_solver<liepp::lm_solve_policy<double, N>> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iterations += result->iterations;
            auto [pos_err, ori_err] = liepp::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos_error += pos_err;
            total_ori_error += ori_err;
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
    // Convert liepp targets to KDL frames and seeds to KDL JntArrays
    auto count = static_cast<std::size_t>(num_targets);
    std::vector<KDL::Frame> kdl_targets;
    std::vector<KDL::JntArray> kdl_seeds;
    kdl_targets.reserve(count);
    kdl_seeds.reserve(count);

    for (std::size_t i = 0; i < count; ++i)
    {
        kdl_targets.push_back(liepp::benchmarks::se3_to_kdl_frame(ts.targets[i]));
        KDL::JntArray kdl_seed(static_cast<unsigned int>(N));
        for (unsigned int j = 0; j < static_cast<unsigned int>(N); ++j)
        {
            kdl_seed(j) = ts.seeds[i](static_cast<int>(j));
        }
        kdl_seeds.push_back(kdl_seed);
    }

    // TRAC-IK with large timeout for convergence-based termination (Pitfall 3)
    // eps=1e-5 matches liepp convergence tolerance
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
    state.counters["success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
}

// ============================================================================
// 6-DOF robots
// ============================================================================

// --- UR3e ---

static void bm_comparison_ur3e_liepp(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_liepp_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_ur3e_liepp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_ur3e_trac_ik(benchmark::State& state)
{
    auto liepp_chain = liepp::benchmarks::make_ur3e_chain<double>();
    auto kdl_chain = liepp::benchmarks::make_ur3e_kdl_chain();
    KDL::JntArray q_min(6), q_max(6);
    liepp::benchmarks::make_ur3e_kdl_limits(q_min, q_max);
    static const target_set<double, 6> ts(liepp_chain, num_targets, 42);
    bm_trac_ik_comparison<6>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_ur3e_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- KR 6 SIXX ---

static void bm_comparison_kr6_sixx_liepp(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_kr6_sixx_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_liepp_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_kr6_sixx_liepp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kr6_sixx_trac_ik(benchmark::State& state)
{
    auto liepp_chain = liepp::benchmarks::make_kr6_sixx_chain<double>();
    auto kdl_chain = liepp::benchmarks::make_kr6_sixx_kdl_chain();
    KDL::JntArray q_min(6), q_max(6);
    liepp::benchmarks::make_kr6_sixx_kdl_limits(q_min, q_max);
    static const target_set<double, 6> ts(liepp_chain, num_targets, 42);
    bm_trac_ik_comparison<6>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_kr6_sixx_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- ABB IRB 120 ---

static void bm_comparison_abb_irb120_liepp(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_abb_irb120_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_liepp_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_abb_irb120_liepp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_abb_irb120_trac_ik(benchmark::State& state)
{
    auto liepp_chain = liepp::benchmarks::make_abb_irb120_chain<double>();
    auto kdl_chain = liepp::benchmarks::make_abb_irb120_kdl_chain();
    KDL::JntArray q_min(6), q_max(6);
    liepp::benchmarks::make_abb_irb120_kdl_limits(q_min, q_max);
    static const target_set<double, 6> ts(liepp_chain, num_targets, 42);
    bm_trac_ik_comparison<6>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_abb_irb120_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- Jaco2 ---

static void bm_comparison_jaco2_liepp(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_jaco2_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_liepp_comparison<6>(state, chain, ts);
}
BENCHMARK(bm_comparison_jaco2_liepp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_jaco2_trac_ik(benchmark::State& state)
{
    auto liepp_chain = liepp::benchmarks::make_jaco2_chain<double>();
    auto kdl_chain = liepp::benchmarks::make_jaco2_kdl_chain();
    KDL::JntArray q_min(6), q_max(6);
    liepp::benchmarks::make_jaco2_kdl_limits(q_min, q_max);
    static const target_set<double, 6> ts(liepp_chain, num_targets, 42);
    bm_trac_ik_comparison<6>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_jaco2_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// ============================================================================
// 7-DOF robots
// ============================================================================

// --- LBR Med 14 ---

static void bm_comparison_lbr_med14_liepp(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_lbr_med14_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_liepp_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_lbr_med14_liepp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_lbr_med14_trac_ik(benchmark::State& state)
{
    auto liepp_chain = liepp::benchmarks::make_lbr_med14_chain<double>();
    auto kdl_chain = liepp::benchmarks::make_lbr_med14_kdl_chain();
    KDL::JntArray q_min(7), q_max(7);
    liepp::benchmarks::make_lbr_med14_kdl_limits(q_min, q_max);
    static const target_set<double, 7> ts(liepp_chain, num_targets, 42);
    bm_trac_ik_comparison<7>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_lbr_med14_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- Panda ---

static void bm_comparison_panda_liepp(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_panda_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_liepp_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_panda_liepp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_panda_trac_ik(benchmark::State& state)
{
    auto liepp_chain = liepp::benchmarks::make_panda_chain<double>();
    auto kdl_chain = liepp::benchmarks::make_panda_kdl_chain();
    KDL::JntArray q_min(7), q_max(7);
    liepp::benchmarks::make_panda_kdl_limits(q_min, q_max);
    static const target_set<double, 7> ts(liepp_chain, num_targets, 42);
    bm_trac_ik_comparison<7>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_panda_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- Fetch ---

static void bm_comparison_fetch_liepp(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_fetch_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_liepp_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_fetch_liepp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_fetch_trac_ik(benchmark::State& state)
{
    auto liepp_chain = liepp::benchmarks::make_fetch_chain<double>();
    auto kdl_chain = liepp::benchmarks::make_fetch_kdl_chain();
    KDL::JntArray q_min(7), q_max(7);
    liepp::benchmarks::make_fetch_kdl_limits(q_min, q_max);
    static const target_set<double, 7> ts(liepp_chain, num_targets, 42);
    bm_trac_ik_comparison<7>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_fetch_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- Baxter ---

static void bm_comparison_baxter_liepp(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_baxter_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_liepp_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_baxter_liepp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_baxter_trac_ik(benchmark::State& state)
{
    auto liepp_chain = liepp::benchmarks::make_baxter_chain<double>();
    auto kdl_chain = liepp::benchmarks::make_baxter_kdl_chain();
    KDL::JntArray q_min(7), q_max(7);
    liepp::benchmarks::make_baxter_kdl_limits(q_min, q_max);
    static const target_set<double, 7> ts(liepp_chain, num_targets, 42);
    bm_trac_ik_comparison<7>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_baxter_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// --- KUKA LWR 4+ ---

static void bm_comparison_kuka_lwr4_liepp(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_kuka_lwr4_chain<double>();
    static const target_set<double, 7> ts(chain, num_targets, 42);
    bm_liepp_comparison<7>(state, chain, ts);
}
BENCHMARK(bm_comparison_kuka_lwr4_liepp)->Iterations(1000)->Unit(benchmark::kMicrosecond);

static void bm_comparison_kuka_lwr4_trac_ik(benchmark::State& state)
{
    auto liepp_chain = liepp::benchmarks::make_kuka_lwr4_chain<double>();
    auto kdl_chain = liepp::benchmarks::make_kuka_lwr4_kdl_chain();
    KDL::JntArray q_min(7), q_max(7);
    liepp::benchmarks::make_kuka_lwr4_kdl_limits(q_min, q_max);
    static const target_set<double, 7> ts(liepp_chain, num_targets, 42);
    bm_trac_ik_comparison<7>(state, kdl_chain, q_min, q_max, ts);
}
BENCHMARK(bm_comparison_kuka_lwr4_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

} // anonymous namespace
