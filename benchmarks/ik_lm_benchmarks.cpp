/// @file ik_lm_benchmarks.cpp
/// @brief LM stepper IK benchmarks for 3/6/7-DOF chains.
///
/// Reports custom counters: success_rate, avg_iterations,
/// avg_position_error, avg_orientation_error per D-08.

#include <liepp/serial/ik/basic_ik_solver.h>
#include <liepp/serial/ik/lm_solve_policy.h>
#include <liepp/serial/ik/ik_types.h>

#include "benchmark_utils.h"

#include <benchmark/benchmark.h>

#include <random>
#include <algorithm>

static void bm_ik_lm_3r_planar(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_3r_planar_chain<double>();
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 100};
    std::mt19937 rng(42);

    int total_solves = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_pos_error = 0.0;
    double total_ori_error = 0.0;

    for (auto _ : state)
    {
        auto target = liepp::benchmarks::random_reachable_target(chain, rng);
        auto q_seed = liepp::benchmarks::random_joint_config(chain, rng);

        liepp::basic_ik_solver<liepp::lm_solve_policy<double, 3>> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        ++total_solves;
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

    state.counters["success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total_solves, 1),
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
BENCHMARK(bm_ik_lm_3r_planar)->Iterations(10000)->Unit(benchmark::kMicrosecond);

static void bm_ik_lm_ur3e(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_ur3e_chain<double>();
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 100};
    std::mt19937 rng(42);

    int total_solves = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_pos_error = 0.0;
    double total_ori_error = 0.0;

    for (auto _ : state)
    {
        auto target = liepp::benchmarks::random_reachable_target(chain, rng);
        auto q_seed = liepp::benchmarks::random_joint_config(chain, rng);

        liepp::basic_ik_solver<liepp::lm_solve_policy<double, 6>> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        ++total_solves;
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

    state.counters["success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total_solves, 1),
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
BENCHMARK(bm_ik_lm_ur3e)->Iterations(10000)->Unit(benchmark::kMicrosecond);

static void bm_ik_lm_lbr_med14(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_lbr_med14_chain<double>();
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 100};
    std::mt19937 rng(42);

    int total_solves = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_pos_error = 0.0;
    double total_ori_error = 0.0;

    for (auto _ : state)
    {
        auto target = liepp::benchmarks::random_reachable_target(chain, rng);
        auto q_seed = liepp::benchmarks::random_joint_config(chain, rng);

        liepp::basic_ik_solver<liepp::lm_solve_policy<double, 7>> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        ++total_solves;
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

    state.counters["success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total_solves, 1),
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
BENCHMARK(bm_ik_lm_lbr_med14)->Iterations(10000)->Unit(benchmark::kMicrosecond);
