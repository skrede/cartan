#include "benchmark_utils.h"

#include <cartan/serial/fk/forward_kinematics.h>

#include <benchmark/benchmark.h>

#include <random>

// ===========================================================================
// FK benchmarks: 3-DOF fixed, 6-DOF fixed+dynamic, 7-DOF fixed+dynamic
// ===========================================================================

static void bm_fk_3r_planar(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_3r_planar_chain<double>();
    std::mt19937 rng(42);
    auto q = cartan::benchmarks::random_joint_config(chain, rng);

    for (auto _ : state)
    {
        auto result = cartan::forward_kinematics(chain, q);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_fk_3r_planar);

static void bm_fk_ur3e_fixed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain<double>();
    std::mt19937 rng(42);
    auto q = cartan::benchmarks::random_joint_config(chain, rng);

    for (auto _ : state)
    {
        auto result = cartan::forward_kinematics(chain, q);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_fk_ur3e_fixed);

static void bm_fk_ur3e_dynamic(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_ur3e_chain_dynamic<double>();
    std::mt19937 rng(42);
    auto q = cartan::benchmarks::random_joint_config(chain, rng);

    for (auto _ : state)
    {
        auto result = cartan::forward_kinematics(chain, q);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_fk_ur3e_dynamic);

static void bm_fk_lbr_med14_fixed(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_lbr_med14_chain<double>();
    std::mt19937 rng(42);
    auto q = cartan::benchmarks::random_joint_config(chain, rng);

    for (auto _ : state)
    {
        auto result = cartan::forward_kinematics(chain, q);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_fk_lbr_med14_fixed);

static void bm_fk_lbr_med14_dynamic(benchmark::State& state)
{
    auto chain = cartan::benchmarks::make_lbr_med14_chain_dynamic<double>();
    std::mt19937 rng(42);
    auto q = cartan::benchmarks::random_joint_config(chain, rng);

    for (auto _ : state)
    {
        auto result = cartan::forward_kinematics(chain, q);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_fk_lbr_med14_dynamic);
