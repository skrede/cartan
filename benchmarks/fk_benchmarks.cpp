#include "benchmark_utils.h"

#include <cartan/serial/fk/forward_kinematics.h>

#include <benchmark/benchmark.h>

#include <array>
#include <random>
#include <cstddef>

// ===========================================================================
// FK benchmarks: 3-DOF fixed, 6-DOF fixed+dynamic, 7-DOF fixed+dynamic
// ===========================================================================

namespace
{

// A single loop-invariant q lets the optimizer hoist FK out of the timed loop.
// Each cell draws from a table of varied configs indexed by the iteration
// counter and DoNotOptimize's the chosen q before the call. Power-of-two size
// wraps the index with a mask.
constexpr std::size_t kInputs = 1024;

}

static void bm_fk_3r_planar(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_3r_planar_chain<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs;
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& q = qs[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(q);
        auto result = cartan::forward_kinematics(chain, q);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_fk_3r_planar);

static void bm_fk_ur3e_fixed(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_ur3e_chain<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs;
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& q = qs[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(q);
        auto result = cartan::forward_kinematics(chain, q);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_fk_ur3e_fixed);

static void bm_fk_ur3e_dynamic(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_ur3e_chain_dynamic<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs;
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& q = qs[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(q);
        auto result = cartan::forward_kinematics(chain, q);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_fk_ur3e_dynamic);

static void bm_fk_lbr_med14_fixed(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_lbr_med14_chain<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs;
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& q = qs[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(q);
        auto result = cartan::forward_kinematics(chain, q);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_fk_lbr_med14_fixed);

static void bm_fk_lbr_med14_dynamic(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_lbr_med14_chain_dynamic<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs;
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& q = qs[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(q);
        auto result = cartan::forward_kinematics(chain, q);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_fk_lbr_med14_dynamic);
