#include "checked_cache.h"
#include "benchmark_utils.h"

#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/fk/jacobian.h>

#include <benchmark/benchmark.h>

#include <array>
#include <random>
#include <cstddef>

namespace
{

// A single cached fk lets the optimizer hoist the Jacobian out of the timed
// loop. Each cell draws from a table of fk results built from varied configs,
// indexed by the iteration counter, and DoNotOptimize's the chosen fk before
// the call. Power-of-two size wraps the index with a mask.
constexpr std::size_t kInputs = 1024;

}

// ===========================================================================
// Space Jacobian benchmarks
// ===========================================================================

// The timed loop calls the unchecked entry point. The checked one validates the
// cached result's joint count on every call, which inside a timed region
// measures the guard rather than the Jacobian and would shift a published
// number under an unchanged benchmark name. The cache above it is built outside
// the timed region and takes the checked entry point.
static void bm_space_jacobian_3r_planar(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_3r_planar_chain<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::forward_kinematics_unchecked(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks;
    if (!cartan::bench::fill_fk_cache(fks, chain, rng, state)) return;

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& fk = fks[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(fk);
        auto J = cartan::space_jacobian_unchecked(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_space_jacobian_3r_planar);

static void bm_space_jacobian_ur3e_fixed(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_ur3e_chain<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::forward_kinematics_unchecked(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks;
    if (!cartan::bench::fill_fk_cache(fks, chain, rng, state)) return;

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& fk = fks[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(fk);
        auto J = cartan::space_jacobian_unchecked(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_space_jacobian_ur3e_fixed);

static void bm_space_jacobian_ur3e_dynamic(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_ur3e_chain_dynamic<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::forward_kinematics_unchecked(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks;
    if (!cartan::bench::fill_fk_cache(fks, chain, rng, state)) return;

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& fk = fks[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(fk);
        auto J = cartan::space_jacobian_unchecked(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_space_jacobian_ur3e_dynamic);

static void bm_space_jacobian_lbr_med14_fixed(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_lbr_med14_chain<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::forward_kinematics_unchecked(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks;
    if (!cartan::bench::fill_fk_cache(fks, chain, rng, state)) return;

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& fk = fks[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(fk);
        auto J = cartan::space_jacobian_unchecked(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_space_jacobian_lbr_med14_fixed);

static void bm_space_jacobian_lbr_med14_dynamic(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_lbr_med14_chain_dynamic<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::forward_kinematics_unchecked(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks;
    if (!cartan::bench::fill_fk_cache(fks, chain, rng, state)) return;

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& fk = fks[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(fk);
        auto J = cartan::space_jacobian_unchecked(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_space_jacobian_lbr_med14_dynamic);

// ===========================================================================
// Body Jacobian benchmarks
// ===========================================================================

static void bm_body_jacobian_3r_planar(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_3r_planar_chain<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::forward_kinematics_unchecked(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks;
    if (!cartan::bench::fill_fk_cache(fks, chain, rng, state)) return;

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& fk = fks[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(fk);
        auto J = cartan::body_jacobian_unchecked(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_body_jacobian_3r_planar);

static void bm_body_jacobian_ur3e_fixed(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_ur3e_chain<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::forward_kinematics_unchecked(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks;
    if (!cartan::bench::fill_fk_cache(fks, chain, rng, state)) return;

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& fk = fks[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(fk);
        auto J = cartan::body_jacobian_unchecked(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_body_jacobian_ur3e_fixed);

static void bm_body_jacobian_ur3e_dynamic(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_ur3e_chain_dynamic<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::forward_kinematics_unchecked(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks;
    if (!cartan::bench::fill_fk_cache(fks, chain, rng, state)) return;

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& fk = fks[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(fk);
        auto J = cartan::body_jacobian_unchecked(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_body_jacobian_ur3e_dynamic);

static void bm_body_jacobian_lbr_med14_fixed(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_lbr_med14_chain<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::forward_kinematics_unchecked(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks;
    if (!cartan::bench::fill_fk_cache(fks, chain, rng, state)) return;

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& fk = fks[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(fk);
        auto J = cartan::body_jacobian_unchecked(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_body_jacobian_lbr_med14_fixed);

static void bm_body_jacobian_lbr_med14_dynamic(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_lbr_med14_chain_dynamic<double>();
    std::mt19937 rng(42);
    std::array<decltype(cartan::forward_kinematics_unchecked(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks;
    if (!cartan::bench::fill_fk_cache(fks, chain, rng, state)) return;

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& fk = fks[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(fk);
        auto J = cartan::body_jacobian_unchecked(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_body_jacobian_lbr_med14_dynamic);
