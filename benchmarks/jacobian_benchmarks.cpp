#include "benchmark_utils.h"

#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/fk/jacobian.h>

#include <benchmark/benchmark.h>

#include <random>

// ===========================================================================
// Space Jacobian benchmarks
// ===========================================================================

static void bm_space_jacobian_3r_planar(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_3r_planar_chain<double>();
    std::mt19937 rng(42);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);

    for (auto _ : state)
    {
        auto J = cartan::space_jacobian(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_space_jacobian_3r_planar);

static void bm_space_jacobian_ur3e_fixed(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_ur3e_chain<double>();
    std::mt19937 rng(42);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);

    for (auto _ : state)
    {
        auto J = cartan::space_jacobian(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_space_jacobian_ur3e_fixed);

static void bm_space_jacobian_ur3e_dynamic(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_ur3e_chain_dynamic<double>();
    std::mt19937 rng(42);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);

    for (auto _ : state)
    {
        auto J = cartan::space_jacobian(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_space_jacobian_ur3e_dynamic);

static void bm_space_jacobian_lbr_med14_fixed(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_lbr_med14_chain<double>();
    std::mt19937 rng(42);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);

    for (auto _ : state)
    {
        auto J = cartan::space_jacobian(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_space_jacobian_lbr_med14_fixed);

static void bm_space_jacobian_lbr_med14_dynamic(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_lbr_med14_chain_dynamic<double>();
    std::mt19937 rng(42);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);

    for (auto _ : state)
    {
        auto J = cartan::space_jacobian(chain, fk);
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
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);

    for (auto _ : state)
    {
        auto J = cartan::body_jacobian(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_body_jacobian_3r_planar);

static void bm_body_jacobian_ur3e_fixed(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_ur3e_chain<double>();
    std::mt19937 rng(42);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);

    for (auto _ : state)
    {
        auto J = cartan::body_jacobian(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_body_jacobian_ur3e_fixed);

static void bm_body_jacobian_ur3e_dynamic(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_ur3e_chain_dynamic<double>();
    std::mt19937 rng(42);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);

    for (auto _ : state)
    {
        auto J = cartan::body_jacobian(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_body_jacobian_ur3e_dynamic);

static void bm_body_jacobian_lbr_med14_fixed(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_lbr_med14_chain<double>();
    std::mt19937 rng(42);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);

    for (auto _ : state)
    {
        auto J = cartan::body_jacobian(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_body_jacobian_lbr_med14_fixed);

static void bm_body_jacobian_lbr_med14_dynamic(benchmark::State& state)
{
    auto chain = cartan::fixtures::make_lbr_med14_chain_dynamic<double>();
    std::mt19937 rng(42);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);

    for (auto _ : state)
    {
        auto J = cartan::body_jacobian(chain, fk);
        benchmark::DoNotOptimize(J.data());
    }
}
BENCHMARK(bm_body_jacobian_lbr_med14_dynamic);
