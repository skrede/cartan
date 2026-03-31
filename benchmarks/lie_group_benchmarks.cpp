#include <liepp/types.h>
#include <liepp/lie/so2.h>
#include <liepp/lie/se2.h>
#include <liepp/lie/so3.h>
#include <liepp/lie/se3.h>

#include <benchmark/benchmark.h>

// ===========================================================================
// SO(2) benchmarks (4 total)
// ===========================================================================

static void bm_so2_exp(benchmark::State& state)
{
    double theta = 1.234;
    for (auto _ : state)
    {
        auto result = liepp::so2<double>::exp(theta);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so2_exp);

static void bm_so2_log(benchmark::State& state)
{
    auto elem = liepp::so2<double>::exp(1.234);
    for (auto _ : state)
    {
        auto result = elem.log();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so2_log);

static void bm_so2_compose(benchmark::State& state)
{
    auto a = liepp::so2<double>::exp(0.7);
    auto b = liepp::so2<double>::exp(1.3);
    for (auto _ : state)
    {
        auto result = a * b;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so2_compose);

static void bm_so2_inverse(benchmark::State& state)
{
    auto elem = liepp::so2<double>::exp(1.234);
    for (auto _ : state)
    {
        auto result = elem.inverse();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so2_inverse);

// ===========================================================================
// SE(2) benchmarks (5 total)
// ===========================================================================

static void bm_se2_exp(benchmark::State& state)
{
    liepp::vector3<double> twist;
    twist << 0.5, 1.0, -0.3;
    for (auto _ : state)
    {
        auto result = liepp::se2<double>::exp(twist);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se2_exp);

static void bm_se2_log(benchmark::State& state)
{
    liepp::vector3<double> twist;
    twist << 0.5, 1.0, -0.3;
    auto elem = liepp::se2<double>::exp(twist);
    for (auto _ : state)
    {
        auto result = elem.log();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se2_log);

static void bm_se2_compose(benchmark::State& state)
{
    liepp::vector3<double> v1, v2;
    v1 << 0.5, 1.0, -0.3;
    v2 << -0.2, 0.7, 0.9;
    auto a = liepp::se2<double>::exp(v1);
    auto b = liepp::se2<double>::exp(v2);
    for (auto _ : state)
    {
        auto result = a * b;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se2_compose);

static void bm_se2_inverse(benchmark::State& state)
{
    liepp::vector3<double> twist;
    twist << 0.5, 1.0, -0.3;
    auto elem = liepp::se2<double>::exp(twist);
    for (auto _ : state)
    {
        auto result = elem.inverse();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se2_inverse);

static void bm_se2_adjoint(benchmark::State& state)
{
    liepp::vector3<double> twist;
    twist << 0.5, 1.0, -0.3;
    auto elem = liepp::se2<double>::exp(twist);
    for (auto _ : state)
    {
        auto result = elem.adjoint();
        benchmark::DoNotOptimize(result.data());
    }
}
BENCHMARK(bm_se2_adjoint);

// ===========================================================================
// SO(3) benchmarks (6 total)
// ===========================================================================

static void bm_so3_exp(benchmark::State& state)
{
    liepp::vector3<double> phi = liepp::vector3<double>::Random();
    for (auto _ : state)
    {
        auto result = liepp::so3<double>::exp(phi);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so3_exp);

static void bm_so3_log(benchmark::State& state)
{
    auto elem = liepp::so3<double>::exp(liepp::vector3<double>::Random());
    for (auto _ : state)
    {
        auto result = elem.log();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so3_log);

static void bm_so3_compose(benchmark::State& state)
{
    auto a = liepp::so3<double>::exp(liepp::vector3<double>::Random());
    auto b = liepp::so3<double>::exp(liepp::vector3<double>::Random());
    for (auto _ : state)
    {
        auto result = a * b;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so3_compose);

static void bm_so3_inverse(benchmark::State& state)
{
    auto elem = liepp::so3<double>::exp(liepp::vector3<double>::Random());
    for (auto _ : state)
    {
        auto result = elem.inverse();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so3_inverse);

static void bm_so3_adjoint(benchmark::State& state)
{
    auto elem = liepp::so3<double>::exp(liepp::vector3<double>::Random());
    for (auto _ : state)
    {
        auto result = elem.adjoint();
        benchmark::DoNotOptimize(result.data());
    }
}
BENCHMARK(bm_so3_adjoint);

static void bm_so3_coadjoint(benchmark::State& state)
{
    auto elem = liepp::so3<double>::exp(liepp::vector3<double>::Random());
    for (auto _ : state)
    {
        auto result = elem.coadjoint();
        benchmark::DoNotOptimize(result.data());
    }
}
BENCHMARK(bm_so3_coadjoint);

// ===========================================================================
// SE(3) benchmarks (6 total)
// ===========================================================================

static void bm_se3_exp(benchmark::State& state)
{
    liepp::vector6<double> twist = liepp::vector6<double>::Random();
    for (auto _ : state)
    {
        auto result = liepp::se3<double>::exp(twist);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se3_exp);

static void bm_se3_log(benchmark::State& state)
{
    auto elem = liepp::se3<double>::exp(liepp::vector6<double>::Random());
    for (auto _ : state)
    {
        auto result = elem.log();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se3_log);

static void bm_se3_compose(benchmark::State& state)
{
    auto a = liepp::se3<double>::exp(liepp::vector6<double>::Random());
    auto b = liepp::se3<double>::exp(liepp::vector6<double>::Random());
    for (auto _ : state)
    {
        auto result = a * b;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se3_compose);

static void bm_se3_inverse(benchmark::State& state)
{
    auto elem = liepp::se3<double>::exp(liepp::vector6<double>::Random());
    for (auto _ : state)
    {
        auto result = elem.inverse();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se3_inverse);

static void bm_se3_adjoint(benchmark::State& state)
{
    auto elem = liepp::se3<double>::exp(liepp::vector6<double>::Random());
    for (auto _ : state)
    {
        auto result = elem.adjoint();
        benchmark::DoNotOptimize(result.data());
    }
}
BENCHMARK(bm_se3_adjoint);

static void bm_se3_coadjoint(benchmark::State& state)
{
    auto elem = liepp::se3<double>::exp(liepp::vector6<double>::Random());
    for (auto _ : state)
    {
        auto result = elem.coadjoint();
        benchmark::DoNotOptimize(result.data());
    }
}
BENCHMARK(bm_se3_coadjoint);
