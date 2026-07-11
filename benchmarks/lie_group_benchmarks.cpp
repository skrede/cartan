#include <cartan/types.h>
#include <cartan/lie/so2.h>
#include <cartan/lie/se2.h>
#include <cartan/lie/so3.h>
#include <cartan/lie/se3.h>

#include <benchmark/benchmark.h>

#include <array>
#include <random>
#include <cstddef>

namespace
{

// Loop-invariant inputs let the optimizer hoist or constant-fold the op under
// test (a compile-time-constant SO(2)/SE(2) input folds the whole exp to a
// store). Each cell instead draws from a table of varied inputs indexed by the
// iteration counter, and DoNotOptimize's the chosen input before the op so the
// compiler must recompute it every iteration. Power-of-two size lets the index
// wrap with a mask.
constexpr std::size_t kInputs = 1024;

}

// ===========================================================================
// SO(2) benchmarks (4 total)
// ===========================================================================

static void bm_so2_exp(benchmark::State& state)
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-3.14159, 3.14159);
    std::array<double, kInputs> thetas;
    for (auto& x : thetas) x = dist(rng);

    std::size_t i = 0;
    for (auto _ : state)
    {
        double theta = thetas[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(theta);
        auto result = cartan::so2<double>::exp(theta);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so2_exp);

static void bm_so2_log(benchmark::State& state)
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-3.14159, 3.14159);
    std::array<cartan::so2<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::so2<double>::exp(dist(rng));

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
        auto result = elem.log();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so2_log);

static void bm_so2_compose(benchmark::State& state)
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-3.14159, 3.14159);
    std::array<cartan::so2<double>, kInputs> lhs, rhs;
    for (auto& e : lhs) e = cartan::so2<double>::exp(dist(rng));
    for (auto& e : rhs) e = cartan::so2<double>::exp(dist(rng));

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto idx = i++ & (kInputs - 1);
        auto& a = lhs[idx];
        auto& b = rhs[idx];
        benchmark::DoNotOptimize(a);
        benchmark::DoNotOptimize(b);
        auto result = a * b;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so2_compose);

static void bm_so2_inverse(benchmark::State& state)
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-3.14159, 3.14159);
    std::array<cartan::so2<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::so2<double>::exp(dist(rng));

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
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
    std::array<cartan::vector3<double>, kInputs> twists;
    for (auto& v : twists) v = cartan::vector3<double>::Random();

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& twist = twists[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(twist);
        auto result = cartan::se2<double>::exp(twist);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se2_exp);

static void bm_se2_log(benchmark::State& state)
{
    std::array<cartan::se2<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::se2<double>::exp(cartan::vector3<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
        auto result = elem.log();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se2_log);

static void bm_se2_compose(benchmark::State& state)
{
    std::array<cartan::se2<double>, kInputs> lhs, rhs;
    for (auto& e : lhs) e = cartan::se2<double>::exp(cartan::vector3<double>::Random());
    for (auto& e : rhs) e = cartan::se2<double>::exp(cartan::vector3<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto idx = i++ & (kInputs - 1);
        auto& a = lhs[idx];
        auto& b = rhs[idx];
        benchmark::DoNotOptimize(a);
        benchmark::DoNotOptimize(b);
        auto result = a * b;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se2_compose);

static void bm_se2_inverse(benchmark::State& state)
{
    std::array<cartan::se2<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::se2<double>::exp(cartan::vector3<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
        auto result = elem.inverse();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se2_inverse);

static void bm_se2_adjoint(benchmark::State& state)
{
    std::array<cartan::se2<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::se2<double>::exp(cartan::vector3<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
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
    std::array<cartan::vector3<double>, kInputs> phis;
    for (auto& p : phis) p = cartan::vector3<double>::Random();

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& phi = phis[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(phi);
        auto result = cartan::so3<double>::exp(phi);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so3_exp);

static void bm_so3_log(benchmark::State& state)
{
    std::array<cartan::so3<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::so3<double>::exp(cartan::vector3<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
        auto result = elem.log();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so3_log);

static void bm_so3_compose(benchmark::State& state)
{
    std::array<cartan::so3<double>, kInputs> lhs, rhs;
    for (auto& e : lhs) e = cartan::so3<double>::exp(cartan::vector3<double>::Random());
    for (auto& e : rhs) e = cartan::so3<double>::exp(cartan::vector3<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto idx = i++ & (kInputs - 1);
        auto& a = lhs[idx];
        auto& b = rhs[idx];
        benchmark::DoNotOptimize(a);
        benchmark::DoNotOptimize(b);
        auto result = a * b;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so3_compose);

static void bm_so3_inverse(benchmark::State& state)
{
    std::array<cartan::so3<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::so3<double>::exp(cartan::vector3<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
        auto result = elem.inverse();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_so3_inverse);

static void bm_so3_adjoint(benchmark::State& state)
{
    std::array<cartan::so3<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::so3<double>::exp(cartan::vector3<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
        auto result = elem.adjoint();
        benchmark::DoNotOptimize(result.data());
    }
}
BENCHMARK(bm_so3_adjoint);

static void bm_so3_coadjoint(benchmark::State& state)
{
    std::array<cartan::so3<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::so3<double>::exp(cartan::vector3<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
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
    std::array<cartan::vector6<double>, kInputs> twists;
    for (auto& v : twists) v = cartan::vector6<double>::Random();

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& twist = twists[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(twist);
        auto result = cartan::se3<double>::exp(twist);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se3_exp);

static void bm_se3_log(benchmark::State& state)
{
    std::array<cartan::se3<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::se3<double>::exp(cartan::vector6<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
        auto result = elem.log();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se3_log);

static void bm_se3_compose(benchmark::State& state)
{
    std::array<cartan::se3<double>, kInputs> lhs, rhs;
    for (auto& e : lhs) e = cartan::se3<double>::exp(cartan::vector6<double>::Random());
    for (auto& e : rhs) e = cartan::se3<double>::exp(cartan::vector6<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto idx = i++ & (kInputs - 1);
        auto& a = lhs[idx];
        auto& b = rhs[idx];
        benchmark::DoNotOptimize(a);
        benchmark::DoNotOptimize(b);
        auto result = a * b;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se3_compose);

static void bm_se3_inverse(benchmark::State& state)
{
    std::array<cartan::se3<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::se3<double>::exp(cartan::vector6<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
        auto result = elem.inverse();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_se3_inverse);

static void bm_se3_adjoint(benchmark::State& state)
{
    std::array<cartan::se3<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::se3<double>::exp(cartan::vector6<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
        auto result = elem.adjoint();
        benchmark::DoNotOptimize(result.data());
    }
}
BENCHMARK(bm_se3_adjoint);

static void bm_se3_coadjoint(benchmark::State& state)
{
    std::array<cartan::se3<double>, kInputs> elems;
    for (auto& e : elems) e = cartan::se3<double>::exp(cartan::vector6<double>::Random());

    std::size_t i = 0;
    for (auto _ : state)
    {
        auto& elem = elems[i++ & (kInputs - 1)];
        benchmark::DoNotOptimize(elem);
        auto result = elem.coadjoint();
        benchmark::DoNotOptimize(result.data());
    }
}
BENCHMARK(bm_se3_coadjoint);
