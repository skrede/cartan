#include "benchmark_utils.h"

#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/fk/detail/axis_specializations.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <benchmark/benchmark.h>

#include <array>
#include <random>
#include <cstddef>

namespace
{

// Cartan-side cells draw q from a table of varied configs indexed by the
// iteration counter and DoNotOptimize the chosen q before the call, so the
// optimizer cannot hoist a loop-invariant FK out of the timed loop. KDL cells
// keep their own loop-invariant q (equal end-effector work is the point).
// Power-of-two size wraps the index with a mask.
constexpr std::size_t kInputs = 1024;

// ============================================================================
// Static chain factories (mirrored from test infrastructure)
// ============================================================================

template <typename Scalar>
auto make_3r_planar_static()
{
    auto kc = cartan::fixtures::make_3r_planar_chain<Scalar>();
    return cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_z, cartan::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_ur3e_static()
{
    auto kc = cartan::fixtures::make_ur3e_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_y, cartan::revolute_z, cartan::revolute_y>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_lbr_med14_static()
{
    auto kc = cartan::fixtures::make_lbr_med14_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z, cartan::revolute_y,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_kr6_sixx_static()
{
    auto kc = cartan::fixtures::make_kr6_sixx_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_panda_static()
{
    auto kc = cartan::fixtures::make_panda_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z, cartan::revolute_y,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_abb_irb120_static()
{
    auto kc = cartan::fixtures::make_abb_irb120_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_jaco2_static()
{
    auto kc = cartan::fixtures::make_jaco2_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_fetch_static()
{
    auto kc = cartan::fixtures::make_fetch_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_x, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_baxter_static()
{
    auto kc = cartan::fixtures::make_baxter_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_x, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

// ============================================================================
// Random config for static_chain
// ============================================================================

template <typename ChainType>
auto random_config_static(const ChainType& chain, std::mt19937& rng)
{
    using Scalar = typename ChainType::scalar_type;
    constexpr int N = ChainType::joints;
    using position_type = typename cartan::joint_state<Scalar, N>::position_type;

    position_type q;
    const auto& limits = chain.limits();
    for (int i = 0; i < N; ++i)
    {
        auto idx = static_cast<std::size_t>(i);
        std::uniform_real_distribution<Scalar> dist(
            limits[idx].position_min, limits[idx].position_max);
        q(i) = dist(rng);
    }
    return q;
}

// ============================================================================
// KDL FK helper
// ============================================================================

void fill_kdl_random(KDL::JntArray& q, int n, std::mt19937& rng)
{
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);
    q.resize(static_cast<unsigned int>(n));
    for (int i = 0; i < n; ++i)
        q(static_cast<unsigned int>(i)) = dist(rng);
}

// ============================================================================
// FK comparison benchmark macros
// ============================================================================

#define FK_BENCHMARK_KINEMATIC_CHAIN(ROBOT, FACTORY)                     \
static void bm_fk_##ROBOT##_kinematic_chain(benchmark::State& state)     \
{                                                                        \
    auto chain = cartan::fixtures::FACTORY<double>();                    \
    std::mt19937 rng(42);                                                \
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs; \
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng); \
    std::size_t i = 0;                                                   \
    for (auto _ : state)                                                 \
    {                                                                    \
        auto& q = qs[i++ & (kInputs - 1)];                              \
        benchmark::DoNotOptimize(q);                                    \
        auto result = cartan::forward_kinematics(chain, q);               \
        benchmark::DoNotOptimize(result);                                \
    }                                                                    \
}                                                                        \
BENCHMARK(bm_fk_##ROBOT##_kinematic_chain)

#define FK_BENCHMARK_STATIC_GENERIC(ROBOT, STATIC_FACTORY)               \
static void bm_fk_##ROBOT##_static_generic(benchmark::State& state)      \
{                                                                        \
    auto sc = STATIC_FACTORY<double>();                                   \
    cartan::detail::generic_chain_wrapper wrapped{sc};                     \
    std::mt19937 rng(42);                                                \
    std::array<decltype(random_config_static(sc, rng)), kInputs> qs;      \
    for (auto& q : qs) q = random_config_static(sc, rng);                 \
    std::size_t i = 0;                                                   \
    for (auto _ : state)                                                 \
    {                                                                    \
        auto& q = qs[i++ & (kInputs - 1)];                              \
        benchmark::DoNotOptimize(q);                                    \
        auto result = cartan::forward_kinematics(wrapped, q);             \
        benchmark::DoNotOptimize(result);                                \
    }                                                                    \
}                                                                        \
BENCHMARK(bm_fk_##ROBOT##_static_generic)

#define FK_BENCHMARK_STATIC_SPECIALIZED(ROBOT, STATIC_FACTORY)           \
static void bm_fk_##ROBOT##_static_specialized(benchmark::State& state)  \
{                                                                        \
    auto sc = STATIC_FACTORY<double>();                                   \
    std::mt19937 rng(42);                                                \
    std::array<decltype(random_config_static(sc, rng)), kInputs> qs;      \
    for (auto& q : qs) q = random_config_static(sc, rng);                 \
    std::size_t i = 0;                                                   \
    for (auto _ : state)                                                 \
    {                                                                    \
        auto& q = qs[i++ & (kInputs - 1)];                              \
        benchmark::DoNotOptimize(q);                                    \
        auto result = cartan::forward_kinematics(sc, q);                  \
        benchmark::DoNotOptimize(result);                                \
    }                                                                    \
}                                                                        \
BENCHMARK(bm_fk_##ROBOT##_static_specialized)

#define FK_BENCHMARK_KDL(ROBOT, KDL_FACTORY, N_JOINTS)                   \
static void bm_fk_##ROBOT##_kdl(benchmark::State& state)                 \
{                                                                        \
    auto chain = cartan::fixtures::KDL_FACTORY();                       \
    KDL::ChainFkSolverPos_recursive fk_solver(chain);                    \
    std::mt19937 rng(42);                                                \
    KDL::JntArray q;                                                     \
    fill_kdl_random(q, N_JOINTS, rng);                                   \
    KDL::Frame frame;                                                    \
    for (auto _ : state)                                                 \
    {                                                                    \
        fk_solver.JntToCart(q, frame);                                   \
        benchmark::DoNotOptimize(frame);                                 \
    }                                                                    \
}                                                                        \
BENCHMARK(bm_fk_##ROBOT##_kdl)

#define FK_COMPARISON(ROBOT, KC_FACTORY, SC_FACTORY, KDL_FACTORY, N)     \
    FK_BENCHMARK_KINEMATIC_CHAIN(ROBOT, KC_FACTORY);                     \
    FK_BENCHMARK_STATIC_GENERIC(ROBOT, SC_FACTORY);                      \
    FK_BENCHMARK_STATIC_SPECIALIZED(ROBOT, SC_FACTORY);                  \
    FK_BENCHMARK_KDL(ROBOT, KDL_FACTORY, N)

// ============================================================================
// All 9 robots
// ============================================================================

FK_COMPARISON(3r_planar, make_3r_planar_chain, make_3r_planar_static, make_3r_planar_kdl_chain, 3);
FK_COMPARISON(ur3e, make_ur3e_chain, make_ur3e_static, make_ur3e_kdl_chain, 6);
FK_COMPARISON(lbr_med14, make_lbr_med14_chain, make_lbr_med14_static, make_lbr_med14_kdl_chain, 7);
FK_COMPARISON(kr6_sixx, make_kr6_sixx_chain, make_kr6_sixx_static, make_kr6_sixx_kdl_chain, 6);
FK_COMPARISON(panda, make_panda_chain, make_panda_static, make_panda_kdl_chain, 7);
FK_COMPARISON(abb_irb120, make_abb_irb120_chain, make_abb_irb120_static, make_abb_irb120_kdl_chain, 6);
FK_COMPARISON(jaco2, make_jaco2_chain, make_jaco2_static, make_jaco2_kdl_chain, 6);
FK_COMPARISON(fetch, make_fetch_chain, make_fetch_static, make_fetch_kdl_chain, 7);
FK_COMPARISON(baxter, make_baxter_chain, make_baxter_static, make_baxter_kdl_chain, 7);

}
