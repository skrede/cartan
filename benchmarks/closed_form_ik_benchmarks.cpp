/// @file closed_form_ik_benchmarks.cpp
/// @brief Head-to-head IK benchmark: closed-form analytical solvers vs a
///        matched iterative subset, on synthetic 2R/3R fixtures plus the two
///        Pieper-compatible 6R robots (ABB IRB120, KR6 R900). Emits wall +
///        accuracy from FK-walked targets and workspace-coverage on a
///        per-robot bounding-box-uniform target set.
///
/// One bench-run command produces the entire head-to-head matrix: closed-form
/// cells (planar_2r_solver, spatial_3r_solver, pieper_6r_solver) alongside the
/// matched iterative subset (projected_lm, builtin_lm, and optionally
/// argmin_slsqp) on the same FK-walked target set, plus closed-form-only
/// workspace coverage cells using a uniform bounding-box target distribution.
/// Convergence criteria for the iterative subset are bit-identical to the
/// sibling cross-solver benchmark so cross-bench result joins are valid.

#include "benchmark_utils.h"
#include "closed_form_bench_utils.h"

#include <cartan/analytical/solver_2r.h>
#include <cartan/analytical/solver_3r.h>
#include <cartan/analytical/solver_6r.h>
#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/ik/solver/projected_lm.h>
#include <cartan/serial/chain/kinematic_chain.h>

#ifdef CARTAN_BUILD_ARGMIN
#include <cartan/serial/ik/solver/argmin_slsqp.h>
#endif

#include <benchmark/benchmark.h>

#include <Eigen/Dense>

#include <random>
#include <vector>

namespace
{

constexpr int num_targets  = 10000;
constexpr int bbox_targets = 1000;

// ============================================================================
// Target set generation -- FK-walked, identical seed/protocol to the sibling
// cross-solver pinocchio bench (seed 42, random_reachable_target).
// ============================================================================

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

// ============================================================================
// --- Planar 2R synthetic fixture ---
// ============================================================================

void bm_2r_closed_form(benchmark::State& state)
{
    using solver_t = cartan::planar_2r_solver<
        double, cartan::revolute_y, cartan::revolute_y>;
    static auto static_chain = cartan::benchmarks::make_planar_2r_static<double>();
    static auto bench_chain  = cartan::benchmarks::make_planar_2r_chain<double>();
    static target_set<double, 2> ts(bench_chain, num_targets);
    cartan::benchmarks::bm_closed_form_solver<
        decltype(static_chain), decltype(bench_chain), solver_t>(
        state, static_chain, bench_chain, ts);
}
BENCHMARK(bm_2r_closed_form)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_2r_projected_lm(benchmark::State& state)
{
    using chain_t  = cartan::kinematic_chain<double, 2>;
    using solver_t = cartan::ik::projected_lm<chain_t>;
    static auto chain = cartan::benchmarks::make_planar_2r_chain<double>();
    static target_set<double, 2> ts(chain, num_targets);
    cartan::benchmarks::bm_iterative_solver<chain_t, solver_t>(state, chain, ts);
}
BENCHMARK(bm_2r_projected_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_2r_builtin_lm(benchmark::State& state)
{
    using chain_t  = cartan::kinematic_chain<double, 2>;
    using solver_t = cartan::ik::builtin_lm<chain_t>;
    static auto chain = cartan::benchmarks::make_planar_2r_chain<double>();
    static target_set<double, 2> ts(chain, num_targets);
    cartan::benchmarks::bm_iterative_solver<chain_t, solver_t>(state, chain, ts);
}
BENCHMARK(bm_2r_builtin_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_2r_coverage(benchmark::State& state)
{
    using solver_t = cartan::planar_2r_solver<
        double, cartan::revolute_y, cartan::revolute_y>;
    static auto static_chain = cartan::benchmarks::make_planar_2r_static<double>();
    static auto bench_chain  = cartan::benchmarks::make_planar_2r_chain<double>();
    static cartan::benchmarks::bbox_target_set<double, 2> bbox_ts(bench_chain, bbox_targets);
    cartan::benchmarks::bm_closed_form_coverage<
        decltype(static_chain), solver_t>(state, static_chain, bbox_ts);
}
BENCHMARK(bm_2r_coverage)->Iterations(2000)->Unit(benchmark::kMicrosecond);

// ============================================================================
// --- Spatial 3R synthetic fixture (ZYZ wrist geometry) ---
// ============================================================================

void bm_3r_closed_form(benchmark::State& state)
{
    using solver_t = cartan::spatial_3r_solver<
        double, cartan::revolute_z, cartan::revolute_y, cartan::revolute_z>;
    static auto static_chain = cartan::benchmarks::make_spatial_3r_static<double>();
    static auto bench_chain  = cartan::benchmarks::make_spatial_3r_chain<double>();
    static target_set<double, 3> ts(bench_chain, num_targets);
    cartan::benchmarks::bm_closed_form_solver<
        decltype(static_chain), decltype(bench_chain), solver_t>(
        state, static_chain, bench_chain, ts);
}
BENCHMARK(bm_3r_closed_form)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_3r_projected_lm(benchmark::State& state)
{
    using chain_t  = cartan::kinematic_chain<double, 3>;
    using solver_t = cartan::ik::projected_lm<chain_t>;
    static auto chain = cartan::benchmarks::make_spatial_3r_chain<double>();
    static target_set<double, 3> ts(chain, num_targets);
    cartan::benchmarks::bm_iterative_solver<chain_t, solver_t>(state, chain, ts);
}
BENCHMARK(bm_3r_projected_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_3r_builtin_lm(benchmark::State& state)
{
    using chain_t  = cartan::kinematic_chain<double, 3>;
    using solver_t = cartan::ik::builtin_lm<chain_t>;
    static auto chain = cartan::benchmarks::make_spatial_3r_chain<double>();
    static target_set<double, 3> ts(chain, num_targets);
    cartan::benchmarks::bm_iterative_solver<chain_t, solver_t>(state, chain, ts);
}
BENCHMARK(bm_3r_builtin_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_3r_coverage(benchmark::State& state)
{
    using solver_t = cartan::spatial_3r_solver<
        double, cartan::revolute_z, cartan::revolute_y, cartan::revolute_z>;
    static auto static_chain = cartan::benchmarks::make_spatial_3r_static<double>();
    static auto bench_chain  = cartan::benchmarks::make_spatial_3r_chain<double>();
    static cartan::benchmarks::bbox_target_set<double, 3> bbox_ts(bench_chain, bbox_targets);
    cartan::benchmarks::bm_closed_form_coverage<
        decltype(static_chain), solver_t>(state, static_chain, bbox_ts);
}
BENCHMARK(bm_3r_coverage)->Iterations(2000)->Unit(benchmark::kMicrosecond);

// ============================================================================
// --- ABB IRB120 (Pieper-compatible 6R) ---
// ============================================================================

void bm_abb_irb120_pieper_6r(benchmark::State& state)
{
    using solver_t = cartan::pieper_6r_solver<
        double,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>;
    static auto static_chain = cartan::benchmarks::make_abb_irb120_static<double>();
    static auto bench_chain  = cartan::benchmarks::make_abb_irb120_chain<double>();
    static target_set<double, 6> ts(bench_chain, num_targets);
    cartan::benchmarks::bm_closed_form_solver<
        decltype(static_chain), decltype(bench_chain), solver_t>(
        state, static_chain, bench_chain, ts);
}
BENCHMARK(bm_abb_irb120_pieper_6r)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_abb_irb120_projected_lm(benchmark::State& state)
{
    using chain_t  = cartan::kinematic_chain<double, 6>;
    using solver_t = cartan::ik::projected_lm<chain_t>;
    static auto chain = cartan::benchmarks::make_abb_irb120_chain<double>();
    static target_set<double, 6> ts(chain, num_targets);
    cartan::benchmarks::bm_iterative_solver<chain_t, solver_t>(state, chain, ts);
}
BENCHMARK(bm_abb_irb120_projected_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_abb_irb120_builtin_lm(benchmark::State& state)
{
    using chain_t  = cartan::kinematic_chain<double, 6>;
    using solver_t = cartan::ik::builtin_lm<chain_t>;
    static auto chain = cartan::benchmarks::make_abb_irb120_chain<double>();
    static target_set<double, 6> ts(chain, num_targets);
    cartan::benchmarks::bm_iterative_solver<chain_t, solver_t>(state, chain, ts);
}
BENCHMARK(bm_abb_irb120_builtin_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_abb_irb120_coverage(benchmark::State& state)
{
    using solver_t = cartan::pieper_6r_solver<
        double,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>;
    static auto static_chain = cartan::benchmarks::make_abb_irb120_static<double>();
    static auto bench_chain  = cartan::benchmarks::make_abb_irb120_chain<double>();
    static cartan::benchmarks::bbox_target_set<double, 6> bbox_ts(bench_chain, bbox_targets);
    cartan::benchmarks::bm_closed_form_coverage<
        decltype(static_chain), solver_t>(state, static_chain, bbox_ts);
}
BENCHMARK(bm_abb_irb120_coverage)->Iterations(2000)->Unit(benchmark::kMicrosecond);

// ============================================================================
// --- KUKA KR6 R900 SIXX (Pieper-compatible 6R) ---
// ============================================================================

void bm_kr6_sixx_pieper_6r(benchmark::State& state)
{
    using solver_t = cartan::pieper_6r_solver<
        double,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>;
    static auto static_chain = cartan::benchmarks::make_kr6_sixx_static<double>();
    static auto bench_chain  = cartan::benchmarks::make_kr6_sixx_chain<double>();
    static target_set<double, 6> ts(bench_chain, num_targets);
    cartan::benchmarks::bm_closed_form_solver<
        decltype(static_chain), decltype(bench_chain), solver_t>(
        state, static_chain, bench_chain, ts);
}
BENCHMARK(bm_kr6_sixx_pieper_6r)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_kr6_sixx_projected_lm(benchmark::State& state)
{
    using chain_t  = cartan::kinematic_chain<double, 6>;
    using solver_t = cartan::ik::projected_lm<chain_t>;
    static auto chain = cartan::benchmarks::make_kr6_sixx_chain<double>();
    static target_set<double, 6> ts(chain, num_targets);
    cartan::benchmarks::bm_iterative_solver<chain_t, solver_t>(state, chain, ts);
}
BENCHMARK(bm_kr6_sixx_projected_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_kr6_sixx_builtin_lm(benchmark::State& state)
{
    using chain_t  = cartan::kinematic_chain<double, 6>;
    using solver_t = cartan::ik::builtin_lm<chain_t>;
    static auto chain = cartan::benchmarks::make_kr6_sixx_chain<double>();
    static target_set<double, 6> ts(chain, num_targets);
    cartan::benchmarks::bm_iterative_solver<chain_t, solver_t>(state, chain, ts);
}
BENCHMARK(bm_kr6_sixx_builtin_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_kr6_sixx_coverage(benchmark::State& state)
{
    using solver_t = cartan::pieper_6r_solver<
        double,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>;
    static auto static_chain = cartan::benchmarks::make_kr6_sixx_static<double>();
    static auto bench_chain  = cartan::benchmarks::make_kr6_sixx_chain<double>();
    static cartan::benchmarks::bbox_target_set<double, 6> bbox_ts(bench_chain, bbox_targets);
    cartan::benchmarks::bm_closed_form_coverage<
        decltype(static_chain), solver_t>(state, static_chain, bbox_ts);
}
BENCHMARK(bm_kr6_sixx_coverage)->Iterations(2000)->Unit(benchmark::kMicrosecond);

// ============================================================================
// --- argmin SLSQP matched-iterative-subset (gated, Pieper robots only) ---
// ============================================================================

#ifdef CARTAN_BUILD_ARGMIN
void bm_abb_irb120_argmin_slsqp(benchmark::State& state)
{
    using chain_t  = cartan::kinematic_chain<double, 6>;
    using solver_t = cartan::ik::argmin_slsqp<chain_t>;
    static auto chain = cartan::benchmarks::make_abb_irb120_chain<double>();
    static target_set<double, 6> ts(chain, num_targets);
    cartan::benchmarks::bm_iterative_solver<chain_t, solver_t>(state, chain, ts);
}
BENCHMARK(bm_abb_irb120_argmin_slsqp)->Iterations(2000)->Unit(benchmark::kMicrosecond);

void bm_kr6_sixx_argmin_slsqp(benchmark::State& state)
{
    using chain_t  = cartan::kinematic_chain<double, 6>;
    using solver_t = cartan::ik::argmin_slsqp<chain_t>;
    static auto chain = cartan::benchmarks::make_kr6_sixx_chain<double>();
    static target_set<double, 6> ts(chain, num_targets);
    cartan::benchmarks::bm_iterative_solver<chain_t, solver_t>(state, chain, ts);
}
BENCHMARK(bm_kr6_sixx_argmin_slsqp)->Iterations(2000)->Unit(benchmark::kMicrosecond);
#endif

// ============================================================================
// Pieper-class wrist-decoupled solver registered only for ABB IRB120 and
// KR6 R900. The following robots are intentionally excluded from
// pieper_6r cell registration:
//   ur3e:       offset wrist; axes 4-6 do not intersect at a common point.
//   jaco2:      non-spherical 6-DoF wrist geometry.
//   panda:      7 DoF.
//   fetch:      7 DoF.
//   baxter:     7 DoF with asymmetric shoulder.
//   kuka_lwr4:  7 DoF.
//   lbr_med14:  7 DoF.
// ============================================================================

}
