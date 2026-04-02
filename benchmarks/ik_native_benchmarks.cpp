/// @file ik_native_benchmarks.cpp
/// @brief Full matrix benchmarks for native IK solvers across all robots.
///
/// Benchmarks every solver configuration against all 9 robots with 1,000
/// FK-generated targets per robot (seed=42). Solver configs include individual
/// policies (DLS, LM, projected LM, L-BFGS-B), restart-wrapped variants
/// (speed, convergence, restart+LM, Newton-Raphson, Gauss-Newton, aggressive
/// L-BFGS-B), the variadic racing solver, and TRAC-IK baseline.
///
/// NLopt solvers (BOBYQA, SLSQP) are gated behind LIEPP_HAS_NLOPT.
///
/// Target count: 1,000 (full matrix with 9 robots x 12+ configs; 10,000 would
/// take prohibitively long for a single benchmark run).

#include "benchmark_utils.h"

#include <liepp/serial/ik/ik_types.h>
#include <liepp/serial/ik/basic_ik_solver.h>
#include <liepp/serial/ik/dls_solve_policy.h>
#include <liepp/serial/ik/lm_solve_policy.h>
#include <liepp/serial/ik/limits_policy.h>
#include <liepp/serial/ik/restart_solve_policy.h>
#include <liepp/serial/ik/lbfgsb_solve_policy.h>
#include <liepp/serial/ik/projected_lm_solve_policy.h>
#include <liepp/serial/ik/newton_raphson_solve_policy.h>
#include <liepp/serial/ik/default_solvers.h>

#include <liepp/serial/ik/slsqp_solve_policy.h>
#include <liepp/serial/ik/bobyqa_solve_policy.h>
#include <liepp/serial/ik/cmaes_solve_policy.h>
#include <liepp/serial/ik/nw_sqp_solve_policy.h>
#include <liepp/serial/ik/nablapp_lm_solve_policy.h>
#include <liepp/serial/ik/nablapp_lbfgsb_solve_policy.h>
#include <liepp/serial/ik/augmented_lagrangian_solve_policy.h>

#ifdef LIEPP_HAS_NLOPT
#include <liepp/serial/ik/nlopt_bobyqa_solve_policy.h>
#include <liepp/serial/ik/nlopt_slsqp_solve_policy.h>
#endif

#include <trac_ik/trac_ik.hpp>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <benchmark/benchmark.h>

#include <vector>
#include <random>
#include <algorithm>

namespace
{

constexpr int num_targets = 1000;

// ============================================================================
// Pre-generated target/seed sets
// ============================================================================

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

// ============================================================================
// Generic benchmark for basic_ik_solver-wrapped steppers
// ============================================================================

template <int N, typename Solver>
void bm_native_solver(
    benchmark::State& state,
    const liepp::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts,
    const liepp::convergence_criteria<double>& criteria)
{
    std::size_t idx = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_position_error = 0.0;
    double total_orientation_error = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        Solver solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iterations += result->iterations;
            auto [position_error, orientation_error] = liepp::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_position_error += position_error;
            total_orientation_error += orientation_error;
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
        total_position_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_orientation_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

// ============================================================================
// Benchmark for variadic racing solver
// ============================================================================

template <int N, typename Solver>
void bm_racing_solver(
    benchmark::State& state,
    const liepp::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts,
    int max_total_iterations)
{
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 500};
    std::mt19937 rng(42);

    std::size_t idx = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_position_error = 0.0;
    double total_orientation_error = 0.0;

    int n_joints = chain.num_joints();

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        // Generate random q0 within joint limits
        typename liepp::joint_state<double, N>::position_type q0;
        if constexpr (N == liepp::dynamic)
            q0.resize(n_joints);
        for (int j = 0; j < n_joints; ++j)
        {
            auto lo = chain.limits()[static_cast<std::size_t>(j)].position_min;
            auto hi = chain.limits()[static_cast<std::size_t>(j)].position_max;
            std::uniform_real_distribution<double> dist(lo, hi);
            q0(j) = dist(rng);
        }

        Solver solver;
        liepp::solver_options<double> opts;
        opts.max_total_iterations = max_total_iterations;
        opts.halton_seed = static_cast<unsigned int>(idx);
        solver.setup(chain, target, q0, criteria, opts);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iterations += result->iterations;
            auto [position_error, orientation_error] = liepp::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_position_error += position_error;
            total_orientation_error += orientation_error;
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
        total_position_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_orientation_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

// ============================================================================
// TRAC-IK baseline
// ============================================================================

template <int N>
void bm_trac_ik_baseline(
    benchmark::State& state,
    const liepp::kinematic_chain<double, N>& chain,
    const KDL::Chain& kdl_chain,
    KDL::JntArray q_min,
    KDL::JntArray q_max,
    const target_set<double, N>& ts)
{
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

    TRAC_IK::TRAC_IK solver(kdl_chain, q_min, q_max,
                             /*maxtime=*/0.005, /*eps=*/1e-5, TRAC_IK::Speed);

    std::size_t idx = 0;
    int successes = 0;
    double total_position_error = 0.0;
    double total_orientation_error = 0.0;

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
            typename liepp::joint_state<double, N>::position_type q_solution;
            for (unsigned int j = 0; j < static_cast<unsigned int>(N); ++j)
            {
                q_solution(static_cast<int>(j)) = q_out(j);
            }
            auto [position_error, orientation_error] = liepp::benchmarks::compute_pose_errors(
                chain, q_solution, ts.targets[idx - 1]);
            total_position_error += position_error;
            total_orientation_error += orientation_error;
        }
        benchmark::DoNotOptimize(q_out);
    }

    auto total = static_cast<int>(idx);
    state.counters["success_rate"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_position_error"] = benchmark::Counter(
        total_position_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_orientation_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

// ============================================================================
// Custom benchmark for solvers requiring explicit options
// ============================================================================

/// Benchmark L-BFGS-B with aggressive tuning:
/// history_depth=10, stall_window=20, max_restarts=5, 1000 iterations.
template <int N>
void bm_lbfgsb_aggressive(
    benchmark::State& state,
    const liepp::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts)
{
    using chain_t = liepp::kinematic_chain<double, N>;
    typename liepp::lbfgsb_solve_policy<chain_t>::options lbfgsb_opts{
        .history_depth = 10,
        .stall_window = 20
    };
    typename liepp::restart_solve_policy<chain_t, liepp::lbfgsb_solve_policy<chain_t>>::options restart_opts{
        .max_restarts = 5
    };
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 1000};

    std::size_t idx = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_position_error = 0.0;
    double total_orientation_error = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        liepp::lbfgsb_solve_policy<chain_t> inner(lbfgsb_opts);
        liepp::restart_solve_policy<chain_t, liepp::lbfgsb_solve_policy<chain_t>> stepper(
            restart_opts, std::move(inner));
        liepp::basic_ik_solver solver(std::move(stepper));
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iterations += result->iterations;
            auto [position_error, orientation_error] = liepp::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_position_error += position_error;
            total_orientation_error += orientation_error;
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
        total_position_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_orientation_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

/// Benchmark Gauss-Newton: projected LM with initial_lambda_factor=0.
template <int N>
void bm_gauss_newton(
    benchmark::State& state,
    const liepp::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts)
{
    using chain_t = liepp::kinematic_chain<double, N>;
    typename liepp::projected_lm_solve_policy<chain_t>::options plm_opts{
        .initial_lambda_factor = 0.0
    };
    typename liepp::restart_solve_policy<chain_t, liepp::projected_lm_solve_policy<chain_t>>::options restart_opts{
        .max_restarts = 20
    };
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 200};

    std::size_t idx = 0;
    int successes = 0;
    int total_iterations = 0;
    double total_position_error = 0.0;
    double total_orientation_error = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        liepp::projected_lm_solve_policy<chain_t> inner(plm_opts);
        liepp::restart_solve_policy<chain_t, liepp::projected_lm_solve_policy<chain_t>> stepper(
            restart_opts, std::move(inner));
        liepp::basic_ik_solver solver(std::move(stepper));
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iterations += result->iterations;
            auto [position_error, orientation_error] = liepp::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_position_error += position_error;
            total_orientation_error += orientation_error;
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
        total_position_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
    state.counters["avg_orientation_error"] = benchmark::Counter(
        total_orientation_error / std::max(successes, 1),
        benchmark::Counter::kAvgThreads);
}

// ============================================================================
// Solver type aliases
// ============================================================================

// Chain alias for convenience
template <int N>
using chain_t = liepp::kinematic_chain<double, N>;

// Individual policies (no restart wrapper)
template <int N>
using dls_ik_solver = liepp::basic_ik_solver<liepp::dls_solve_policy<chain_t<N>>>;

template <int N>
using lm_ik_solver = liepp::basic_ik_solver<liepp::lm_solve_policy<chain_t<N>>>;

template <int N>
using projected_lm_ik_solver = liepp::basic_ik_solver<liepp::projected_lm_solve_policy<chain_t<N>>>;

template <int N>
using lbfgsb_ik_solver = liepp::basic_ik_solver<liepp::lbfgsb_solve_policy<chain_t<N>>>;

// Restart-wrapped variants
template <int N>
using speed_ik_solver = liepp::basic_ik_solver<liepp::speed_solver<chain_t<N>>>;

template <int N>
using tuned_lbfgsb = liepp::restart_solve_policy<chain_t<N>, liepp::lbfgsb_solve_policy<chain_t<N>>>;

template <int N>
using convergence_ik_solver = liepp::basic_ik_solver<tuned_lbfgsb<N>>;

template <int N>
using restart_lm = liepp::restart_solve_policy<chain_t<N>, liepp::lm_solve_policy<chain_t<N>, liepp::no_limits>>;

template <int N>
using restart_lm_ik_solver = liepp::basic_ik_solver<restart_lm<N>>;

template <int N>
using nr_restart = liepp::restart_solve_policy<chain_t<N>, liepp::newton_raphson_solve_policy<chain_t<N>, liepp::no_limits>>;

template <int N>
using nr_ik_solver = liepp::basic_ik_solver<nr_restart<N>>;

// NLopt solvers
#ifdef LIEPP_HAS_NLOPT
template <int N>
using bobyqa_restart = liepp::restart_solve_policy<chain_t<N>, liepp::nlopt_bobyqa_solve_policy<chain_t<N>>>;

template <int N>
using bobyqa_ik_solver = liepp::basic_ik_solver<bobyqa_restart<N>>;

template <int N>
using slsqp_restart = liepp::restart_solve_policy<chain_t<N>, liepp::nlopt_slsqp_solve_policy<chain_t<N>>>;

template <int N>
using slsqp_ik_solver = liepp::basic_ik_solver<slsqp_restart<N>>;
#endif

// nablapp solvers (always available)
template <int N>
using nablapp_bobyqa_restart = liepp::restart_solve_policy<chain_t<N>, liepp::bobyqa_solve_policy<chain_t<N>>>;

template <int N>
using nablapp_bobyqa_ik_solver = liepp::basic_ik_solver<nablapp_bobyqa_restart<N>>;

template <int N>
using nablapp_slsqp_restart = liepp::restart_solve_policy<chain_t<N>, liepp::slsqp_solve_policy<chain_t<N>>>;

template <int N>
using nablapp_slsqp_ik_solver = liepp::basic_ik_solver<nablapp_slsqp_restart<N>>;

// nablapp L-BFGS-B (distinct from native lbfgsb)
template <int N>
using nablapp_lbfgsb_restart = liepp::restart_solve_policy<chain_t<N>, liepp::nablapp_lbfgsb_solve_policy<chain_t<N>>>;
template <int N>
using nablapp_lbfgsb_ik_solver = liepp::basic_ik_solver<nablapp_lbfgsb_restart<N>>;

// nablapp NW-SQP (inequality-constrained)
template <int N>
using nw_sqp_restart = liepp::restart_solve_policy<chain_t<N>, liepp::nw_sqp_solve_policy<chain_t<N>>>;
template <int N>
using nw_sqp_ik_solver = liepp::basic_ik_solver<nw_sqp_restart<N>>;

// nablapp LM (via least-squares adapter)
template <int N>
using nablapp_lm_restart = liepp::restart_solve_policy<chain_t<N>, liepp::nablapp_lm_solve_policy<chain_t<N>, liepp::no_limits>>;
template <int N>
using nablapp_lm_ik_solver = liepp::basic_ik_solver<nablapp_lm_restart<N>>;

// nablapp CMA-ES (population-based, derivative-free)
template <int N>
using cmaes_restart = liepp::restart_solve_policy<chain_t<N>, liepp::cmaes_solve_policy<chain_t<N>>>;
template <int N>
using cmaes_ik_solver = liepp::basic_ik_solver<cmaes_restart<N>>;

// nablapp Augmented Lagrangian (inequality-constrained outer, L-BFGS-B inner)
template <int N>
using auglag_restart = liepp::restart_solve_policy<chain_t<N>, liepp::augmented_lagrangian_solve_policy<chain_t<N>>>;
template <int N>
using auglag_ik_solver = liepp::basic_ik_solver<auglag_restart<N>>;

// Racing: variadic solver (speed + convergence policies)
template <int N>
using racing_solver = liepp::default_solver<chain_t<N>>;

// ============================================================================
// Convergence criteria per solver type
// ============================================================================

inline liepp::convergence_criteria<double> dls_criteria()      { return {1e-5, 1e-5, 100}; }
inline liepp::convergence_criteria<double> lm_criteria()       { return {1e-5, 1e-5, 100}; }
inline liepp::convergence_criteria<double> plm_criteria()      { return {1e-5, 1e-5, 200}; }
inline liepp::convergence_criteria<double> lbfgsb_criteria()   { return {1e-5, 1e-5, 500}; }
inline liepp::convergence_criteria<double> speed_criteria()    { return {1e-5, 1e-5, 200}; }
inline liepp::convergence_criteria<double> convergence_criteria_tuned() { return {1e-5, 1e-5, 500}; }
inline liepp::convergence_criteria<double> restart_lm_criteria() { return {1e-5, 1e-5, 200}; }
inline liepp::convergence_criteria<double> nr_criteria()       { return {1e-5, 1e-5, 200}; }
inline liepp::convergence_criteria<double> bobyqa_criteria()   { return {1e-5, 1e-5, 500}; }
inline liepp::convergence_criteria<double> slsqp_criteria()    { return {1e-5, 1e-5, 500}; }
inline liepp::convergence_criteria<double> nablapp_bobyqa_criteria()  { return {1e-5, 1e-5, 500}; }
inline liepp::convergence_criteria<double> nablapp_slsqp_criteria()  { return {1e-5, 1e-5, 500}; }
inline liepp::convergence_criteria<double> nablapp_lbfgsb_criteria() { return {1e-5, 1e-5, 500}; }
inline liepp::convergence_criteria<double> nw_sqp_criteria()         { return {1e-5, 1e-5, 500}; }
inline liepp::convergence_criteria<double> nablapp_lm_criteria()     { return {1e-5, 1e-5, 200}; }
inline liepp::convergence_criteria<double> cmaes_criteria()          { return {1e-5, 1e-5, 2000}; }
inline liepp::convergence_criteria<double> auglag_criteria()         { return {1e-5, 1e-5, 500}; }

// ============================================================================
// Macro-based benchmark registration to avoid per-robot boilerplate
// ============================================================================

// Register all native solver benchmarks for a 6-DOF robot.
// ROBOT: lowercase name for benchmark function (e.g., ur3e)
// CHAIN_FN: factory function name (e.g., make_ur3e_chain)
#define REGISTER_6DOF_BENCHMARKS(ROBOT, CHAIN_FN)                                                   \
                                                                                                     \
static void bm_native_##ROBOT##_dls(benchmark::State& state)                                        \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_native_solver<6, dls_ik_solver<6>>(state, chain, ts, dls_criteria());                         \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_dls)->Iterations(1000)->Unit(benchmark::kMicrosecond);                 \
                                                                                                     \
static void bm_native_##ROBOT##_lm(benchmark::State& state)                                         \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_native_solver<6, lm_ik_solver<6>>(state, chain, ts, lm_criteria());                           \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);                  \
                                                                                                     \
static void bm_native_##ROBOT##_projected_lm(benchmark::State& state)                               \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_native_solver<6, projected_lm_ik_solver<6>>(state, chain, ts, plm_criteria());                \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_projected_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);        \
                                                                                                     \
static void bm_native_##ROBOT##_lbfgsb(benchmark::State& state)                                     \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_native_solver<6, lbfgsb_ik_solver<6>>(state, chain, ts, lbfgsb_criteria());                   \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_lbfgsb)->Iterations(1000)->Unit(benchmark::kMicrosecond);              \
                                                                                                     \
static void bm_native_##ROBOT##_speed(benchmark::State& state)                                      \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_native_solver<6, speed_ik_solver<6>>(state, chain, ts, speed_criteria());                     \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);               \
                                                                                                     \
static void bm_native_##ROBOT##_convergence(benchmark::State& state)                                \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_native_solver<6, convergence_ik_solver<6>>(state, chain, ts, convergence_criteria_tuned());   \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_convergence)->Iterations(1000)->Unit(benchmark::kMicrosecond);         \
                                                                                                     \
static void bm_native_##ROBOT##_restart_lm(benchmark::State& state)                                 \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_native_solver<6, restart_lm_ik_solver<6>>(state, chain, ts, restart_lm_criteria());           \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);          \
                                                                                                     \
static void bm_native_##ROBOT##_newton_raphson(benchmark::State& state)                              \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_native_solver<6, nr_ik_solver<6>>(state, chain, ts, nr_criteria());                           \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_newton_raphson)->Iterations(1000)->Unit(benchmark::kMicrosecond);      \
                                                                                                     \
static void bm_native_##ROBOT##_gauss_newton(benchmark::State& state)                               \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_gauss_newton<6>(state, chain, ts);                                                            \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_gauss_newton)->Iterations(1000)->Unit(benchmark::kMicrosecond);        \
                                                                                                     \
static void bm_native_##ROBOT##_lbfgsb_aggressive(benchmark::State& state)                          \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_lbfgsb_aggressive<6>(state, chain, ts);                                                       \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_lbfgsb_aggressive)->Iterations(1000)->Unit(benchmark::kMicrosecond);   \
                                                                                                     \
static void bm_native_##ROBOT##_racing(benchmark::State& state)                                     \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_racing_solver<6, racing_solver<6>>(state, chain, ts, 1000);                                \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Register TRAC-IK baseline for a 6-DOF robot.
// ROBOT: lowercase name, KDL_FN: KDL chain factory, LIMITS_FN: KDL limits factory
#define REGISTER_6DOF_TRAC_IK(ROBOT, CHAIN_FN, KDL_FN, LIMITS_FN)                                   \
                                                                                                     \
static void bm_native_##ROBOT##_trac_ik(benchmark::State& state)                                    \
{                                                                                                    \
    auto liepp_chain = liepp::benchmarks::CHAIN_FN<double>();                                        \
    auto kdl_chain = liepp::benchmarks::KDL_FN();                                                    \
    KDL::JntArray q_min(6), q_max(6);                                                                \
    liepp::benchmarks::LIMITS_FN(q_min, q_max);                                                      \
    static const target_set<double, 6> ts(liepp_chain, num_targets, 42);                             \
    bm_trac_ik_baseline<6>(state, liepp_chain, kdl_chain, q_min, q_max, ts);                         \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Register NLopt solver benchmarks for a 6-DOF robot.
#ifdef LIEPP_HAS_NLOPT
#define REGISTER_6DOF_NLOPT(ROBOT, CHAIN_FN)                                                         \
                                                                                                     \
static void bm_native_##ROBOT##_bobyqa(benchmark::State& state)                                     \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_native_solver<6, bobyqa_ik_solver<6>>(state, chain, ts, bobyqa_criteria());                   \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_bobyqa)->Iterations(1000)->Unit(benchmark::kMicrosecond);              \
                                                                                                     \
static void bm_native_##ROBOT##_slsqp(benchmark::State& state)                                      \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                   \
    bm_native_solver<6, slsqp_ik_solver<6>>(state, chain, ts, slsqp_criteria());                     \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);
#else
#define REGISTER_6DOF_NLOPT(ROBOT, CHAIN_FN)
#endif

// Register nablapp solver benchmarks for a 6-DOF robot (always available).
#define REGISTER_6DOF_NABLAPP(ROBOT, CHAIN_FN)                                                        \
                                                                                                      \
static void bm_native_##ROBOT##_nablapp_bobyqa(benchmark::State& state)                               \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                    \
    bm_native_solver<6, nablapp_bobyqa_ik_solver<6>>(state, chain, ts, nablapp_bobyqa_criteria());    \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_nablapp_bobyqa)->Iterations(1000)->Unit(benchmark::kMicrosecond);       \
                                                                                                      \
static void bm_native_##ROBOT##_nablapp_slsqp(benchmark::State& state)                               \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                    \
    bm_native_solver<6, nablapp_slsqp_ik_solver<6>>(state, chain, ts, nablapp_slsqp_criteria());     \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_nablapp_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);        \
                                                                                                      \
static void bm_native_##ROBOT##_nablapp_lbfgsb(benchmark::State& state)                              \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                    \
    bm_native_solver<6, nablapp_lbfgsb_ik_solver<6>>(state, chain, ts, nablapp_lbfgsb_criteria());   \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_nablapp_lbfgsb)->Iterations(1000)->Unit(benchmark::kMicrosecond);       \
                                                                                                      \
static void bm_native_##ROBOT##_nw_sqp(benchmark::State& state)                                      \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                    \
    bm_native_solver<6, nw_sqp_ik_solver<6>>(state, chain, ts, nw_sqp_criteria());                   \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_nw_sqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);               \
                                                                                                      \
static void bm_native_##ROBOT##_nablapp_lm(benchmark::State& state)                                  \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                    \
    bm_native_solver<6, nablapp_lm_ik_solver<6>>(state, chain, ts, nablapp_lm_criteria());           \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_nablapp_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);           \
                                                                                                      \
static void bm_native_##ROBOT##_cmaes(benchmark::State& state)                                       \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                    \
    bm_native_solver<6, cmaes_ik_solver<6>>(state, chain, ts, cmaes_criteria());                     \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_cmaes)->Iterations(1000)->Unit(benchmark::kMicrosecond);                \
                                                                                                      \
static void bm_native_##ROBOT##_auglag(benchmark::State& state)                                      \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 6> ts(chain, num_targets, 42);                                    \
    bm_native_solver<6, auglag_ik_solver<6>>(state, chain, ts, auglag_criteria());                   \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_auglag)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Register all native solver benchmarks for a 7-DOF robot.
#define REGISTER_7DOF_BENCHMARKS(ROBOT, CHAIN_FN)                                                   \
                                                                                                     \
static void bm_native_##ROBOT##_dls(benchmark::State& state)                                        \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_native_solver<7, dls_ik_solver<7>>(state, chain, ts, dls_criteria());                         \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_dls)->Iterations(1000)->Unit(benchmark::kMicrosecond);                 \
                                                                                                     \
static void bm_native_##ROBOT##_lm(benchmark::State& state)                                         \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_native_solver<7, lm_ik_solver<7>>(state, chain, ts, lm_criteria());                           \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);                  \
                                                                                                     \
static void bm_native_##ROBOT##_projected_lm(benchmark::State& state)                               \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_native_solver<7, projected_lm_ik_solver<7>>(state, chain, ts, plm_criteria());                \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_projected_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);        \
                                                                                                     \
static void bm_native_##ROBOT##_lbfgsb(benchmark::State& state)                                     \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_native_solver<7, lbfgsb_ik_solver<7>>(state, chain, ts, lbfgsb_criteria());                   \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_lbfgsb)->Iterations(1000)->Unit(benchmark::kMicrosecond);              \
                                                                                                     \
static void bm_native_##ROBOT##_speed(benchmark::State& state)                                      \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_native_solver<7, speed_ik_solver<7>>(state, chain, ts, speed_criteria());                     \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_speed)->Iterations(1000)->Unit(benchmark::kMicrosecond);               \
                                                                                                     \
static void bm_native_##ROBOT##_convergence(benchmark::State& state)                                \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_native_solver<7, convergence_ik_solver<7>>(state, chain, ts, convergence_criteria_tuned());   \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_convergence)->Iterations(1000)->Unit(benchmark::kMicrosecond);         \
                                                                                                     \
static void bm_native_##ROBOT##_restart_lm(benchmark::State& state)                                 \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_native_solver<7, restart_lm_ik_solver<7>>(state, chain, ts, restart_lm_criteria());           \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_restart_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);          \
                                                                                                     \
static void bm_native_##ROBOT##_newton_raphson(benchmark::State& state)                              \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_native_solver<7, nr_ik_solver<7>>(state, chain, ts, nr_criteria());                           \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_newton_raphson)->Iterations(1000)->Unit(benchmark::kMicrosecond);      \
                                                                                                     \
static void bm_native_##ROBOT##_gauss_newton(benchmark::State& state)                               \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_gauss_newton<7>(state, chain, ts);                                                            \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_gauss_newton)->Iterations(1000)->Unit(benchmark::kMicrosecond);        \
                                                                                                     \
static void bm_native_##ROBOT##_lbfgsb_aggressive(benchmark::State& state)                          \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_lbfgsb_aggressive<7>(state, chain, ts);                                                       \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_lbfgsb_aggressive)->Iterations(1000)->Unit(benchmark::kMicrosecond);   \
                                                                                                     \
static void bm_native_##ROBOT##_racing(benchmark::State& state)                                     \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_racing_solver<7, racing_solver<7>>(state, chain, ts, 1000);                                \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_racing)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Register TRAC-IK baseline for a 7-DOF robot.
#define REGISTER_7DOF_TRAC_IK(ROBOT, CHAIN_FN, KDL_FN, LIMITS_FN)                                   \
                                                                                                     \
static void bm_native_##ROBOT##_trac_ik(benchmark::State& state)                                    \
{                                                                                                    \
    auto liepp_chain = liepp::benchmarks::CHAIN_FN<double>();                                        \
    auto kdl_chain = liepp::benchmarks::KDL_FN();                                                    \
    KDL::JntArray q_min(7), q_max(7);                                                                \
    liepp::benchmarks::LIMITS_FN(q_min, q_max);                                                      \
    static const target_set<double, 7> ts(liepp_chain, num_targets, 42);                             \
    bm_trac_ik_baseline<7>(state, liepp_chain, kdl_chain, q_min, q_max, ts);                         \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_trac_ik)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Register NLopt solver benchmarks for a 7-DOF robot.
#ifdef LIEPP_HAS_NLOPT
#define REGISTER_7DOF_NLOPT(ROBOT, CHAIN_FN)                                                         \
                                                                                                     \
static void bm_native_##ROBOT##_bobyqa(benchmark::State& state)                                     \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_native_solver<7, bobyqa_ik_solver<7>>(state, chain, ts, bobyqa_criteria());                   \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_bobyqa)->Iterations(1000)->Unit(benchmark::kMicrosecond);              \
                                                                                                     \
static void bm_native_##ROBOT##_slsqp(benchmark::State& state)                                      \
{                                                                                                    \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                              \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                   \
    bm_native_solver<7, slsqp_ik_solver<7>>(state, chain, ts, slsqp_criteria());                     \
}                                                                                                    \
BENCHMARK(bm_native_##ROBOT##_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);
#else
#define REGISTER_7DOF_NLOPT(ROBOT, CHAIN_FN)
#endif

// Register nablapp solver benchmarks for a 7-DOF robot (always available).
#define REGISTER_7DOF_NABLAPP(ROBOT, CHAIN_FN)                                                        \
                                                                                                      \
static void bm_native_##ROBOT##_nablapp_bobyqa(benchmark::State& state)                               \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                    \
    bm_native_solver<7, nablapp_bobyqa_ik_solver<7>>(state, chain, ts, nablapp_bobyqa_criteria());    \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_nablapp_bobyqa)->Iterations(1000)->Unit(benchmark::kMicrosecond);       \
                                                                                                      \
static void bm_native_##ROBOT##_nablapp_slsqp(benchmark::State& state)                               \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                    \
    bm_native_solver<7, nablapp_slsqp_ik_solver<7>>(state, chain, ts, nablapp_slsqp_criteria());     \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_nablapp_slsqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);        \
                                                                                                      \
static void bm_native_##ROBOT##_nablapp_lbfgsb(benchmark::State& state)                              \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                    \
    bm_native_solver<7, nablapp_lbfgsb_ik_solver<7>>(state, chain, ts, nablapp_lbfgsb_criteria());   \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_nablapp_lbfgsb)->Iterations(1000)->Unit(benchmark::kMicrosecond);       \
                                                                                                      \
static void bm_native_##ROBOT##_nw_sqp(benchmark::State& state)                                      \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                    \
    bm_native_solver<7, nw_sqp_ik_solver<7>>(state, chain, ts, nw_sqp_criteria());                   \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_nw_sqp)->Iterations(1000)->Unit(benchmark::kMicrosecond);               \
                                                                                                      \
static void bm_native_##ROBOT##_nablapp_lm(benchmark::State& state)                                  \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                    \
    bm_native_solver<7, nablapp_lm_ik_solver<7>>(state, chain, ts, nablapp_lm_criteria());           \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_nablapp_lm)->Iterations(1000)->Unit(benchmark::kMicrosecond);           \
                                                                                                      \
static void bm_native_##ROBOT##_cmaes(benchmark::State& state)                                       \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                    \
    bm_native_solver<7, cmaes_ik_solver<7>>(state, chain, ts, cmaes_criteria());                     \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_cmaes)->Iterations(1000)->Unit(benchmark::kMicrosecond);                \
                                                                                                      \
static void bm_native_##ROBOT##_auglag(benchmark::State& state)                                      \
{                                                                                                     \
    auto chain = liepp::benchmarks::CHAIN_FN<double>();                                               \
    static const target_set<double, 7> ts(chain, num_targets, 42);                                    \
    bm_native_solver<7, auglag_ik_solver<7>>(state, chain, ts, auglag_criteria());                   \
}                                                                                                     \
BENCHMARK(bm_native_##ROBOT##_auglag)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// ============================================================================
// 6-DOF robots: UR3e, KR6 SIXX, ABB IRB120, Jaco2
// ============================================================================

REGISTER_6DOF_BENCHMARKS(ur3e,       make_ur3e_chain)
REGISTER_6DOF_TRAC_IK(ur3e,         make_ur3e_chain, make_ur3e_kdl_chain, make_ur3e_kdl_limits)
REGISTER_6DOF_NLOPT(ur3e,           make_ur3e_chain)
REGISTER_6DOF_NABLAPP(ur3e,         make_ur3e_chain)

REGISTER_6DOF_BENCHMARKS(kr6_sixx,   make_kr6_sixx_chain)
REGISTER_6DOF_TRAC_IK(kr6_sixx,     make_kr6_sixx_chain, make_kr6_sixx_kdl_chain, make_kr6_sixx_kdl_limits)
REGISTER_6DOF_NLOPT(kr6_sixx,       make_kr6_sixx_chain)
REGISTER_6DOF_NABLAPP(kr6_sixx,     make_kr6_sixx_chain)

REGISTER_6DOF_BENCHMARKS(abb_irb120, make_abb_irb120_chain)
REGISTER_6DOF_TRAC_IK(abb_irb120,   make_abb_irb120_chain, make_abb_irb120_kdl_chain, make_abb_irb120_kdl_limits)
REGISTER_6DOF_NLOPT(abb_irb120,     make_abb_irb120_chain)
REGISTER_6DOF_NABLAPP(abb_irb120,   make_abb_irb120_chain)

REGISTER_6DOF_BENCHMARKS(jaco2,      make_jaco2_chain)
REGISTER_6DOF_TRAC_IK(jaco2,        make_jaco2_chain, make_jaco2_kdl_chain, make_jaco2_kdl_limits)
REGISTER_6DOF_NLOPT(jaco2,          make_jaco2_chain)
REGISTER_6DOF_NABLAPP(jaco2,        make_jaco2_chain)

// ============================================================================
// 7-DOF robots: LBR Med14, Panda, Fetch, Baxter, KUKA LWR4+
// ============================================================================

REGISTER_7DOF_BENCHMARKS(lbr_med14,  make_lbr_med14_chain)
REGISTER_7DOF_TRAC_IK(lbr_med14,    make_lbr_med14_chain, make_lbr_med14_kdl_chain, make_lbr_med14_kdl_limits)
REGISTER_7DOF_NLOPT(lbr_med14,      make_lbr_med14_chain)
REGISTER_7DOF_NABLAPP(lbr_med14,    make_lbr_med14_chain)

REGISTER_7DOF_BENCHMARKS(panda,      make_panda_chain)
REGISTER_7DOF_TRAC_IK(panda,        make_panda_chain, make_panda_kdl_chain, make_panda_kdl_limits)
REGISTER_7DOF_NLOPT(panda,          make_panda_chain)
REGISTER_7DOF_NABLAPP(panda,        make_panda_chain)

REGISTER_7DOF_BENCHMARKS(fetch,      make_fetch_chain)
REGISTER_7DOF_TRAC_IK(fetch,        make_fetch_chain, make_fetch_kdl_chain, make_fetch_kdl_limits)
REGISTER_7DOF_NLOPT(fetch,          make_fetch_chain)
REGISTER_7DOF_NABLAPP(fetch,        make_fetch_chain)

REGISTER_7DOF_BENCHMARKS(baxter,     make_baxter_chain)
REGISTER_7DOF_TRAC_IK(baxter,       make_baxter_chain, make_baxter_kdl_chain, make_baxter_kdl_limits)
REGISTER_7DOF_NLOPT(baxter,         make_baxter_chain)
REGISTER_7DOF_NABLAPP(baxter,       make_baxter_chain)

REGISTER_7DOF_BENCHMARKS(kuka_lwr4,  make_kuka_lwr4_chain)
REGISTER_7DOF_TRAC_IK(kuka_lwr4,    make_kuka_lwr4_chain, make_kuka_lwr4_kdl_chain, make_kuka_lwr4_kdl_limits)
REGISTER_7DOF_NLOPT(kuka_lwr4,      make_kuka_lwr4_chain)
REGISTER_7DOF_NABLAPP(kuka_lwr4,    make_kuka_lwr4_chain)

// ============================================================================
// D-11: Dynamic vs fixed-size dimension benchmarks
// ============================================================================
//
// Paired benchmarks comparing fixed-size chain_t<6> (compile-time N) against
// dynamic-dimension chain_t<liepp::dynamic> (runtime N) using the same nablapp
// policy. Quantifies the compile-time dimension benefit from nablapp develop.

using dynamic_chain = liepp::kinematic_chain<double, liepp::dynamic>;

using nablapp_bobyqa_dynamic_restart = liepp::restart_solve_policy<dynamic_chain, liepp::bobyqa_solve_policy<dynamic_chain>>;
using nablapp_bobyqa_dynamic_solver = liepp::basic_ik_solver<nablapp_bobyqa_dynamic_restart>;

using nablapp_slsqp_dynamic_restart = liepp::restart_solve_policy<dynamic_chain, liepp::slsqp_solve_policy<dynamic_chain>>;
using nablapp_slsqp_dynamic_solver = liepp::basic_ik_solver<nablapp_slsqp_dynamic_restart>;

// Fixed-size UR3e nablapp BOBYQA (reference: same as bm_native_ur3e_nablapp_bobyqa above)
static void bm_native_ur3e_nablapp_bobyqa_fixed(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_native_solver<6, nablapp_bobyqa_ik_solver<6>>(state, chain, ts, nablapp_bobyqa_criteria());
}
BENCHMARK(bm_native_ur3e_nablapp_bobyqa_fixed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Dynamic-size UR3e nablapp BOBYQA
static void bm_native_ur3e_nablapp_bobyqa_dynamic(benchmark::State& state)
{
    auto fixed_chain = liepp::benchmarks::make_ur3e_chain<double>();
    auto chain = fixed_chain.to_dynamic();
    static const target_set<double, liepp::dynamic> ts(chain, num_targets, 42);
    bm_native_solver<liepp::dynamic, nablapp_bobyqa_dynamic_solver>(state, chain, ts, nablapp_bobyqa_criteria());
}
BENCHMARK(bm_native_ur3e_nablapp_bobyqa_dynamic)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Fixed-size UR3e nablapp SLSQP (reference: same as bm_native_ur3e_nablapp_slsqp above)
static void bm_native_ur3e_nablapp_slsqp_fixed(benchmark::State& state)
{
    auto chain = liepp::benchmarks::make_ur3e_chain<double>();
    static const target_set<double, 6> ts(chain, num_targets, 42);
    bm_native_solver<6, nablapp_slsqp_ik_solver<6>>(state, chain, ts, nablapp_slsqp_criteria());
}
BENCHMARK(bm_native_ur3e_nablapp_slsqp_fixed)->Iterations(1000)->Unit(benchmark::kMicrosecond);

// Dynamic-size UR3e nablapp SLSQP
static void bm_native_ur3e_nablapp_slsqp_dynamic(benchmark::State& state)
{
    auto fixed_chain = liepp::benchmarks::make_ur3e_chain<double>();
    auto chain = fixed_chain.to_dynamic();
    static const target_set<double, liepp::dynamic> ts(chain, num_targets, 42);
    bm_native_solver<liepp::dynamic, nablapp_slsqp_dynamic_solver>(state, chain, ts, nablapp_slsqp_criteria());
}
BENCHMARK(bm_native_ur3e_nablapp_slsqp_dynamic)->Iterations(1000)->Unit(benchmark::kMicrosecond);

}
