#ifndef HPP_GUARD_CARTAN_BENCHMARKS_CLOSED_FORM_BENCH_UTILS_H
#define HPP_GUARD_CARTAN_BENCHMARKS_CLOSED_FORM_BENCH_UTILS_H

/// @file closed_form_bench_utils.h
/// @brief Bench-local helpers for closed-form vs iterative IK comparison.
///
/// Provides three things needed by the closed-form-vs-iterative driver TU:
///   1. closest_to_seed: selects the argmin_q ||q - q_seed||_2 from an
///      analytical_result, returning it as a cartan::expected so the call site
///      can fail through with the unchanged analytical_error path when the
///      solver returned zero solutions.
///   2. compute_bounding_box / bbox_target_set: a uniform bounding-box
///      target generator built from FK-sweep over the joint limits. This is
///      the workspace-coverage probe — it intentionally over-samples outside
///      the reachable manifold so that "fraction with has_value()" becomes a
///      meaningful coverage metric for the chain.
///   3. bm_closed_form_solver / bm_closed_form_coverage / bm_iterative_solver:
///      template benchmark drivers parameterized on the cartan::chain concept.

#include "../tests/fixtures/chain_factories.h"

#include <cartan/analytical/analytical_types.h>
#include <cartan/analytical/analytical_solver.h>
#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/chain/chain_concept.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <Eigen/Dense>
#include <benchmark/benchmark.h>

#include <random>
#include <vector>
#include <limits>
#include "cartan/expected.h"
#include <algorithm>

namespace cartan::fixtures
{

/// Pick the solution branch closest to a seed configuration.
///
/// Selects the joint vector in `r.solutions[0 .. r.count)` that minimizes
/// ||q - q_seed||_2 (compared via squared norm). Returns the unchanged
/// analytical error path with `analytical_failure::unreachable` if `r.count == 0`
/// — defensive guard; upstream analytical solvers normally surface this case
/// via `cartan::unexpected` directly.
template <typename Scalar, int N, int MaxSolutions>
[[nodiscard]] auto closest_to_seed(
    const cartan::analytical_result<Scalar, N, MaxSolutions>& r,
    const Eigen::Vector<Scalar, N>& q_seed)
    -> cartan::expected<Eigen::Vector<Scalar, N>, cartan::analytical_error<Scalar>>
{
    auto it = std::min_element(
        r.begin(), r.end(),
        [&](const Eigen::Vector<Scalar, N>& a, const Eigen::Vector<Scalar, N>& b)
        {
            return (a - q_seed).squaredNorm() < (b - q_seed).squaredNorm();
        });

    if (it == r.end())
    {
        return cartan::unexpected(cartan::analytical_error<Scalar>{
            cartan::analytical_failure::unreachable, Scalar(0)});
    }

    return *it;
}

/// Axis-aligned bounding box of the FK image over the joint limits.
///
/// Chain must be a `cartan::kinematic_chain<Scalar, N>` instance — `static_chain`
/// callers should pair with the matching `_chain` factory via the factory-pair
/// pattern in `chain_factories.h`. The dependence is via `random_joint_config`
/// and `forward_kinematics`, both of which are concretely typed on
/// `kinematic_chain` at the time of writing.
template <cartan::chain Chain>
struct bounding_box
{
    using scalar_type = typename Chain::scalar_type;
    Eigen::Matrix<scalar_type, 3, 1> tmin;
    Eigen::Matrix<scalar_type, 3, 1> tmax;
};

/// Estimate the FK-image bounding box by uniform sampling in joint space.
///
/// Performs `sample_count` independent FK evaluations of `random_joint_config`
/// and accumulates the elementwise min/max of the end-effector translation.
/// Initial extremes use `+inf` / `-inf` so the first sample always replaces
/// them. The result is intentionally loose — it bounds the reachable
/// workspace from outside and is meant to be used with `bbox_target_set` to
/// probe coverage, not to characterize the workspace exactly.
template <cartan::chain Chain>
[[nodiscard]] auto compute_bounding_box(
    const Chain& chain,
    int sample_count = 10000,
    unsigned seed = 4242)
    -> bounding_box<Chain>
{
    using Scalar = typename Chain::scalar_type;
    bounding_box<Chain> bb;
    bb.tmin = Eigen::Matrix<Scalar, 3, 1>::Constant(
        std::numeric_limits<Scalar>::infinity());
    bb.tmax = -bb.tmin;

    std::mt19937 rng(seed);
    for (int i = 0; i < sample_count; ++i)
    {
        auto q = random_joint_config(chain, rng);
        auto fk = cartan::forward_kinematics(chain, q);
        auto t = fk.end_effector.translation();
        bb.tmin = bb.tmin.cwiseMin(t);
        bb.tmax = bb.tmax.cwiseMax(t);
    }
    return bb;
}

/// Uniform target set over a chain's bounding box (identity rotation).
///
/// Stores `count` target poses whose translations are drawn uniformly from
/// the chain's bounding box and whose rotations are identity. Bounding box
/// is computed once on construction (see `compute_bounding_box`). The
/// orientation is fixed to identity because the workspace-coverage metric
/// is binary (`solver returned has_value()`) — adding random orientations
/// would entangle reachability with orientation feasibility.
template <typename Scalar, int N>
struct bbox_target_set
{
    bounding_box<cartan::kinematic_chain<Scalar, N>> bbox;
    std::vector<cartan::se3<Scalar>> targets;

    bbox_target_set(
        const cartan::kinematic_chain<Scalar, N>& chain,
        int count,
        unsigned seed = 4242)
        : bbox(compute_bounding_box(chain, 10000, seed))
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<Scalar> dx(bbox.tmin(0), bbox.tmax(0));
        std::uniform_real_distribution<Scalar> dy(bbox.tmin(1), bbox.tmax(1));
        std::uniform_real_distribution<Scalar> dz(bbox.tmin(2), bbox.tmax(2));

        targets.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i)
        {
            targets.push_back(cartan::se3<Scalar>(
                cartan::so3<Scalar>::identity(),
                Eigen::Matrix<Scalar, 3, 1>(dx(rng), dy(rng), dz(rng))));
        }
    }
};

/// Closed-form solver driver — accuracy + success-rate cell.
///
/// `static_chain_for_solver` is the static_chain instance handed to the
/// analytical solver; `chain_for_fk_check` is the matching kinematic_chain
/// used by `compute_pose_errors` (factory-pair pattern). The target set
/// supplies `targets[i]` and `seeds[i]` parallel arrays (mirrors the existing
/// pinocchio bench's `target_set` shape; the consumer's `TargetSet` template
/// parameter is intentionally unconstrained so the driver works with any
/// struct exposing those two member vectors).
template <
    cartan::chain Chain,
    cartan::chain ErrorChain,
    typename Solver,
    typename TargetSet>
void bm_closed_form_solver(
    benchmark::State& state,
    const Chain& static_chain_for_solver,
    const ErrorChain& chain_for_fk_check,
    const TargetSet& ts)
{
    using Scalar = typename Chain::scalar_type;

    Solver solver(static_chain_for_solver);

    std::size_t idx = 0;
    int successes = 0;
    Scalar total_pos = Scalar(0);
    Scalar total_ori = Scalar(0);

    const auto target_count = ts.targets.size();
    for (auto _ : state)
    {
        const auto i = idx % std::max<std::size_t>(target_count, 1);
        const auto& target = ts.targets[i];
        const auto& q_seed = ts.seeds[i];
        ++idx;

        auto result = solver.solve(target);
        if (result.has_value())
        {
            auto pick = closest_to_seed(*result, q_seed);
            if (pick.has_value())
            {
                ++successes;
                auto [pos_err, ori_err] = cartan::fixtures::compute_pose_errors(
                    chain_for_fk_check, *pick, target);
                total_pos += pos_err;
                total_ori += ori_err;
            }
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1));
    state.counters["pos_err"] = benchmark::Counter(
        static_cast<double>(total_pos) / std::max(successes, 1));
    state.counters["ori_err"] = benchmark::Counter(
        static_cast<double>(total_ori) / std::max(successes, 1));
}

/// Closed-form solver driver — workspace coverage cell.
///
/// Reports the fraction of bounding-box targets for which the analytical
/// solver returns `has_value()`. No FK back-check is performed because the
/// metric is intentionally binary (coverage = "did the closed-form admit
/// any solution at all"). Targets come from a `bbox_target_set` — no seed
/// is consumed.
template <
    cartan::chain Chain,
    typename Solver,
    typename BboxTargetSet>
void bm_closed_form_coverage(
    benchmark::State& state,
    const Chain& static_chain_for_solver,
    const BboxTargetSet& bbox_ts)
{
    Solver solver(static_chain_for_solver);

    std::size_t idx = 0;
    int hits = 0;
    const auto target_count = bbox_ts.targets.size();

    for (auto _ : state)
    {
        const auto i = idx % std::max<std::size_t>(target_count, 1);
        const auto& target = bbox_ts.targets[i];
        ++idx;

        auto result = solver.solve(target);
        if (result.has_value())
        {
            ++hits;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["coverage_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(hits) / std::max(total, 1));
}

/// Iterative-solver driver — bit-identical convergence-criteria + counter
/// shape as `bm_cartan_solver` in `ik_comparison_pinocchio_benchmarks.cpp`,
/// but template-deduced on the chain type (so the same driver instantiates
/// for `cartan::kinematic_chain<Scalar, N>` of any DOF).
template <
    cartan::chain Chain,
    typename Solver,
    typename TargetSet>
void bm_iterative_solver(
    benchmark::State& state,
    const Chain& chain,
    const TargetSet& ts,
    int max_iter = 100)
{
    using Scalar = typename Chain::scalar_type;
    cartan::convergence_criteria<Scalar> criteria{
        Scalar(1e-5), Scalar(1e-5), max_iter};

    std::size_t idx = 0;
    int successes = 0;
    int total_iter = 0;
    Scalar total_pos = Scalar(0);
    Scalar total_ori = Scalar(0);
    const auto target_count = ts.targets.size();

    for (auto _ : state)
    {
        const auto i = idx % std::max<std::size_t>(target_count, 1);
        const auto& target = ts.targets[i];
        const auto& q_seed = ts.seeds[i];
        ++idx;

        cartan::basic_ik_runner<Solver> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iter += result->iterations;
            auto [pos_err, ori_err] = cartan::fixtures::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos += pos_err;
            total_ori += ori_err;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1));
    state.counters["avg_iter"] = benchmark::Counter(
        static_cast<double>(total_iter) / std::max(successes, 1));
    state.counters["pos_err"] = benchmark::Counter(
        static_cast<double>(total_pos) / std::max(successes, 1));
    state.counters["ori_err"] = benchmark::Counter(
        static_cast<double>(total_ori) / std::max(successes, 1));
}

}

#endif
