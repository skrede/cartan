/// Empirical sweep harness for the unbounded-angular fallback range.
///
/// Determines which value of k_unbounded_angular_range_v (the fallback used
/// when (position_max - position_min) is non-finite, e.g. for unbounded
/// angular joints) maximizes IK success across three iterative solver
/// families (LM via restart-wrapping, argmin SLSQP, argmin projected GN) on a
/// continuous-wrist fixture. The harness is committed in-tree and gated to a
/// manual ctest target via the DISABLED property so day-to-day regression
/// cycles stay fast; it is intended to be run on demand to update the
/// fallback constant when the surrounding solver code or geometry changes.

#include "cartan/urdf.h"

#include "cartan/serial/ik/solver/lm.h"
#include "cartan/serial/ik/solver/argmin_slsqp.h"
#include "cartan/serial/ik/solver/argmin_projected_gn.h"
#include "cartan/serial/ik/wrapper/restart_wrapper.h"

#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include "cartan/serial/fk/forward_kinematics.h"

#include "cartan/lie/se3.h"

#include <chrono>
#include <cstdio>
#include <random>
#include <vector>
#include <string>
#include <numbers>
#include <algorithm>
#include <filesystem>

namespace
{

using chain_t = cartan::kinematic_chain<double, cartan::dynamic>;
using restart_lm = cartan::restart_wrapper<chain_t,
    cartan::builtin_lm<chain_t, cartan::no_limits>, cartan::no_limits>;
using argmin_slsqp_t = cartan::argmin_slsqp<chain_t>;
using argmin_pgn_t = cartan::argmin_projected_gn<chain_t>;

/// Replace the continuous joint's bounds with finite values of half-range
/// (candidate / 2). The PoE chain math is unchanged; only the joint_limits
/// the solvers consume differ. This lets the sweep exercise the EXACT code
/// paths that finite_range_or feeds into without rebuilding the library
/// against four different inline constants.
chain_t synthesize_chain_with_range(const chain_t& source, double candidate_range)
{
    auto axes = source.axes();
    auto limits = source.limits();
    const double half = candidate_range / 2.0;
    for (auto& lim : limits)
    {
        if (!std::isfinite(lim.position_min) || !std::isfinite(lim.position_max))
        {
            lim.position_min = -half;
            lim.position_max = +half;
        }
    }
    std::vector<cartan::screw_axis<double>> ax_v(axes.begin(), axes.end());
    std::vector<cartan::joint_limits<double>> li_v(limits.begin(), limits.end());
    return chain_t(source.home(), std::move(ax_v), std::move(li_v));
}

struct trial_result
{
    bool converged{};
    double wall_us{};
};

template <typename Solver>
trial_result run_one_trial(const chain_t& chain,
                           const cartan::se3<double>& target,
                           const Eigen::VectorXd& q0)
{
    cartan::convergence_criteria<double> criteria{};
    criteria.position_tol = 1e-7;
    criteria.orientation_tol = 1e-7;
    criteria.max_iterations_per_attempt = 400;
    criteria.max_total_work_units = 2000;

    Solver solver{};
    solver.setup(chain, target, q0, criteria);

    auto start = std::chrono::steady_clock::now();
    cartan::ik_status status = cartan::ik_status::running;
    int safety = 4000;
    while (status == cartan::ik_status::running && safety-- > 0)
    {
        status = solver.step(chain, 1).status;
    }
    auto end = std::chrono::steady_clock::now();
    const double wall_us =
        std::chrono::duration<double, std::micro>(end - start).count();

    if (!solver.converged())
    {
        return {false, wall_us};
    }
    auto fk_sol = cartan::forward_kinematics(chain, solver.solution());
    const auto err = (fk_sol.end_effector.inverse() * target).log();
    const bool ok = err.template head<3>().norm() < 1e-5
        && err.template tail<3>().norm() < 1e-5;
    return {ok, wall_us};
}

struct sweep_cell
{
    double fallback;
    const char* solver_name;
    int trials;
    int successes;
    double mean_wall_us;
    double p99_wall_us;
};

template <typename Solver>
sweep_cell sweep_solver(const chain_t& chain,
                        double fallback,
                        const char* name,
                        const std::vector<Eigen::VectorXd>& targets_q)
{
    std::vector<double> wall_us_v;
    wall_us_v.reserve(targets_q.size());
    int successes = 0;

    for (const auto& q : targets_q)
    {
        auto fk = cartan::forward_kinematics(chain, q);
        Eigen::VectorXd q0(chain.num_joints());
        q0.setZero();
        auto r = run_one_trial<Solver>(chain, fk.end_effector, q0);
        if (r.converged) { ++successes; }
        wall_us_v.push_back(r.wall_us);
    }

    sweep_cell cell{};
    cell.fallback = fallback;
    cell.solver_name = name;
    cell.trials = static_cast<int>(targets_q.size());
    cell.successes = successes;
    double sum = 0.0;
    for (double w : wall_us_v) { sum += w; }
    cell.mean_wall_us = sum / static_cast<double>(wall_us_v.size());
    std::sort(wall_us_v.begin(), wall_us_v.end());
    const auto p99_idx = static_cast<std::size_t>(
        0.99 * static_cast<double>(wall_us_v.size() - 1));
    cell.p99_wall_us = wall_us_v[p99_idx];
    return cell;
}

}

int main(int, char**)
{
    const std::filesystem::path fixture =
        std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR}
        / "urdf" / "extractor_continuous_wrist.urdf";

    auto loaded = cartan::load_urdf<double>(fixture);
    if (!loaded)
    {
        std::fprintf(stderr, "fallback sweep: load_urdf failed\n");
        return 1;
    }
    chain_t source_chain = std::move(loaded->chain);

    // Generate 100 reachable targets via FK on bounded random configurations.
    // The continuous joint is sampled uniformly in [-pi, pi] regardless of
    // candidate fallback so the target set is identical across candidates.
    std::mt19937 rng(0xC0FFEE);
    std::uniform_real_distribution<double> u1(-1.4, 1.4);
    std::uniform_real_distribution<double> u2(-2.0, 2.0);
    std::uniform_real_distribution<double> u3(-std::numbers::pi, std::numbers::pi);
    std::vector<Eigen::VectorXd> targets_q;
    for (int i = 0; i < 100; ++i)
    {
        Eigen::VectorXd q(3);
        q << u1(rng), u2(rng), u3(rng);
        targets_q.push_back(q);
    }

    const double candidates[] = {
        std::numbers::pi / 2.0,
        std::numbers::pi,
        2.0 * std::numbers::pi,
        4.0 * std::numbers::pi,
    };
    const char* candidate_names[] = {"pi/2", "pi", "2*pi", "4*pi"};

    std::vector<sweep_cell> records;
    for (std::size_t ci = 0; ci < std::size(candidates); ++ci)
    {
        const double c = candidates[ci];
        chain_t chain = synthesize_chain_with_range(source_chain, c);

        records.push_back(
            sweep_solver<restart_lm>(chain, c, "restart_lm", targets_q));
        records.push_back(
            sweep_solver<argmin_slsqp_t>(chain, c, "argmin_slsqp", targets_q));
        records.push_back(
            sweep_solver<argmin_pgn_t>(chain, c, "argmin_projected_gn", targets_q));
        (void)candidate_names[ci];
    }

    std::printf("| fallback | solver | success_rate | mean_wall_us | p99_wall_us |\n");
    std::printf("|----------|--------|--------------|--------------|-------------|\n");
    for (const auto& r : records)
    {
        const double rate = static_cast<double>(r.successes)
            / static_cast<double>(r.trials);
        std::printf("| %.6f | %s | %.4f | %.2f | %.2f |\n",
            r.fallback, r.solver_name, rate, r.mean_wall_us, r.p99_wall_us);
    }

    // Aggregate score per candidate: summed success across the 3 solvers (max
    // 3 * trials), tiebroken by lowest summed mean wall time.
    struct agg { double fallback; const char* name; int success_sum; double wall_sum; };
    std::vector<agg> aggregates;
    for (std::size_t ci = 0; ci < std::size(candidates); ++ci)
    {
        agg a{candidates[ci], candidate_names[ci], 0, 0.0};
        for (const auto& r : records)
        {
            if (r.fallback == candidates[ci])
            {
                a.success_sum += r.successes;
                a.wall_sum += r.mean_wall_us;
            }
        }
        aggregates.push_back(a);
    }

    std::printf("\n| candidate | success_sum | mean_wall_sum_us |\n");
    std::printf("|-----------|-------------|------------------|\n");
    for (const auto& a : aggregates)
    {
        std::printf("| %s | %d | %.2f |\n", a.name, a.success_sum, a.wall_sum);
    }

    std::sort(aggregates.begin(), aggregates.end(),
        [](const agg& l, const agg& r) {
            if (l.success_sum != r.success_sum)
            {
                return l.success_sum > r.success_sum;
            }
            return l.wall_sum < r.wall_sum;
        });

    std::printf("\nwinner: %s (success_sum=%d, mean_wall_sum_us=%.2f)\n",
        aggregates[0].name, aggregates[0].success_sum, aggregates[0].wall_sum);
    return 0;
}
