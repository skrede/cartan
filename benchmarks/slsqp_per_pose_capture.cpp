/// @file slsqp_per_pose_capture.cpp
/// @brief Per-pose SLSQP timing/accuracy capture for cross-library coordination.
///
/// Runs the same argmin_slsqp solver stack as the bm_full_*_argmin_slsqp /
/// bm_comparison_*_argmin_slsqp google/benchmark cells (basic_ik_runner with
/// restart_wrapper around argmin_slsqp), but emits per-pose CSV rows instead
/// of aggregated counters, so a coordinator agent can compare distributions
/// (not just means) before and after a per-iter optimization.
///
/// Stack matches turn 30's measured baseline exactly:
///   - convergence_criteria<double>{1e-5, 1e-5, 500}
///   - target_set with seed 42, 1000 poses per robot
///   - argmin_slsqp default options + restart_wrapper default max_restarts
///
/// Usage: slsqp_per_pose_capture <output.csv>

#include "../tests/fixtures/chain_factories.h"

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/ik_result.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/ik/wrapper/restart_wrapper.h>
#include <cartan/serial/ik/solver/argmin_slsqp.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace
{

constexpr int num_targets = 1000;
constexpr unsigned target_seed = 42;

template <typename Scalar, int N>
struct target_set
{
    using position_type = typename cartan::joint_state<Scalar, N>::position_type;

    std::vector<cartan::se3<Scalar>> targets;
    std::vector<position_type> seeds;

    target_set(const cartan::kinematic_chain<Scalar, N>& chain, int count, unsigned seed)
    {
        std::mt19937 rng(seed);
        targets.reserve(static_cast<std::size_t>(count));
        seeds.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i)
        {
            targets.push_back(cartan::fixtures::random_reachable_target(chain, rng));
            seeds.push_back(cartan::fixtures::random_joint_config(chain, rng));
        }
    }
};

template <int N>
using chain_t = cartan::kinematic_chain<double, N>;

template <int N>
using argmin_slsqp_solver = cartan::basic_ik_runner<
    cartan::ik::restart_wrapper<chain_t<N>, cartan::ik::argmin_slsqp<chain_t<N>>>>;

constexpr std::string_view to_string(cartan::ik_failure r) noexcept
{
    using F = cartan::ik_failure;
    switch (r)
    {
        case F::unreachable:           return "unreachable";
        case F::diverged:              return "diverged";
        case F::stalled:               return "stalled";
        case F::iteration_limit:       return "iteration_limit";
        case F::joint_limit_violation: return "joint_limit_violation";
        case F::aborted:               return "aborted";
    }
    return "unknown_failure";
}

constexpr std::string_view to_string(cartan::ik_termination_reason r) noexcept
{
    using R = cartan::ik_termination_reason;
    switch (r)
    {
        case R::unknown:                      return "unknown";
        case R::converged:                    return "converged";
        case R::iteration_limit:              return "iteration_limit";
        case R::stall_detected:               return "stall_detected";
        case R::divergence_detected:          return "divergence_detected";
        case R::joint_limit_hit:              return "joint_limit_hit";
        case R::solver_converged_pose_missed: return "solver_converged_pose_missed";
        case R::solver_ftol_reached:          return "solver_ftol_reached";
        case R::solver_xtol_reached:          return "solver_xtol_reached";
        case R::solver_objective_stalled:     return "solver_objective_stalled";
        case R::solver_roundoff_limited:      return "solver_roundoff_limited";
        case R::solver_stalled:               return "solver_stalled";
        case R::solver_aborted:               return "solver_aborted";
        case R::solver_budget_exhausted:      return "solver_budget_exhausted";
        case R::solver_max_iterations:        return "solver_max_iterations";
        case R::solver_diverged:              return "solver_diverged";
    }
    return "unmapped";
}

template <int N, typename Solver>
void run_per_pose(
    std::string_view robot,
    const chain_t<N>& chain,
    const target_set<double, N>& ts,
    const cartan::convergence_criteria<double>& criteria,
    std::ostream& out)
{
    for (std::size_t idx = 0; idx < ts.targets.size(); ++idx)
    {
        const auto& target = ts.targets[idx];
        const auto& q_seed = ts.seeds[idx];

        Solver solver;

        auto t0 = std::chrono::steady_clock::now();
        solver.setup(chain, target, q_seed, criteria);
        auto t1 = std::chrono::steady_clock::now();
        auto result = solver.solve();
        auto t2 = std::chrono::steady_clock::now();
        const double setup_us =
            std::chrono::duration<double, std::micro>(t1 - t0).count();
        const double solve_us =
            std::chrono::duration<double, std::micro>(t2 - t1).count();
        const double wall_us = setup_us + solve_us;

        bool success = result.has_value();
        int iters = 0;
        double pos_err = std::numeric_limits<double>::quiet_NaN();
        double ori_err = std::numeric_limits<double>::quiet_NaN();
        double final_obj = std::numeric_limits<double>::quiet_NaN();
        bool pose_hit = false;
        std::string_view status_str{"runner_failure"};
        std::string_view termination_str{"unknown"};

        if (success)
        {
            iters = result->iterations;
            const auto& q = result->solution.position;
            auto fk = cartan::forward_kinematics(chain, q);
            auto Vb = (target.inverse() * fk.end_effector).log();
            ori_err = static_cast<double>(Vb.template head<3>().norm());
            pos_err = static_cast<double>(Vb.template tail<3>().norm());
            final_obj = 0.5 * static_cast<double>(Vb.squaredNorm());
            pose_hit = pos_err <= criteria.position_tol
                    && ori_err <= criteria.orientation_tol;
            status_str = pose_hit ? "runner_success_pose_hit" : "runner_success_pose_miss";
            termination_str = "converged";
        }
        else
        {
            const auto& e = result.error();
            status_str = to_string(e.reason);
            termination_str = to_string(e.termination_reason);
            const auto& q = e.last_q;
            if (q.size() == chain.num_joints())
            {
                auto fk = cartan::forward_kinematics(chain, q);
                auto Vb = (target.inverse() * fk.end_effector).log();
                ori_err = static_cast<double>(Vb.template head<3>().norm());
                pos_err = static_cast<double>(Vb.template tail<3>().norm());
                final_obj = 0.5 * static_cast<double>(Vb.squaredNorm());
            }
        }

        out << robot << ',' << idx << ','
            << wall_us << ','
            << setup_us << ','
            << solve_us << ','
            << iters << ','
            << (pose_hit ? 1 : 0) << ','
            << pos_err << ',' << ori_err << ','
            << final_obj << ','
            << status_str << ','
            << termination_str << '\n';
    }
}

}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: slsqp_per_pose_capture <output.csv>\n";
        return EXIT_FAILURE;
    }

    std::ofstream out(argv[1]);
    if (!out)
    {
        std::cerr << "Failed to open output file: " << argv[1] << '\n';
        return EXIT_FAILURE;
    }
    out.precision(10);

    out << "robot,pose_index,wall_time_us,setup_us,solve_us,runner_iterations,"
           "pose_tolerance_hit,pos_error_m,ori_error_rad,final_objective,"
           "status,termination_reason\n";

    const cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    {
        auto chain = cartan::fixtures::make_ur3e_chain<double>();
        target_set<double, 6> ts(chain, num_targets, target_seed);
        run_per_pose<6, argmin_slsqp_solver<6>>("ur3e", chain, ts, criteria, out);
        std::cerr << "ur3e: 1000 poses written\n";
    }
    {
        auto chain = cartan::fixtures::make_panda_chain<double>();
        target_set<double, 7> ts(chain, num_targets, target_seed);
        run_per_pose<7, argmin_slsqp_solver<7>>("panda", chain, ts, criteria, out);
        std::cerr << "panda: 1000 poses written\n";
    }

    return EXIT_SUCCESS;
}
