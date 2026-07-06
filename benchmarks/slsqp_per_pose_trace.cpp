/// @file slsqp_per_pose_trace.cpp
/// @brief Per-outer-step tracer for cartan::argmin_slsqp on a focused
///        pose list. Captures argmin inner-state telemetry per outer step
///        to diagnose the 200e287 regressed-cohort failure mode.
///
/// Bypasses basic_ik_runner / restart_wrapper -- drives argmin_slsqp::step()
/// directly so we can read the inner argmin state between calls (which the
/// runner stack hides). The argmin_slsqp object still owns its own perturb-
/// restart logic; restart_count() reports those.
///
/// Per-step trace fields (one row per outer argmin_slsqp::step() call):
///   robot, pose, outer_iter, status, error_norm, error_norm_prev,
///   x_norm, x_change_norm, sigma, objective, grad_norm,
///   argmin_iter_total, argmin_iter_delta,
///   ls_calls_total, ls_calls_delta, restart_count,
///   step_us
///
/// Stack matches slsqp_per_pose_capture.cpp exactly:
///   convergence_criteria{1e-5, 1e-5, max_iterations_per_attempt = 500,
///   max_total_work_units = 500}, target_set seed 42, 1000 poses
///
/// Usage:
///   slsqp_per_pose_trace <output.csv> <robot:pose> [<robot:pose>...]
/// Example:
///   slsqp_per_pose_trace trace.csv ur3e:79 ur3e:176 ur3e:159 ur3e:0

#include "../tests/fixtures/chain_factories.h"

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/ik_result.h>
#include <cartan/serial/ik/solver/argmin_slsqp.h>
#include <cartan/serial/ik/solver/detail/halton_seed_generator.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
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

constexpr std::string_view to_string(cartan::ik_status s) noexcept
{
    using S = cartan::ik_status;
    switch (s)
    {
        case S::running:               return "running";
        case S::converged:             return "converged";
        case S::stalled:               return "stalled";
        case S::diverged:              return "diverged";
        case S::iteration_limit:       return "iteration_limit";
        case S::joint_limit_hit:       return "joint_limit_hit";
    }
    return "unknown";
}

template <int N>
void trace_pose(
    std::string_view robot,
    int pose_index,
    const chain_t<N>& chain,
    const cartan::se3<double>& target,
    const typename cartan::joint_state<double, N>::position_type& q_seed,
    const cartan::convergence_criteria<double>& criteria,
    std::ostream& out)
{
    using position_type = typename cartan::joint_state<double, N>::position_type;

    // Replicate basic_ik_runner + restart_wrapper logic in-line so we can
    // read argmin_slsqp's internal telemetry between every call.
    cartan::argmin_slsqp<chain_t<N>> solver;
    solver.setup(chain, target, q_seed, criteria);

    cartan::halton_seed_generator<chain_t<N>> seed_gen(chain);

    constexpr int wrapper_max_restarts = 20;  // restart_wrapper default

    double err_prev = std::numeric_limits<double>::quiet_NaN();
    position_type q_prev = q_seed;
    std::uint32_t inner_prev = 0;
    std::uint64_t lsc_prev = 0;
    int wrapper_restarts = 0;

    int outer = 0;
    while (true)
    {
        auto t0 = std::chrono::steady_clock::now();
        auto status = solver.step(chain, 1).status;
        auto t1 = std::chrono::steady_clock::now();
        ++outer;

        const double err = static_cast<double>(solver.error_norm());
        const auto q = solver.solution();
        double dq = 0.0;
        for (int i = 0; i < N; ++i)
        {
            const double d = static_cast<double>(q[i] - q_prev[i]);
            dq += d * d;
        }
        dq = std::sqrt(dq);

        double q_norm = 0.0;
        for (int i = 0; i < N; ++i)
            q_norm += static_cast<double>(q[i]) * static_cast<double>(q[i]);
        q_norm = std::sqrt(q_norm);

        const std::uint32_t inner_total = solver.argmin_iterations();
        const std::uint64_t lsc_total = solver.line_search_calls();
        const int restart_total = solver.restart_count();

        out << robot << ',' << pose_index << ',' << outer << ','
            << to_string(status) << ','
            << err << ',' << err_prev << ','
            << q_norm << ',' << dq << ','
            << inner_total << ',' << (inner_total - inner_prev) << ','
            << lsc_total << ',' << (lsc_total - lsc_prev) << ','
            << restart_total << ',' << wrapper_restarts << ','
            << std::chrono::duration<double, std::micro>(t1 - t0).count() << '\n';

        err_prev = err;
        q_prev = q;
        inner_prev = inner_total;
        lsc_prev = lsc_total;

        if (status == cartan::ik_status::converged) break;

        if (status != cartan::ik_status::running)
        {
            // Inner gave up. Apply restart_wrapper-style Halton restart.
            if (wrapper_restarts >= wrapper_max_restarts) break;
            auto q_new = seed_gen(wrapper_restarts);
            solver.setup(chain, target, q_new, criteria);
            ++wrapper_restarts;
            // After restart, reset inner counters tracking baselines
            inner_prev = 0;
            lsc_prev = 0;
        }

        if (outer >= criteria.max_iterations_per_attempt) break;
    }
}

template <int N>
void run_chain_traces(
    std::string_view robot,
    const chain_t<N>& chain,
    const std::vector<int>& poses,
    const cartan::convergence_criteria<double>& criteria,
    std::ostream& out)
{
    target_set<double, N> ts(chain, num_targets, target_seed);
    for (int p : poses)
    {
        if (p < 0 || p >= num_targets) continue;
        trace_pose<N>(robot, p, chain, ts.targets[p], ts.seeds[p], criteria, out);
        std::cerr << robot << " pose " << p << " traced\n";
    }
}

}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: slsqp_per_pose_trace <output.csv> <robot:pose> [<robot:pose>...]\n"
                     "Example: slsqp_per_pose_trace trace.csv ur3e:79 ur3e:176 ur3e:159\n";
        return EXIT_FAILURE;
    }

    std::ofstream out(argv[1]);
    if (!out)
    {
        std::cerr << "Failed to open output file: " << argv[1] << '\n';
        return EXIT_FAILURE;
    }
    out.precision(10);

    std::vector<int> ur3e_poses, panda_poses;
    for (int i = 2; i < argc; ++i)
    {
        std::string_view arg = argv[i];
        auto sep = arg.find(':');
        if (sep == std::string_view::npos)
        {
            std::cerr << "Bad argument (expected robot:pose): " << arg << '\n';
            return EXIT_FAILURE;
        }
        std::string_view robot = arg.substr(0, sep);
        int p = std::atoi(std::string(arg.substr(sep + 1)).c_str());
        if (robot == "ur3e")  ur3e_poses.push_back(p);
        else if (robot == "panda") panda_poses.push_back(p);
        else {
            std::cerr << "Unknown robot: " << robot << '\n';
            return EXIT_FAILURE;
        }
    }

    out << "robot,pose,outer_iter,status,error_norm,error_norm_prev,"
           "x_norm,x_change_norm,"
           "argmin_iter_total,argmin_iter_delta,"
           "ls_calls_total,ls_calls_delta,inner_restart_count,wrapper_restart_count,step_us\n";

    const cartan::convergence_criteria<double> criteria{
        .position_tol               = 1e-5,
        .orientation_tol            = 1e-5,
        .max_iterations_per_attempt = 500,
        .max_total_work_units       = 500
    };

    if (!ur3e_poses.empty())
    {
        auto chain = cartan::fixtures::make_ur3e_chain<double>();
        run_chain_traces<6>("ur3e", chain, ur3e_poses, criteria, out);
    }
    if (!panda_poses.empty())
    {
        auto chain = cartan::fixtures::make_panda_chain<double>();
        run_chain_traces<7>("panda", chain, panda_poses, criteria, out);
    }

    return EXIT_SUCCESS;
}
