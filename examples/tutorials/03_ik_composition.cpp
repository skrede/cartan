/// @file 03_ik_composition.cpp
/// @brief Tradeoff table -- closed-form pieper_6r_solver vs iterative
///        projected_lm on a Pieper-class 6R, aggregated over N FK-walked
///        random targets.
///
/// Shows: cartan::pieper_6r_solver direct construction on a static_chain;
///        closest-to-seed selection by std::min_element over the populated
///        subset of analytical_result::solutions; projected_lm-based
///        iterative IK wrapped in basic_ik_runner; std::chrono per-call
///        timing and std::cout/std::setw columnar output. Demonstrates that "which
///        solver wins" depends entirely on what the student optimizes for.

#include "cartan/serial_chain.h"
#include "cartan/analytical/solver_6r.h"

#include <vector>
#include <chrono>
#include <iomanip>
#include <ios>
#include <random>
#include <numbers>
#include <iostream>
#include <algorithm>

namespace
{

/// One row of per-seed bookkeeping for either solver column.
struct call_record
{
    bool   success{false};
    double wall_us{0.0};
    double pos_err{0.0};
    int    multi_solutions{0};
};

/// Aggregated per-solver metrics that get printed in the tradeoff table.
struct aggregate
{
    double mean_wall_us{0.0};
    double max_wall_us{0.0};
    double mean_pos_err{0.0};
    double success_rate{0.0};
    double mean_multi_solutions{0.0};
};

aggregate summarize(const std::vector<call_record>& records)
{
    aggregate out;
    if (records.empty())
        return out;

    int success_count = 0;
    double err_sum = 0.0;
    double wall_sum = 0.0;
    double multi_sum = 0.0;
    for (const auto& r : records)
    {
        wall_sum += r.wall_us;
        out.max_wall_us = std::max(out.max_wall_us, r.wall_us);
        if (r.success)
        {
            ++success_count;
            err_sum += r.pos_err;
            multi_sum += static_cast<double>(r.multi_solutions);
        }
    }

    const auto n = static_cast<double>(records.size());
    out.mean_wall_us = wall_sum / n;
    out.success_rate = static_cast<double>(success_count) / n;
    if (success_count > 0)
    {
        // Average position error and multi-solution count are taken over
        // successful runs only: averaging in a failed run's leftover error
        // would silently skew the accuracy column.
        const auto s = static_cast<double>(success_count);
        out.mean_pos_err = err_sum / s;
        out.mean_multi_solutions = multi_sum / s;
    }
    return out;
}

}

int main()
{
    using vec3 = cartan::vector3<double>;

    // --- KUKA KR 6 R900 SIXX as a static_chain -----------------------------
    //
    // Same screw-axis block walked through in the FK + Jacobians tutorial;
    // here we instantiate it as a cartan::static_chain so the joint-tag
    // template parameter list (six revolute joints with z/y/y/x/y/x axes)
    // is visible at compile time. pieper_6r_solver requires a static_chain
    // because it specializes its wrist-Euler extraction on the joint tag
    // sequence at the wrist (joints 4-5-6 here are x-y-x, an asymmetric
    // Euler chain).
    //
    // Reference: Lynch & Park, Modern Robotics, Ch. 6.1 for the Pieper
    // decomposition and Ch. 4 for the underlying product-of-exponentials
    // form. The screw axes use the same (omega, q) parameterization the
    // FK + Jacobians tutorial introduced.
    auto k1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0,     0, 0));
    auto k2 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0,     0, 0.400));
    auto k3 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0.455, 0, 0.400));
    auto k4 = cartan::screw_axis<double>::revolute(vec3(1, 0, 0), vec3(0.875, 0, 0.400));
    auto k5 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0.875, 0, 0.400));
    auto k6 = cartan::screw_axis<double>::revolute(vec3(1, 0, 0), vec3(0.935, 0, 0.400));

    vec3 home_trans(0.935, 0, 0.400);
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    cartan::joint_limits<double> lim{
        -std::numbers::pi, std::numbers::pi};

    using chain_t = cartan::static_chain<double,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>;

    chain_t chain(
        home,
        {k1, k2, k3, k4, k5, k6},
        {lim, lim, lim, lim, lim, lim});

    // --- Solver instances --------------------------------------------------
    //
    // Both solvers are constructed once outside the per-seed loop. The
    // closed-form path pre-computes a chain-dependent wrist offset at
    // construction; reconstructing it per seed would waste those cycles.
    // The iterative path is a basic_ik_runner wrapping projected_lm with
    // the no_limits policy (the closed-form path itself imposes no joint
    // limits on its decomposition, so we make the iterative comparison
    // apples-to-apples by also dropping the box projection for this race).
    cartan::pieper_6r_solver<chain_t> analytical(chain);

    cartan::basic_ik_runner<
        cartan::projected_lm<chain_t, cartan::no_limits>>
        iterative;

    // Convergence criteria identical to the basic_ik tutorial:
    //   position_tol = 1e-6 m, orientation_tol = 1e-6 rad,
    //   max_iterations_per_attempt = 200 (max_total_work_units defaults
    //   to 200, which is sufficient on KR6 R900 FK-walked targets).
    cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    // --- Generate N FK-walked random reachable targets ---------------------
    //
    // We draw N joint configurations uniformly within the per-joint limits,
    // walk each through forward kinematics, and use the resulting pose as
    // the IK target. Because each target comes from a known joint
    // configuration, both solvers face problems with a guaranteed solution.
    // The deterministic seed (42) keeps the table reproducible; bumping N
    // to 100 or 200 sharpens the statistical signal at the cost of wall.
    constexpr int N = 50;
    std::mt19937 rng{42};
    std::uniform_real_distribution<double> dist(
        -std::numbers::pi, std::numbers::pi);

    std::vector<Eigen::Vector<double, 6>> targets_q;
    std::vector<cartan::se3<double>> targets;
    targets_q.reserve(N);
    targets.reserve(N);
    for (int i = 0; i < N; ++i)
    {
        Eigen::Vector<double, 6> q_truth;
        for (int j = 0; j < 6; ++j)
        {
            q_truth(j) = dist(rng);
        }
        targets_q.push_back(q_truth);
        targets.push_back(
            cartan::forward_kinematics(chain, q_truth).end_effector);
    }

    // --- Per-seed race -----------------------------------------------------
    //
    // Both solvers see the same target and the same seed (zeros). This is
    // the apples-to-apples shape: a closed-form enumerator returns all IK
    // branches at once and the closest-to-seed selection picks the branch
    // nearest the iterative solver's initial guess, so both paths are
    // judged on how close they get to the same neighborhood of joint space.
    const Eigen::Vector<double, 6> q_seed = Eigen::Vector<double, 6>::Zero();

    std::vector<call_record> cf_records;
    std::vector<call_record> it_records;
    cf_records.reserve(N);
    it_records.reserve(N);

    for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i)
    {
        const auto& target = targets[i];

        // --- Closed-form cell ---
        //
        // pieper_6r_solver.solve(target) returns up to eight IK branches
        // in analytical_result::solutions (the .count member reports how
        // many were FK-verified). We then pick the branch nearest to
        // q_seed via std::min_element with an L2-distance comparator;
        // there is no free closest_to_seed function in the public API --
        // the iteration is the idiom.
        {
            auto t0 = std::chrono::steady_clock::now();
            auto result = analytical.solve(target);
            auto t1 = std::chrono::steady_clock::now();

            call_record rec;
            rec.wall_us = static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    t1 - t0).count());
            if (result.has_value())
            {
                const auto& solns = *result;
                auto best = std::min_element(
                    solns.begin(), solns.end(),
                    [&](const auto& a, const auto& b)
                    {
                        return (a - q_seed).norm() < (b - q_seed).norm();
                    });
                auto fk_verify = cartan::forward_kinematics(chain, *best);
                rec.success = true;
                rec.pos_err = (fk_verify.end_effector.inverse() * target)
                    .log().norm();
                rec.multi_solutions = solns.count;
            }
            cf_records.push_back(rec);
        }

        // --- Iterative cell ---
        //
        // Single-solution path: basic_ik_runner drives projected_lm from
        // q_seed under the same convergence criteria. The runner returns
        // an cartan::expected<ik_result, ik_error>; on success the position
        // is verified by re-walking forward kinematics through the
        // returned configuration -- the same back-check the closed-form
        // path runs internally.
        {
            iterative.setup(chain, target, q_seed, criteria);
            auto t0 = std::chrono::steady_clock::now();
            auto result = iterative.solve();
            auto t1 = std::chrono::steady_clock::now();

            call_record rec;
            rec.wall_us = static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    t1 - t0).count());
            rec.multi_solutions = 1;
            if (result.has_value())
            {
                auto fk_verify = cartan::forward_kinematics(
                    chain, result->solution.position);
                rec.pos_err = (fk_verify.end_effector.inverse() * target)
                    .log().norm();
                rec.success = rec.pos_err < 1e-4;
            }
            it_records.push_back(rec);
        }
    }

    // --- Aggregation + tradeoff table --------------------------------------
    const auto cf = summarize(cf_records);
    const auto it = summarize(it_records);

    std::cout << "Race over N = " << N
              << " FK-walked random targets on KR6 R900\n";
    std::cout << std::right
              << std::setw(20) << "solver" << " | "
              << std::setw(10) << "mean_us" << " | "
              << std::setw(10) << "max_us" << " | "
              << std::setw(14) << "mean_pos_err" << " | "
              << std::setw(13) << "success_rate" << " | "
              << std::setw(16) << "multi_solutions" << "\n";
    std::cout << std::string(20 + 3 + 10 + 3 + 10 + 3 + 14 + 3 + 13 + 3 + 16,
        '-') << "\n";
    std::cout << std::right
              << std::setw(20) << "pieper_6r_solver" << " | "
              << std::setw(10) << std::fixed << std::setprecision(2) << cf.mean_wall_us << " | "
              << std::setw(10) << std::fixed << std::setprecision(2) << cf.max_wall_us << " | "
              << std::setw(14) << std::scientific << std::setprecision(3) << cf.mean_pos_err << " | "
              << std::setw(12) << std::fixed << std::setprecision(1) << (cf.success_rate * 100.0) << "% | "
              << std::setw(16) << std::fixed << std::setprecision(2) << cf.mean_multi_solutions << "\n";
    std::cout << std::right
              << std::setw(20) << "projected_lm" << " | "
              << std::setw(10) << std::fixed << std::setprecision(2) << it.mean_wall_us << " | "
              << std::setw(10) << std::fixed << std::setprecision(2) << it.max_wall_us << " | "
              << std::setw(14) << std::scientific << std::setprecision(3) << it.mean_pos_err << " | "
              << std::setw(12) << std::fixed << std::setprecision(1) << (it.success_rate * 100.0) << "% | "
              << std::setw(16) << "1" << "\n";

    // --- Footer commentary -------------------------------------------------
    //
    // The columns are intentionally narrow: they invite the student to
    // read the table, not to be told what it says. The footer does not
    // pronounce a verdict because there is none in the abstract --
    // which solver fits a given application depends on which column
    // matters most.
    std::cout << "\n";
    std::cout << "Columns: mean_us / max_us are per-call wall time in "
                 "microseconds; mean_pos_err is\n"
                 "the SE(3) log-norm of the residual end-effector pose, "
                 "averaged over successful\n"
                 "runs; success_rate is the fraction of seeds for which "
                 "the solver produced a\n"
                 "pose-correct result; multi_solutions is how many distinct "
                 "IK branches the\n"
                 "solver returned per call (closed-form enumerates the "
                 "Pieper decomposition's\n"
                 "multiple branches; the iterative path is single-solution "
                 "by construction).\n\n"
                 "There is no \"better\" solver in the abstract. Closed-form "
                 "tends to dominate\n"
                 "wall and accuracy inside the dexterous workspace where the "
                 "Pieper decomposition\n"
                 "applies; the iterative path tends to dominate workspace "
                 "coverage on targets\n"
                 "drawn uniformly from the bounding box, where some fraction "
                 "of targets lie\n"
                 "outside the closed-form's reachable set. Pick the solver "
                 "whose column you most\n"
                 "need to optimize -- and where the application demands "
                 "both, race them together.\n";

    return 0;
}
