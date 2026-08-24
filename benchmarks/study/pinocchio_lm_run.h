#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_PINOCCHIO_LM_RUN_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_PINOCCHIO_LM_RUN_H

/// @file pinocchio_lm_run.h
/// @brief The peer loop's driver: budget, termination, and why it stopped.
///
/// Separated from the kernels it calls because the two answer different
/// questions -- one is the arithmetic of a Levenberg-Marquardt step, the other
/// is when an attempt is over -- and only the second is what a restart layer
/// reads.

#include "pinocchio_lm.h"

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/detail/stall_detection.h>

#include <Eigen/Dense>

#include <cmath>
#include <utility>

namespace cartan::bench::detail
{

/// The status is what a restart layer branches on, and it is produced by the
/// same stall rule cartan's own policy consults rather than by a second one
/// written here: a restart strategy compared across two kernels has to be the
/// strategy, not an imitation of it.
struct pin_lm_answer
{
    Eigen::VectorXd q;
    bool converged;
    int iterations;
    cartan::ik_status status;
};

constexpr double k_pin_stall_threshold = 1e-6;
constexpr double k_pin_divergence_factor = 10.0;
constexpr int k_pin_stall_window = 5;

template <bool Counting>
pin_lm_answer run_pinocchio_lm(
    pinocchio_model& peer,
    pin_lm_scratch& work,
    const pinocchio::SE3& goal,
    Eigen::VectorXd seed,
    const solve_budget& budget,
    kernel_counts& counts)
{
    const auto error = pin_error<Counting>(peer, seed, goal, counts);
    pin_jacobian<Counting>(peer, seed, work.jac, counts);
    work.normal.noalias() = work.jac.transpose() * work.jac;
    pin_lm_state state{std::move(seed), error, error.squaredNorm(),
        pin_initial_damping(work.normal), 2.0};

    cartan::detail::error_ring<double> history;
    const double initial = state.error.norm();

    for (int iteration = 0; iteration < budget.units; ++iteration)
    {
        if (pin_converged(state.error, budget.tolerance))
        {
            return pin_lm_answer{state.q, true, iteration, cartan::ik_status::converged};
        }
        // A run of rejected steps grows the Nielsen damping without bound and
        // no step is ever accepted, so the stall rule below never sees a
        // sample. The policy beside it terminates that attempt as a divergence,
        // and an attempt that does not would spend the whole budget where a
        // restart is what the target needs.
        if (!std::isfinite(state.lambda))
        {
            return pin_lm_answer{state.q, false, iteration, cartan::ik_status::diverged};
        }
        // Rejected steps leave the error untouched, so testing them would trip
        // the no-progress rule before the damping has grown into a usable
        // regime. This is the same restriction the policy beside it applies.
        if (pin_lm_step<Counting>(peer, work, goal, state, counts))
        {
            const auto verdict = cartan::detail::check_stall_divergence(history,
                state.error.norm(), initial, k_pin_stall_window, k_pin_stall_threshold,
                k_pin_divergence_factor);
            if (verdict != cartan::ik_status::running)
            {
                return pin_lm_answer{state.q, false, iteration + 1, verdict};
            }
        }
    }
    const bool done = pin_converged(state.error, budget.tolerance);
    return pin_lm_answer{state.q, done, budget.units,
        done ? cartan::ik_status::converged : cartan::ik_status::iteration_limit};
}

}

#endif
