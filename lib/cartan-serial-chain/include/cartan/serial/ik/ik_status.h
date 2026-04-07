#ifndef HPP_GUARD_CARTAN_SERIAL_IK_IK_STATUS_H
#define HPP_GUARD_CARTAN_SERIAL_IK_IK_STATUS_H

/// @file ik_status.h
/// @brief Status, objective, failure enums, convergence criteria, and solver
///        options for inverse kinematics.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.

#include <type_traits>

namespace cartan
{

/// Status returned by each IK stepper step() call.
/// Stepper is running until it converges, hits a limit, or fails.
enum class ik_status
{
    running,
    converged,
    diverged,
    stalled,
    joint_limit_hit,
    iteration_limit
};

/// Objective for the IK solve -- controls secondary optimization.
enum class ik_objective
{
    speed,
    min_distance,
    max_manipulability,
    max_isotropy
};

/// Failure reason reported in ik_error when solve does not converge.
enum class ik_failure
{
    unreachable,
    diverged,
    stalled,
    iteration_limit,
    joint_limit_violation,
    aborted
};

/// Runtime convergence criteria for IK solvers.
/// Separate position and orientation tolerances per Lynch & Park Ch. 6.2.
template <typename Scalar = double>
struct convergence_criteria
{
    Scalar position_tol{Scalar(1e-6)};
    Scalar orientation_tol{Scalar(1e-6)};
    int max_iterations{100};
};

/// Options controlling multi-policy solver racing behavior.
///
/// Separate from convergence_criteria, which controls per-policy behavior.
/// solver_options governs the outer racing loop: how many total iterations,
/// which objective selects the winner, and the Halton seed for reproducibility.
template <typename Scalar = double>
struct solver_options
{
    static_assert(std::is_floating_point_v<Scalar>,
        "solver_options requires a floating-point Scalar type");

    ik_objective objective{ik_objective::speed};
    int max_total_iterations{500};
    unsigned int halton_seed{42};
};

}

#endif
