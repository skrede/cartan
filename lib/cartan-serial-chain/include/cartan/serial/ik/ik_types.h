#ifndef HPP_GUARD_CARTAN_SERIAL_IK_IK_TYPES_H
#define HPP_GUARD_CARTAN_SERIAL_IK_IK_TYPES_H

/// @file ik_types.h
/// @brief Foundation types for inverse kinematics: status, objective,
///        failure enums, convergence criteria, result, and error structs.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.

#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/storage_trait.h"

#include <type_traits>

namespace cartan
{

/// Status returned by each IK stepper step() call.
/// Stepper is running until it converges, hits a limit, or fails.
/// Reference: D-04 decision.
enum class ik_status
{
    running,
    converged,
    diverged,
    stalled,
    joint_limit_hit,
    iteration_limit
};

/// Objective for the IK solve — controls secondary optimization.
/// Reference: D-09 decision, inspired by TRAC-IK's SolveType.
enum class ik_objective
{
    speed,
    min_distance,
    max_manipulability,
    max_isotropy
};

/// Failure reason reported in ik_error when solve does not converge.
/// Reference: D-17 decision.
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
/// Reference: D-08 decision.
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

/// Successful IK result containing solution configuration and diagnostics.
/// Reference: D-15 decision.
template <typename Scalar = double, int N = dynamic>
struct ik_result
{
    static_assert(std::is_floating_point_v<Scalar>, "ik_result requires a floating-point Scalar type");

    joint_state<Scalar, N> solution;
    Scalar final_error_norm{};
    int iterations{};
    int solver_index{};
};

/// IK error containing failure diagnostics.
/// Reference: D-16 decision.
template <typename Scalar = double, int N = dynamic>
struct ik_error
{
    static_assert(std::is_floating_point_v<Scalar>, "ik_error requires a floating-point Scalar type");

    ik_failure reason;
    typename joint_state<Scalar, N>::position_type last_q;
    Scalar last_error_norm{};
    Scalar condition_number{};
    bool near_singular{};
};

}

#endif
