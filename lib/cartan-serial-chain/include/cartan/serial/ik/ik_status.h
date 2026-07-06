#ifndef HPP_GUARD_CARTAN_SERIAL_IK_IK_STATUS_H
#define HPP_GUARD_CARTAN_SERIAL_IK_IK_STATUS_H

/// Status, objective, failure enums, convergence criteria, and solver
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
    aborted,
    not_initialized
};

/// Fine-grained termination reason reported by individual solve policies.
///
/// `ik_status` distinguishes only five terminal cases (converged, diverged,
/// stalled, iteration_limit, joint_limit_hit), which is too coarse for
/// diagnosing failure clusters in SQP/BFGS-backed solvers: six underlying
/// argmin terminators collapse into `ik_status::stalled` without this.
///
/// Policies that wrap a lower-level solver (e.g. `argmin_slsqp` wrapping
/// argmin's `kraft_slsqp_policy`) report the specific inner terminator via
/// `termination_reason()`. Policies that do not opt in report
/// `ik_termination_reason::unknown`, and `basic_ik_runner` propagates the
/// reported value into `ik_error::termination_reason`.
enum class ik_termination_reason
{
    unknown,                         ///< policy did not report a finer reason
    converged,                       ///< pose tolerance met
    iteration_limit,                 ///< cartan-side max_iterations exhausted
    stall_detected,                  ///< cartan-side stall detection fired
    divergence_detected,             ///< cartan-side divergence detection fired
    joint_limit_hit,                 ///< cartan-side limits policy rejected
    solver_converged_pose_missed,    ///< inner solver converged, pose tol not met
    solver_ftol_reached,             ///< inner solver: objective tol reached, pose tol not met
    solver_xtol_reached,             ///< inner solver: step tol reached
    solver_objective_stalled,        ///< inner solver: objective stalled
    solver_roundoff_limited,         ///< inner solver: roundoff-limited
    solver_stalled,                  ///< inner solver: stall terminator
    solver_aborted,                  ///< inner solver: aborted by callback
    solver_budget_exhausted,         ///< inner solver: per-step budget exhausted
    solver_max_iterations,           ///< inner solver: max_iterations exhausted
    solver_diverged                  ///< inner solver: divergence detected
};

namespace detail
{

/// Per-`Scalar` default IK convergence tolerances (position in meters,
/// orientation in radians), mirroring the `epsilon_traits` struct + `_v`
/// alias shape.
///
/// The generic default is `1e-6` for both tolerances. On a metre-scale chain
/// the forward-kinematics round-off in `double` floors near `1e-13`, so a
/// `1e-6` gate sits far above the noise and is left unchanged.
///
/// `float` is specialized to a larger default. Two floors sit above a `1e-6`
/// gate in `float`: (1) the fundamental forward-kinematics round-off floor,
/// measured against a `double` FK oracle at a 95th-percentile of ~`1e-6` m /
/// ~`2.5e-7` rad on metre-scale chains -- already at the gate; and (2) the
/// dominant Levenberg-Marquardt residual floor, whose achievable `float` pose
/// residual is broadly distributed in the ~`1e-5`..`5e-4` band. The solver
/// residual is what the runner tests against the tolerance, so a `1e-6` gate
/// can never be met and the solve reports non-convergence forever.
///
/// The `float` default is set to `1e-4`: ~100x the recorded FK round-off floor
/// and inside the solver residual band, so a `float` solve can terminate as
/// converged. The value is validated by a reach x joint-count FK round-off
/// sweep plus a float IK convergence contrast (a `1e-6` gate terminates ~0% of
/// metre-scale solves, `1e-4` a large fraction).
template <typename Scalar>
struct default_tolerance_traits
{
    static_assert(std::is_floating_point_v<Scalar>);

    static constexpr Scalar position = Scalar(1e-6);
    static constexpr Scalar orientation = Scalar(1e-6);
};

template <>
struct default_tolerance_traits<float>
{
    static constexpr float position = 1e-4f;
    static constexpr float orientation = 1e-4f;
};

template <typename Scalar>
inline constexpr Scalar default_position_tol_v =
    default_tolerance_traits<Scalar>::position;

template <typename Scalar>
inline constexpr Scalar default_orientation_tol_v =
    default_tolerance_traits<Scalar>::orientation;

}

/// Runtime convergence criteria for IK solvers.
/// Separate position and orientation tolerances per Lynch & Park Ch. 6.2.
///
/// `max_iterations_per_attempt` bounds a single solver attempt (the per-attempt
/// cap consulted by every solver's internal iteration counter, and by
/// self-restarting solvers as their per-attempt budget before triggering a
/// restart). `max_total_work_units` bounds the runner-level total budget,
/// measured in algorithmic work units (1 unit = one major iteration of the
/// solver's design); the runner accumulates `step_result::metrics.units_consumed`
/// against this cap.
template <typename Scalar = double>
struct convergence_criteria
{
    Scalar position_tol{detail::default_position_tol_v<Scalar>};
    Scalar orientation_tol{detail::default_orientation_tol_v<Scalar>};
    int max_iterations_per_attempt{100};
    int max_total_work_units{200};
};

/// Accounting/observability metrics returned by `solve_policy::step(chain, N)`.
///
/// `units_consumed` is the number of algorithmic work units charged by the
/// `step()` call (1 unit = one major iteration of the solver's design;
/// internal-restart events charge zero). `error_norm` is the most recent task
/// error magnitude maintained by the solver.
template <typename Scalar = double>
struct step_metrics
{
    int units_consumed{};
    Scalar error_norm{};
};

/// Result of a single `solve_policy::step(chain, N)` invocation.
///
/// Splits the control-flow signal (`status`) from accounting/observability
/// (`metrics`). Future metric fields extend `step_metrics` without changing
/// the outer return shape.
template <typename Scalar = double>
struct step_result
{
    ik_status status{ik_status::running};
    step_metrics<Scalar> metrics{};
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
