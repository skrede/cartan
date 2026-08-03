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
///
/// The values from `not_initialized` onward are terminal before any iteration
/// runs. A solver starts in `not_initialized` so a caller that never calls
/// setup() cannot enter the work loop with a default-constructed joint vector,
/// and setup() latches one of the others when its precondition fails, because
/// every setup() returns void and has no other way to report.
enum class ik_status
{
    running,
    converged,
    diverged,
    stalled,
    joint_limit_hit,
    iteration_limit,
    aborted,
    not_initialized,
    dimension_mismatch,
    non_finite_input,
    unsupported_configuration,
    unreachable
};

/// Human-readable diagnostic for an ik_status, for logging and binding
/// exception messages. Returns a static string literal; no allocation.
constexpr const char* message(ik_status status)
{
    switch (status)
    {
    case ik_status::running:
        return "Solver is running";
    case ik_status::converged:
        return "Solver converged within the requested tolerances";
    case ik_status::diverged:
        return "Solver diverged";
    case ik_status::stalled:
        return "Solver stopped making progress";
    case ik_status::joint_limit_hit:
        return "Solution lies outside the joint limits";
    case ik_status::iteration_limit:
        return "Iteration budget exhausted before convergence";
    case ik_status::aborted:
        return "Solve was aborted by the caller";
    case ik_status::not_initialized:
        return "Solver was stepped before setup";
    case ik_status::dimension_mismatch:
        return "Seed joint vector length does not match the chain's joint count";
    case ik_status::non_finite_input:
        return "Seed joint vector or target pose contains a NaN or infinite component";
    case ik_status::unsupported_configuration:
        return "Selection objective is not defined for this chain or characteristic length";
    case ik_status::unreachable:
        return "Target lies outside the reachable workspace";
    }
    return "Unknown ik_status";
}

/// Which set of joint bounds a policy actually solved over.
///
/// A backend that cannot accept an infinite coordinate is handed a finite
/// interval substituted for the non-finite one, so on a chain carrying an
/// unbounded joint it solves a different problem from a policy that box-projects
/// against the declared bounds. Racing the two is legitimate and the split is
/// deliberate; reporting which one produced the answer is what keeps it from
/// being silent.
enum class feasible_set
{
    declared,
    substituted
};

/// Objective for the IK solve -- controls secondary optimization.
///
/// `min_error_norm` ranks on the pose residual and `min_joint_distance` on the
/// displacement from the seed configuration; the two Jacobian measures rank on
/// the singular values of the body Jacobian normalized by the characteristic
/// length. Each definition lives once, in detail/selection_metrics.h.
enum class ik_objective
{
    speed,
    min_error_norm,
    min_joint_distance,
    max_manipulability,
    max_isotropy
};

/// Failure reason reported in ik_error when solve does not converge.
///
/// The three setup-precondition reasons share their names with the terminal
/// `ik_status` values the solver latches, and the runner maps one onto the other
/// where it builds the error.
enum class ik_failure
{
    unreachable,
    diverged,
    stalled,
    iteration_limit,
    joint_limit_violation,
    aborted,
    not_initialized,
    dimension_mismatch,
    non_finite_input,
    unsupported_configuration
};

/// Human-readable diagnostic for an ik_failure, for logging and binding
/// exception messages. Returns a static string literal; no allocation.
constexpr const char* message(ik_failure failure)
{
    switch (failure)
    {
    case ik_failure::unreachable:
        return "Target lies outside the reachable workspace";
    case ik_failure::diverged:
        return "Solver diverged";
    case ik_failure::stalled:
        return "Solver stopped making progress";
    case ik_failure::iteration_limit:
        return "Iteration budget exhausted before convergence";
    case ik_failure::joint_limit_violation:
        return "Solution lies outside the joint limits";
    case ik_failure::aborted:
        return "Solve was aborted by the caller";
    case ik_failure::not_initialized:
        return "Solve was requested before setup";
    case ik_failure::dimension_mismatch:
        return "Seed joint vector length does not match the chain's joint count";
    case ik_failure::non_finite_input:
        return "Seed joint vector or target pose contains a NaN or infinite component";
    case ik_failure::unsupported_configuration:
        return "Selection objective is not defined for this chain or characteristic length";
    }
    return "Unknown ik_failure";
}

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
/// against this cap on every entry point, single-policy and racing alike. A
/// racing round-robin tick is atomic, so it can carry the accumulator past the
/// cap by at most one unit per still-active policy.
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
/// Separate from convergence_criteria, which controls per-policy behavior and
/// carries the runner's total work budget. solver_options governs the outer
/// racing loop: which objective selects the winner, and the Halton seed for
/// reproducibility.
///
/// `characteristic_length` is in the chain's linear unit and divides the body
/// Jacobian's linear rows before the decomposition the two Jacobian objectives
/// rank on, so those rows are commensurable with the angular ones. It applies
/// to those objectives alone and is not a library-wide scale. The default of
/// one reproduces the unnormalized arithmetic exactly, which states the unit
/// scale the measures always assumed rather than changing any ranking.
template <typename Scalar = double>
struct solver_options
{
    static_assert(std::is_floating_point_v<Scalar>,
        "solver_options requires a floating-point Scalar type");

    ik_objective objective{ik_objective::speed};
    unsigned int halton_seed{42};
    Scalar characteristic_length{1};
};

}

#endif
