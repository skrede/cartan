#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_LIMITS_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_LIMITS_H

/// Joint limits for kinematic chain joints.
///
/// Stores required position bounds and optional velocity, effort,
/// and acceleration limits. Aggregate-initializable.

#include <cmath>
#include <numbers>
#include <optional>

namespace cartan
{

/// Joint limits with required position bounds and optional dynamic limits.
/// Aggregate-initializable: joint_limits<double>{-3.14, 3.14} or
/// joint_limits<double>{-3.14, 3.14, 2.0, 50.0, 10.0}.
template <typename Scalar = double>
struct joint_limits
{
    Scalar position_min;                              ///< Minimum joint position (required)
    Scalar position_max;                              ///< Maximum joint position (required)
    std::optional<Scalar> velocity_max{};             ///< Maximum joint velocity (optional)
    std::optional<Scalar> effort_max{};               ///< Maximum joint effort/torque (optional)
    std::optional<Scalar> acceleration_max{};         ///< Maximum joint acceleration (optional)

    /// Check whether a position value lies within [position_min, position_max].
    bool contains(Scalar position) const
    {
        return position >= position_min && position <= position_max;
    }
};

namespace detail
{

/// Fallback joint range used when (position_max - position_min) is non-finite,
/// e.g. for unbounded angular joints whose joint_limits use +/-infinity bounds.
/// Empirically tuned via tests/unit/continuous_joint_fallback_sweep.cpp on an
/// unbounded-wrist fixture: the sweep covers {pi/2, pi, 2*pi, 4*pi} against
/// restart-wrapped LM, argmin SLSQP, and argmin projected GN, picks the
/// candidate with the highest aggregate success rate, and breaks ties by
/// lowest mean wall time. Two full revolutions (4*pi) gives the
/// restart-perturbation step enough reach to escape unhelpful local minima on
/// the unbounded joint while keeping the active-set QP's substituted bounds
/// within numerical reach. Re-run the sweep harness when the surrounding
/// solver code or test fixture changes; lock the winner here.
template <typename Scalar>
inline constexpr Scalar k_unbounded_angular_range_v
    = Scalar(4) * std::numbers::pi_v<Scalar>;

/// Return range when it is finite; otherwise return the fallback. Used at every
/// iterative IK solver site that computes (position_max - position_min) so that
/// joints with +/-infinity bounds (unbounded angular joints) produce a finite,
/// usable step scale instead of NaN propagation through the inner loops.
template <typename Scalar>
constexpr Scalar finite_range_or(Scalar range, Scalar fallback) noexcept
{
    return std::isfinite(range) ? range : fallback;
}

/// Clamp a (position_min, position_max) pair to finite values, substituting
/// the +/-half-fallback-range when either bound is non-finite. Used to feed
/// argmin's QP active-set with finite constraint values when the underlying
/// joint is unbounded (e.g. a URDF continuous joint with +/-infinity bounds);
/// SQP-family inner solvers stall when constraint values are themselves
/// infinite even though the constraint is trivially satisfied mathematically.
template <typename Scalar>
constexpr Scalar finite_lower_or(Scalar lo, Scalar half_fallback) noexcept
{
    return std::isfinite(lo) ? lo : -half_fallback;
}

template <typename Scalar>
constexpr Scalar finite_upper_or(Scalar hi, Scalar half_fallback) noexcept
{
    return std::isfinite(hi) ? hi : +half_fallback;
}

}

}

#endif
