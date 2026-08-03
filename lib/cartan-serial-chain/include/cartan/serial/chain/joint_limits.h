#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_LIMITS_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_LIMITS_H

/// Joint limits for kinematic chain joints.
///
/// Required position bounds plus optional velocity, effort and acceleration
/// limits. The five values are private and read-only, so the checked make()
/// factory is the only supported way a joint_limits comes into existence and
/// no ordinary expression assigns one back into a state the factory refused.
/// The type stays trivially copyable, so std::bit_cast and std::memcpy remain
/// well-defined routes around that; the guarantee is against mistakes, not
/// against a caller who reaches for one of those on purpose.

#include "cartan/serial/chain/chain_failure.h"
#include "cartan/serial/chain/detail/limits_validation.h"

#include "cartan/expected.h"

#include <cmath>
#include <numbers>
#include <optional>

namespace cartan
{

/// Joint limits with required position bounds and optional dynamic limits.
template <typename Scalar = double>
class joint_limits
{
public:
    /// Validated construction, the only path to a joint_limits value.
    ///
    /// Positive and negative infinity are legal position bounds and must stay
    /// legal: they are this library's encoding for an unbounded continuous
    /// joint. The URDF builder writes them, and finite_range_or, anchor_bounds
    /// and the fallback range below exist to consume them. The asymmetry with
    /// the dynamic bounds is therefore deliberate: a NaN is refused everywhere,
    /// while an infinite velocity, effort or acceleration bound is refused
    /// because no part of the library treats one as meaningful. An infinite
    /// bound is only meaningful signed outward, so (-inf, +inf) is the unbounded
    /// joint while (+inf, +inf) and (-inf, -inf) are refused as reversed -- they
    /// describe no interval, and an ordering test alone would admit them because
    /// inf is not less than itself.
    ///
    /// Not constexpr: std::isnan and std::isfinite are not constant
    /// expressions before C++23, and the supported compiler floor is C++20.
    static cartan::expected<joint_limits, chain_failure> make(
        Scalar position_min,
        Scalar position_max,
        std::optional<Scalar> velocity_max = std::nullopt,
        std::optional<Scalar> effort_max = std::nullopt,
        std::optional<Scalar> acceleration_max = std::nullopt)
    {
        auto failure = detail::limits_failure(
            position_min, position_max, velocity_max, effort_max, acceleration_max);
        if (failure.has_value())
        {
            return cartan::unexpected(*failure);
        }
        return joint_limits(
            position_min, position_max, velocity_max, effort_max, acceleration_max);
    }

    Scalar position_min() const { return m_position_min; }

    Scalar position_max() const { return m_position_max; }

    std::optional<Scalar> velocity_max() const { return m_velocity_max; }

    std::optional<Scalar> effort_max() const { return m_effort_max; }

    std::optional<Scalar> acceleration_max() const { return m_acceleration_max; }

    /// Whether position lies within [position_min, position_max], or nullopt
    /// when the question has no answer.
    ///
    /// A nonfinite position makes both comparisons false, which reads as
    /// "outside the limits" when the truth is that the question is malformed.
    /// The checked entry points refuse a nonfinite joint value upstream rather
    /// than having it classified here.
    std::optional<bool> contains(Scalar position) const
    {
        if (!std::isfinite(position))
        {
            return std::nullopt;
        }
        return position >= m_position_min && position <= m_position_max;
    }

    /// contains(position) with the unanswerable case decided by the caller.
    ///
    /// This is the spelling to reach for. Writing `if (lim.contains(q))` on the
    /// optional compiles and asks whether the question was answerable, not
    /// whether the joint value is in range -- it is true for a finite value
    /// well outside the bounds.
    bool contains_or(Scalar position, bool when_nonfinite) const
    {
        return contains(position).value_or(when_nonfinite);
    }

private:
    joint_limits(
        Scalar position_min,
        Scalar position_max,
        std::optional<Scalar> velocity_max,
        std::optional<Scalar> effort_max,
        std::optional<Scalar> acceleration_max)
        : m_position_min(position_min)
        , m_position_max(position_max)
        , m_effort_max(effort_max)
        , m_velocity_max(velocity_max)
        , m_acceleration_max(acceleration_max)
    {
    }

    Scalar m_position_min;
    Scalar m_position_max;
    std::optional<Scalar> m_effort_max;
    std::optional<Scalar> m_velocity_max;
    std::optional<Scalar> m_acceleration_max;
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
///
/// The unit is turns and the sweep ran on angular joints only, so an unbounded
/// prismatic joint receives a window of 4*pi of the chain's linear unit -- an
/// angular magnitude wearing a length's clothing.
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

/// A finite position-bound pair substituted for one that may carry an infinity.
template <typename Scalar>
struct anchored_bounds
{
    Scalar lower;
    Scalar upper;
};

/// Substitute a finite interval of the given width for a bound pair with a
/// non-finite side: it runs inward from whichever side is finite, and is
/// centered on reference when neither is. Feeds argmin's QP active-set and the
/// multistart seeder, neither of which accepts an infinite coordinate.
///
/// Anchoring to the finite side is load-bearing: +/-width/2 about the origin
/// substitutes the empty interval [10, width/2] for a joint feasible from 10
/// radians upward, handing a solvable problem to a backend as an unsolvable one.
template <typename Scalar>
constexpr anchored_bounds<Scalar> anchor_bounds(
    Scalar lo, Scalar hi, Scalar width, Scalar reference) noexcept
{
    if (std::isfinite(lo) && std::isfinite(hi))
    {
        return {lo, hi};
    }
    if (std::isfinite(lo))
    {
        return {lo, lo + width};
    }
    if (std::isfinite(hi))
    {
        return {hi - width, hi};
    }
    const Scalar half = width / Scalar(2);
    return {reference - half, reference + half};
}

}

}

#endif
