#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_DETAIL_LIMITS_VALIDATION_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_DETAIL_LIMITS_VALIDATION_H

/// Validation predicates behind the checked joint_limits factory.

#include "cartan/serial/chain/chain_failure.h"

#include <cmath>
#include <optional>

namespace cartan::detail
{

/// An absent bound imposes nothing and is vacuously finite.
template <typename Scalar>
bool bound_is_finite(const std::optional<Scalar>& bound) noexcept
{
    return !bound.has_value() || std::isfinite(*bound);
}

template <typename Scalar>
bool bound_is_negative(const std::optional<Scalar>& bound) noexcept
{
    return bound.has_value() && *bound < Scalar(0);
}

template <typename Scalar>
std::optional<chain_failure> negative_bound_failure(
    const std::optional<Scalar>& velocity_max,
    const std::optional<Scalar>& effort_max,
    const std::optional<Scalar>& acceleration_max) noexcept
{
    if (bound_is_negative(velocity_max))
    {
        return chain_failure::negative_velocity_limit;
    }
    if (bound_is_negative(effort_max))
    {
        return chain_failure::negative_effort_limit;
    }
    if (bound_is_negative(acceleration_max))
    {
        return chain_failure::negative_acceleration_limit;
    }
    return std::nullopt;
}

/// Whether the position bounds describe a non-empty interval.
///
/// An ordering test alone is not enough, because an infinity compares equal to
/// itself: (+inf, +inf) and (-inf, -inf) both survive `position_max <
/// position_min` while describing no interval at all. An infinite bound is only
/// meaningful signed outward, so the two bounds may coincide only where they
/// are finite -- that is a joint pinned to one value, which is degenerate but
/// real.
template <typename Scalar>
bool bounds_are_an_interval(Scalar position_min, Scalar position_max) noexcept
{
    return position_min < position_max
        || (position_min == position_max && std::isfinite(position_min));
}

/// The failure a joint_limits value would carry, or nullopt when it is sound.
///
/// NaN is tested before the interval relation and not after it: every
/// comparison against a NaN is false, so a NaN pair would pass an ordering
/// test as "not reversed" and be reported under the wrong failure.
template <typename Scalar>
std::optional<chain_failure> limits_failure(
    Scalar position_min,
    Scalar position_max,
    const std::optional<Scalar>& velocity_max,
    const std::optional<Scalar>& effort_max,
    const std::optional<Scalar>& acceleration_max) noexcept
{
    if (std::isnan(position_min) || std::isnan(position_max)
        || !bound_is_finite(velocity_max) || !bound_is_finite(effort_max)
        || !bound_is_finite(acceleration_max))
    {
        return chain_failure::non_finite_input;
    }
    if (!bounds_are_an_interval(position_min, position_max))
    {
        return chain_failure::reversed_position_bounds;
    }
    return negative_bound_failure(velocity_max, effort_max, acceleration_max);
}

}

#endif
