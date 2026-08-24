#ifndef HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_ANGLE_UNWRAP_H
#define HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_ANGLE_UNWRAP_H

#include "cartan/serial/ik/detail/limit_enforcement.h"

#include <cmath>
#include <limits>
#include <numbers>
#include <algorithm>

namespace cartan
{
namespace detail
{

/// The 2*pi-equivalent of theta nearest reference, for a joint whose bounds are
/// infinite on both sides and whose every equivalent is therefore in range.
///
/// A non-finite reference returns theta unchanged: the rounding below would
/// otherwise propagate it into the result. The unconditional passthrough this
/// replaces was immune to that by accident, not by design.
template <typename Scalar>
Scalar nearest_equivalent_angle(Scalar theta, Scalar reference) noexcept
{
    if (!std::isfinite(reference))
    {
        return theta;
    }
    constexpr Scalar full_turn = Scalar(2) * std::numbers::pi_v<Scalar>;
    return theta + full_turn * std::round((reference - theta) / full_turn);
}

/// Reference-aware per-angle unwrap. canonical_angle_in_limits
/// (limit_enforcement.h) supplies the arc-anchored base representative; on a
/// multi-turn arc (span > 2*pi) several in-range representatives exist, and the
/// one nearest `reference` is picked by rounding the integer 2*pi shift toward
/// the reference and clamping it into the in-range shift window. An empty window
/// (no shift lands inside the arc) returns the base, which the caller tags
/// joint_limits_violated.
template <typename Scalar>
Scalar unwrap_to_range_nearest(
    Scalar theta, Scalar lo, Scalar hi, Scalar reference, Scalar tol) noexcept
{
    if (!std::isfinite(lo) && !std::isfinite(hi))
    {
        return nearest_equivalent_angle(theta, reference);
    }
    constexpr Scalar full_turn = Scalar(2) * std::numbers::pi_v<Scalar>;
    const Scalar inf = std::numeric_limits<Scalar>::infinity();
    const Scalar base = canonical_angle_in_limits(theta, lo, hi, tol);
    const Scalar k_lo =
        std::isfinite(lo) ? std::ceil((lo - base - tol) / full_turn) : -inf;
    const Scalar k_hi =
        std::isfinite(hi) ? std::floor((hi - base + tol) / full_turn) : inf;
    if (k_lo > k_hi)
    {
        return base;
    }
    const Scalar k =
        std::clamp(std::round((reference - base) / full_turn), k_lo, k_hi);
    return base + full_turn * k;
}

}
}

#endif
