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
        return theta;
    }
    constexpr Scalar two_pi = Scalar(2) * std::numbers::pi_v<Scalar>;
    const Scalar inf = std::numeric_limits<Scalar>::infinity();
    const Scalar base = canonical_angle_in_limits(theta, lo, hi, tol);
    const Scalar k_lo =
        std::isfinite(lo) ? std::ceil((lo - base - tol) / two_pi) : -inf;
    const Scalar k_hi =
        std::isfinite(hi) ? std::floor((hi - base + tol) / two_pi) : inf;
    if (k_lo > k_hi)
    {
        return base;
    }
    const Scalar k =
        std::clamp(std::round((reference - base) / two_pi), k_lo, k_hi);
    return base + two_pi * k;
}

}
}

#endif
