#ifndef HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_CLAMPED_TRIG_H
#define HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_CLAMPED_TRIG_H

#include <cartan/detail/compat.h>

#include <algorithm>
#include <cmath>

namespace cartan::detail
{

template <typename Scalar>
[[nodiscard]] CARTAN_CONSTEXPR_CMATH Scalar safe_acos(Scalar x)
{
    return std::acos(std::clamp(x, Scalar(-1), Scalar(1)));
}

template <typename Scalar>
[[nodiscard]] CARTAN_CONSTEXPR_CMATH Scalar safe_asin(Scalar x)
{
    return std::asin(std::clamp(x, Scalar(-1), Scalar(1)));
}

}

#endif
