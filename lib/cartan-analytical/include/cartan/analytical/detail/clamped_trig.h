#ifndef HPP_GUARD_LIEPP_ANALYTICAL_DETAIL_CLAMPED_TRIG_H
#define HPP_GUARD_LIEPP_ANALYTICAL_DETAIL_CLAMPED_TRIG_H

#include <liepp/detail/compat.h>

#include <algorithm>
#include <cmath>

namespace liepp::detail
{

template <typename Scalar>
[[nodiscard]] LIEPP_CONSTEXPR_CMATH Scalar safe_acos(Scalar x)
{
    return std::acos(std::clamp(x, Scalar(-1), Scalar(1)));
}

template <typename Scalar>
[[nodiscard]] LIEPP_CONSTEXPR_CMATH Scalar safe_asin(Scalar x)
{
    return std::asin(std::clamp(x, Scalar(-1), Scalar(1)));
}

}

#endif
