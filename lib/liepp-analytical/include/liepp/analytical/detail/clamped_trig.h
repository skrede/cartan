#ifndef HPP_GUARD_LIEPP_ANALYTICAL_DETAIL_CLAMPED_TRIG_H
#define HPP_GUARD_LIEPP_ANALYTICAL_DETAIL_CLAMPED_TRIG_H

#include <algorithm>
#include <cmath>

namespace liepp::detail
{

template <typename Scalar>
[[nodiscard]] constexpr Scalar safe_acos(Scalar x)
{
    return std::acos(std::clamp(x, Scalar(-1), Scalar(1)));
}

template <typename Scalar>
[[nodiscard]] constexpr Scalar safe_asin(Scalar x)
{
    return std::asin(std::clamp(x, Scalar(-1), Scalar(1)));
}

}

#endif
