#ifndef HPP_GUARD_LIEPP_DETAIL_EPSILON_H
#define HPP_GUARD_LIEPP_DETAIL_EPSILON_H

#include <cmath>
#include <limits>
#include <type_traits>

namespace liepp::detail
{

template <typename Scalar>
struct epsilon_traits
{
    static_assert(std::is_floating_point_v<Scalar>);

    static constexpr Scalar value = std::numeric_limits<Scalar>::epsilon();
    static constexpr Scalar sqrt_value = []
    {
        if consteval
        {
            if constexpr (std::is_same_v<Scalar, float>)
                return 3.4526698e-4f;
            else
                return 1.4901161193847656e-8;
        }
        else
        {
            return std::sqrt(std::numeric_limits<Scalar>::epsilon());
        }
    }();
};

template <typename Scalar>
inline constexpr Scalar epsilon_v = epsilon_traits<Scalar>::value;

template <typename Scalar>
inline constexpr Scalar sqrt_epsilon_v = epsilon_traits<Scalar>::sqrt_value;

}

#endif
