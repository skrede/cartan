#ifndef HPP_GUARD_CARTAN_DETAIL_EPSILON_H
#define HPP_GUARD_CARTAN_DETAIL_EPSILON_H

#include <cartan/detail/compat.h>

#include <cmath>
#include <limits>
#include <type_traits>

namespace cartan::detail
{

template <typename Scalar>
struct epsilon_traits
{
    static_assert(std::is_floating_point_v<Scalar>);

    static constexpr Scalar value = std::numeric_limits<Scalar>::epsilon();
#if CARTAN_HAS_CONSTEXPR_CMATH
    static constexpr Scalar sqrt_value = std::sqrt(std::numeric_limits<Scalar>::epsilon());
#else
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
#endif
};

template <typename Scalar>
inline constexpr Scalar epsilon_v = epsilon_traits<Scalar>::value;

template <typename Scalar>
inline constexpr Scalar sqrt_epsilon_v = epsilon_traits<Scalar>::sqrt_value;

}

#endif
