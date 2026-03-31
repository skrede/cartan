#ifndef HPP_GUARD_LIEPP_KINEMATICS_FK_RESULT_H
#define HPP_GUARD_LIEPP_KINEMATICS_FK_RESULT_H

/// @file fk_result.h
/// @brief Forward kinematics result with intermediate product caching.
///
/// Stores the end-effector pose and all intermediate products T_i from
/// the Product of Exponentials computation. Intermediate products are
/// reused by Jacobian computations (space and body Jacobians).
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138:
///   T(q) = exp([S1]q1) * exp([S2]q2) * ... * exp([Sn]qn) * M
///
/// Intermediate product T_i = exp([S1]q1) * ... * exp([Si]qi).

#include "liepp/lie/se3.h"
#include "liepp/chain/storage_trait.h"

#include <utility>
#include <type_traits>

namespace liepp
{

namespace detail
{

/// Create a std::array of N identity se3 elements.
template <typename Scalar, std::size_t... Is>
std::array<se3<Scalar>, sizeof...(Is)> make_identity_array(std::index_sequence<Is...>)
{
    return {{ ((void)Is, se3<Scalar>::identity())... }};
}

template <typename Scalar, int N>
auto make_intermediate_storage()
{
    if constexpr (N == dynamic)
    {
        return std::vector<se3<Scalar>>{};
    }
    else
    {
        return make_identity_array<Scalar>(std::make_index_sequence<static_cast<std::size_t>(N)>{});
    }
}

} // namespace detail

/// Result of forward kinematics via Product of Exponentials.
/// @tparam N      Number of joints (compile-time), or liepp::dynamic.
/// @tparam Scalar Floating-point type.
///
/// intermediates[i] holds the partial product exp([S1]q1) * ... * exp([S_{i+1}]q_{i+1}).
/// end_effector = intermediates[n-1] * M (home configuration).
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138.
template <typename Scalar = double, int N = dynamic>
struct fk_result
{
    static_assert(std::is_floating_point_v<Scalar>, "fk_result requires a floating-point Scalar type");
    using intermediate_storage = detail::storage_t<N, se3<Scalar>>;

    se3<Scalar> end_effector{se3<Scalar>::identity()};  ///< End-effector pose T(q) = T_1...T_n * M
    intermediate_storage intermediates{detail::make_intermediate_storage<Scalar, N>()};  ///< Partial products T_i for Jacobian reuse

    /// Number of joints reflected in this result.
    [[nodiscard]] int num_joints() const
    {
        return static_cast<int>(intermediates.size());
    }
};

} // namespace liepp

#endif
