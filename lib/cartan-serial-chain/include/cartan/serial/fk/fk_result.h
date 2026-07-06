#ifndef HPP_GUARD_CARTAN_SERIAL_FK_FK_RESULT_H
#define HPP_GUARD_CARTAN_SERIAL_FK_FK_RESULT_H

/// Forward kinematics result with intermediate product caching.
///
/// Stores the end-effector pose and all intermediate products T_i from
/// the Product of Exponentials computation. Intermediate products are
/// reused by Jacobian computations (space and body Jacobians).
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138:
///   T(q) = exp([S1]q1) * exp([S2]q2) * ... * exp([Sn]qn) * M
///
/// Intermediate product T_i = exp([S1]q1) * ... * exp([Si]qi).

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/storage_trait.h"

#include <type_traits>

namespace cartan
{

/// Result of forward kinematics via Product of Exponentials.
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

    // Default construction yields the identity element for se3 and, for a
    // fixed-N array, N identities; for a dynamic chain the vector starts empty.
    se3<Scalar> end_effector{};  ///< End-effector pose T(q) = T_1...T_n * M
    intermediate_storage intermediates{};  ///< Partial products T_i for Jacobian reuse

    /// Number of joints reflected in this result.
    [[nodiscard]] int num_joints() const
    {
        return static_cast<int>(intermediates.size());
    }
};

}

#endif
