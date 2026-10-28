#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_STATE_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_STATE_H

/// Joint state (position and optional velocity) for kinematic chains.
///
/// Parameterized by joint count N (fixed or cartan::dynamic) and scalar type.

#include "cartan/serial/chain/storage_trait.h"

#include <Eigen/Dense>

#include <limits>
#include <optional>
#include <type_traits>

namespace cartan
{

/// Joint state holding position vector and optional velocity vector.
/// For fixed N: uses Eigen::Vector<Scalar, N>.
/// For dynamic: uses Eigen::VectorX<Scalar>.
template <typename Scalar = double, int N = dynamic>
struct joint_state
{
    static_assert(std::is_floating_point_v<Scalar>, "joint_state requires a floating-point Scalar type");
    using position_type = std::conditional_t<
        N == dynamic,
        Eigen::VectorX<Scalar>,
        Eigen::Vector<Scalar, N>>;

    using velocity_type = std::conditional_t<
        N == dynamic,
        Eigen::VectorX<Scalar>,
        Eigen::Vector<Scalar, N>>;

    position_type position;                   ///< Joint positions
    std::optional<velocity_type> velocity{};  ///< Joint velocities (optional)

    /// Create a joint state from position only (no velocity).
    static joint_state from_position(const position_type& q)
    {
        joint_state js;
        js.position = q;
        return js;
    }

    /// Number of joints in this state.
    int num_joints() const
    {
        return static_cast<int>(position.size());
    }
};

namespace detail
{

/// Poison default for a joint vector that no solver has populated yet: NaN-filled
/// for a fixed-size chain, empty for a dynamic one. A NaN sentinel makes an
/// accidental read fail loudly -- it propagates through arithmetic and, unlike a
/// large finite value, survives angle wrapping -- instead of masquerading as the
/// plausible all-zero home configuration. An Eigen fixed-size vector's default
/// constructor leaves its coefficients indeterminate, so a member holding one
/// needs this default explicitly.
template <typename Scalar, int N>
typename joint_state<Scalar, N>::position_type poison_joint_position()
{
    using position_type = typename joint_state<Scalar, N>::position_type;
    if constexpr (N == dynamic)
    {
        return position_type{};
    }
    else
    {
        return position_type::Constant(std::numeric_limits<Scalar>::quiet_NaN());
    }
}

/// The same poison over a known joint count, for a dynamic vector whose size a
/// caller reads even where its coefficients were never measured.
template <typename Scalar, int N>
typename joint_state<Scalar, N>::position_type poison_joint_position(int joints)
{
    return joint_state<Scalar, N>::position_type::Constant(
        joints, std::numeric_limits<Scalar>::quiet_NaN());
}

}

}

#endif
