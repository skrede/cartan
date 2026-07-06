#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_STATE_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_STATE_H

/// Joint state (position and optional velocity) for kinematic chains.
///
/// Parameterized by joint count N (fixed or cartan::dynamic) and scalar type.

#include "cartan/serial/chain/storage_trait.h"

#include <Eigen/Dense>

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

}

#endif
