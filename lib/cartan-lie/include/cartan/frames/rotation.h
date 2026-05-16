#ifndef HPP_GUARD_CARTAN_FRAMES_ROTATION_H
#define HPP_GUARD_CARTAN_FRAMES_ROTATION_H

#include "cartan/lie/so3.h"
#include "cartan/lie/policy.h"

#include <string>
#include "cartan/expected.h"

namespace cartan
{

/// Compile-time frame-tagged wrapper over SO(3).
/// From/To are unconstrained frame tag types (empty struct, enum class, int NTTP wrapper, etc.).
/// Zero runtime overhead: aggregate struct forwarding to m_value.
/// Frame composition is structural: rotation<A,B> * rotation<B,C> -> rotation<A,C>.
template <typename From, typename To, typename Scalar = double, lie_group_policy Policy = strict_policy>
struct rotation
{
    so3<Scalar, Policy> m_value;

    /// Compose with another rotation whose From frame matches our To frame.
    /// Structural frame matching: operator* only accepts rotation<To, C, ...>.
    template <typename C, lie_group_policy P2>
    [[nodiscard]] auto operator*(const rotation<To, C, Scalar, P2>& rhs) const
        -> rotation<From, C, Scalar, stricter_policy<Policy, P2>>
    {
        return rotation<From, C, Scalar, stricter_policy<Policy, P2>>{
            m_value * rhs.m_value
        };
    }

    /// Inverse flips frame tags: rotation<A,B>.inverse() -> rotation<B,A>.
    [[nodiscard]] rotation<To, From, Scalar, Policy> inverse() const
    {
        return rotation<To, From, Scalar, Policy>{m_value.inverse()};
    }

    /// Convert to 3x3 rotation matrix.
    [[nodiscard]] matrix3<Scalar> matrix() const
    {
        return m_value.matrix();
    }

    /// Access the internal quaternion (read-only).
    [[nodiscard]] const quaternion<Scalar>& quaternion_ref() const
    {
        return m_value.quaternion_ref();
    }

    /// Logarithmic map: SO(3) -> so(3).
    [[nodiscard]] vector3<Scalar> log() const
    {
        return m_value.log();
    }

    /// Rotate a 3D vector.
    [[nodiscard]] vector3<Scalar> act(const vector3<Scalar>& v) const
    {
        return m_value.act(v);
    }

    /// Identity rotation (no rotation).
    [[nodiscard]] static rotation identity()
    {
        return rotation{so3<Scalar, Policy>::identity()};
    }

    /// Construct from 3x3 rotation matrix with validation.
    [[nodiscard]] static cartan::expected<rotation, std::string>
    from_matrix(const matrix3<Scalar>& R)
    {
        auto result = so3<Scalar, Policy>::from_matrix(R);
        if (!result.has_value())
        {
            return cartan::unexpected(result.error());
        }
        return rotation{result.value()};
    }

    /// Construct from quaternion with validation.
    [[nodiscard]] static cartan::expected<rotation, std::string>
    from_quaternion(const quaternion<Scalar>& q)
    {
        auto result = so3<Scalar, Policy>::from_quaternion(q);
        if (!result.has_value())
        {
            return cartan::unexpected(result.error());
        }
        return rotation{result.value()};
    }
};

}

#endif
