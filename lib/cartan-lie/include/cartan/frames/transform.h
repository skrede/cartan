#ifndef HPP_GUARD_CARTAN_FRAMES_TRANSFORM_H
#define HPP_GUARD_CARTAN_FRAMES_TRANSFORM_H

#include "cartan/lie/se3.h"
#include "cartan/lie/policy.h"
#include "cartan/lie/lie_failure.h"

#include "cartan/expected.h"

namespace cartan
{

/// Compile-time frame-tagged wrapper over SE(3).
/// From/To are unconstrained frame tag types.
/// Zero runtime overhead: aggregate struct forwarding to m_value.
/// Frame composition is structural: transform<A,B> * transform<B,C> -> transform<A,C>.
template <typename From, typename To, typename Scalar = double, lie_group_policy Policy = strict_policy>
struct transform
{
    se3<Scalar, Policy> m_value;

    /// Compose with another transform whose From frame matches our To frame.
    template <typename C, lie_group_policy P2>
    [[nodiscard]] auto operator*(const transform<To, C, Scalar, P2>& rhs) const
        -> transform<From, C, Scalar, stricter_policy<Policy, P2>>
    {
        return transform<From, C, Scalar, stricter_policy<Policy, P2>>{
            m_value * rhs.m_value
        };
    }

    /// Inverse flips frame tags: transform<A,B>.inverse() -> transform<B,A>.
    [[nodiscard]] transform<To, From, Scalar, Policy> inverse() const
    {
        return transform<To, From, Scalar, Policy>{m_value.inverse()};
    }

    /// Convert to 4x4 homogeneous transformation matrix.
    [[nodiscard]] matrix4<Scalar> matrix() const
    {
        return m_value.matrix();
    }

    /// Access the rotation component.
    [[nodiscard]] const so3<Scalar, Policy>& rotation() const
    {
        return m_value.rotation();
    }

    /// Access the translation component.
    [[nodiscard]] const vector3<Scalar>& translation() const
    {
        return m_value.translation();
    }

    /// Logarithmic map: SE(3) -> se(3).
    [[nodiscard]] vector6<Scalar> log() const
    {
        return m_value.log();
    }

    /// Transform a 3D point: R * p + t.
    [[nodiscard]] vector3<Scalar> act(const vector3<Scalar>& p) const
    {
        return m_value.act(p);
    }

    /// Identity transform (no rotation, no translation).
    [[nodiscard]] static transform identity()
    {
        return transform{se3<Scalar, Policy>::identity()};
    }

    /// Construct from 4x4 homogeneous matrix with validation.
    [[nodiscard]] static cartan::expected<transform, lie_failure>
    from_matrix(const matrix4<Scalar>& T)
    {
        auto result = se3<Scalar, Policy>::from_matrix(T);
        if (!result.has_value())
        {
            return cartan::unexpected(result.error());
        }
        return transform{*result};
    }
};

}

#endif
