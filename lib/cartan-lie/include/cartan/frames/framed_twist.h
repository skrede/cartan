#ifndef HPP_GUARD_LIEPP_FRAMES_FRAMED_TWIST_H
#define HPP_GUARD_LIEPP_FRAMES_FRAMED_TWIST_H

#include "liepp/frames/transform.h"

#include "liepp/lie/twist.h"

namespace liepp
{

/// Compile-time frame-tagged wrapper over twist.
/// Frame tag indicates the frame in which the twist is expressed.
/// Zero runtime overhead: aggregate struct forwarding to m_value.
template <typename Frame, typename Scalar = double>
struct framed_twist
{
    twist<Scalar> m_value;

    /// Access angular velocity component.
    [[nodiscard]] const vector3<Scalar>& omega() const
    {
        return m_value.omega;
    }

    /// Access linear velocity component.
    [[nodiscard]] const vector3<Scalar>& v() const
    {
        return m_value.v;
    }

    /// Convert to 6-vector (omega-first).
    [[nodiscard]] vector6<Scalar> to_vector() const
    {
        return m_value.to_vector();
    }

    /// Construct from 6-vector (omega-first).
    [[nodiscard]] static framed_twist from_vector(const vector6<Scalar>& vec)
    {
        return framed_twist{twist<Scalar>::from_vector(vec)};
    }
};

/// Adjoint map: transforms a twist from frame To to frame From.
/// Ad_T * V: if T maps From->To, then the twist expressed in To
/// is mapped to be expressed in From.
/// Frame enforcement is structural: twist Frame must match transform's To.
template <typename From, typename To, typename Scalar, lie_group_policy Policy>
[[nodiscard]] framed_twist<From, Scalar>
adjoint_map(const transform<From, To, Scalar, Policy>& T,
            const framed_twist<To, Scalar>& tw)
{
    matrix6<Scalar> Ad = T.m_value.adjoint();
    vector6<Scalar> result = Ad * tw.to_vector();
    return framed_twist<From, Scalar>::from_vector(result);
}

} // namespace liepp

#endif
