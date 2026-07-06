#ifndef HPP_GUARD_CARTAN_FRAMES_FRAMED_WRENCH_H
#define HPP_GUARD_CARTAN_FRAMES_FRAMED_WRENCH_H

#include "cartan/frames/transform.h"

namespace cartan
{

/// Compile-time frame-tagged wrapper over wrench (6-vector).
/// Moment-first [moment; force] ordering consistent with omega-first twist convention.
/// Frame tag indicates the frame in which the wrench is expressed.
/// Zero runtime overhead: aggregate struct forwarding to m_value.
template <typename Frame, typename Scalar = double>
struct framed_wrench
{
    vector6<Scalar> m_value;

    /// Access moment component (first 3 elements).
    auto moment() const
    {
        return m_value.template head<3>();
    }

    /// Access force component (last 3 elements).
    auto force() const
    {
        return m_value.template tail<3>();
    }

    /// Construct from separate moment and force vectors.
    static framed_wrench from_moment_force(
        const vector3<Scalar>& m, const vector3<Scalar>& f)
    {
        vector6<Scalar> v;
        v.template head<3>() = m;
        v.template tail<3>() = f;
        return framed_wrench{v};
    }
};

/// Coadjoint map: transforms a wrench from frame To to frame From.
/// CoAd_T * W = (Ad_{T^{-1}})^T * W.
/// Frame enforcement is structural: wrench Frame must match transform's To.
template <typename From, typename To, typename Scalar, lie_group_policy Policy>
framed_wrench<From, Scalar>
coadjoint_map(const transform<From, To, Scalar, Policy>& T,
              const framed_wrench<To, Scalar>& w)
{
    matrix6<Scalar> CoAd = T.m_value.coadjoint();
    vector6<Scalar> result = CoAd * w.m_value;
    return framed_wrench<From, Scalar>{result};
}

}

#endif
