#ifndef HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_AXIS_ROTATION_H
#define HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_AXIS_ROTATION_H

#include "cartan/types.h"

#include <cmath>

namespace cartan::detail
{

/// Rotate p about the line through q with direction omega by theta
/// (Rodrigues' rotation formula).
template <typename Scalar>
vector3<Scalar> rotate_point_about_axis(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& q,
    const vector3<Scalar>& p,
    Scalar theta)
{
    vector3<Scalar> v = p - q;
    Scalar ct = std::cos(theta);
    Scalar st = std::sin(theta);
    return q + ct * v
        + (Scalar(1) - ct) * omega.dot(v) * omega
        + st * omega.cross(v);
}

}

#endif
