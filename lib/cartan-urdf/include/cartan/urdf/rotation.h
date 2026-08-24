#ifndef HPP_GUARD_CARTAN_URDF_ROTATION_H
#define HPP_GUARD_CARTAN_URDF_ROTATION_H

/// The roll-pitch-yaw lift a URDF origin needs, shared by every reader that
/// stages a document into the parsed model.

#include "cartan/lie/so3.h"

#include "cartan/types.h"

namespace cartan::detail
{

/// Build an SO(3) rotation from URDF roll-pitch-yaw angles. URDF convention:
/// R = Rz(yaw) * Ry(pitch) * Rx(roll). Each axis rotation is built via
/// so3::exp so the implementation reuses the validated cartan exponential map.
template <typename Scalar>
so3<Scalar> rotation_from_rpy(Scalar roll, Scalar pitch, Scalar yaw)
{
    auto rx = so3<Scalar>::exp(vector3<Scalar>(roll, Scalar(0), Scalar(0)));
    auto ry = so3<Scalar>::exp(vector3<Scalar>(Scalar(0), pitch, Scalar(0)));
    auto rz = so3<Scalar>::exp(vector3<Scalar>(Scalar(0), Scalar(0), yaw));
    return rz * ry * rx;
}

}

#endif
