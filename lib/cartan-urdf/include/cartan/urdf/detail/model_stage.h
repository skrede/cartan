#ifndef HPP_GUARD_CARTAN_URDF_DETAIL_MODEL_STAGE_H
#define HPP_GUARD_CARTAN_URDF_DETAIL_MODEL_STAGE_H

/// Translation of the description reader's records into the parsed model the
/// chain extractor walks. The reader is fixed at double, so every value the
/// chain consumes is narrowed here.

#include "cartan/urdf/schema.h"
#include "cartan/urdf/rotation.h"

#include "cartan/lie/se3.h"

#include "cartan/types.h"

#include <meios/records/link.h>
#include <meios/records/joint.h>

#include <meios/math/inertia.h>
#include <meios/math/vector3.h>
#include <meios/math/transform.h>

#include <optional>

namespace cartan::detail
{

inline std::optional<parsed_joint_kind> staged_kind(meios::joint_kind kind)
{
    switch (kind)
    {
    case meios::joint_kind::fixed:
        return parsed_joint_kind::fixed;
    case meios::joint_kind::revolute:
        return parsed_joint_kind::revolute;
    case meios::joint_kind::continuous:
        return parsed_joint_kind::continuous;
    case meios::joint_kind::prismatic:
        return parsed_joint_kind::prismatic;
    case meios::joint_kind::floating:
    case meios::joint_kind::planar:
        break;
    }
    return std::nullopt;
}

template <typename Scalar>
vector3<Scalar> staged_vector(const meios::vector3<double>& v)
{
    return vector3<Scalar>(static_cast<Scalar>(v.x),
                           static_cast<Scalar>(v.y),
                           static_cast<Scalar>(v.z));
}

template <typename Scalar>
se3<Scalar> staged_origin(const meios::transform<double>& origin)
{
    const so3<Scalar> rotation =
        rotation_from_rpy<Scalar>(static_cast<Scalar>(origin.rotation.roll),
                                  static_cast<Scalar>(origin.rotation.pitch),
                                  static_cast<Scalar>(origin.rotation.yaw));
    return se3<Scalar>(rotation, staged_vector<Scalar>(origin.translation));
}

template <typename Scalar>
matrix3<Scalar> staged_inertia(const meios::inertia<double>& tensor)
{
    matrix3<Scalar> out;
    out << static_cast<Scalar>(tensor.ixx), static_cast<Scalar>(tensor.ixy),
        static_cast<Scalar>(tensor.ixz), static_cast<Scalar>(tensor.ixy),
        static_cast<Scalar>(tensor.iyy), static_cast<Scalar>(tensor.iyz),
        static_cast<Scalar>(tensor.ixz), static_cast<Scalar>(tensor.iyz),
        static_cast<Scalar>(tensor.izz);
    return out;
}

template <typename Scalar>
parsed_link<Scalar> staged_link(const meios::link<double>& node)
{
    parsed_link<Scalar> out{};
    out.name = node.name;
    if (node.body)
    {
        out.inertial = parsed_inertial<Scalar>{static_cast<Scalar>(node.body->mass),
                                               staged_vector<Scalar>(node.body->origin.translation),
                                               staged_inertia<Scalar>(node.body->tensor)};
    }
    return out;
}

template <typename Scalar>
parsed_joint<Scalar> staged_joint(const meios::joint<double>& edge, parsed_joint_kind kind)
{
    parsed_joint<Scalar> out{};
    out.name = edge.name;
    out.kind = kind;
    out.parent_link = edge.parent;
    out.child_link = edge.child;
    out.axis = staged_vector<Scalar>(edge.axis);
    out.origin = staged_origin<Scalar>(edge.origin);
    if (edge.limits)
    {
        out.position_min = static_cast<Scalar>(edge.limits->lower);
        out.position_max = static_cast<Scalar>(edge.limits->upper);
        out.velocity_max = static_cast<Scalar>(edge.limits->velocity);
        out.effort_max = static_cast<Scalar>(edge.limits->effort);
    }
    return out;
}

}

#endif
