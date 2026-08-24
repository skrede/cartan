#ifndef HPP_GUARD_CARTAN_URDF_DETAIL_MODEL_STAGE_H
#define HPP_GUARD_CARTAN_URDF_DETAIL_MODEL_STAGE_H

/// Translation of the description reader's records into the parsed model the
/// chain extractor walks. The reader is fixed at double, so every value the
/// chain consumes is narrowed here, and every narrowing is checked: a staging
/// function returns the name of the first field the chain's scalar type cannot
/// hold, or nullopt when the whole record was representable.

#include "cartan/urdf/detail/narrowing.h"

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
#include <string_view>

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
std::optional<std::string_view> stage_origin(const meios::transform<double>& origin,
                                             se3<Scalar>& out)
{
    vector3<Scalar> translation;
    if (narrow_vector(origin.translation, translation)) { return "<origin xyz>"; }
    Scalar roll{};
    Scalar pitch{};
    Scalar yaw{};
    if (narrow_into(origin.rotation.roll, roll) || narrow_into(origin.rotation.pitch, pitch)
        || narrow_into(origin.rotation.yaw, yaw))
    {
        return "<origin rpy>";
    }
    out = se3<Scalar>(rotation_from_rpy<Scalar>(roll, pitch, yaw), translation);
    return std::nullopt;
}

template <typename Scalar>
std::optional<std::string_view> stage_inertia(const meios::inertia<double>& tensor,
                                              matrix3<Scalar>& out)
{
    matrix3<double> wide;
    wide << tensor.ixx, tensor.ixy, tensor.ixz,
        tensor.ixy, tensor.iyy, tensor.iyz,
        tensor.ixz, tensor.iyz, tensor.izz;
    for (Eigen::Index i = 0; i < wide.size(); ++i)
    {
        if (narrow_into(wide(i), out(i))) { return "<inertia>"; }
    }
    return std::nullopt;
}

template <typename Scalar>
std::optional<std::string_view> stage_link(const meios::link<double>& node,
                                           parsed_link<Scalar>& out)
{
    out.name = node.name;
    if (!node.body) { return std::nullopt; }
    parsed_inertial<Scalar> body{};
    if (narrow_into(node.body->mass, body.mass)) { return "<inertial mass>"; }
    if (narrow_vector(node.body->origin.translation, body.com)) { return "<inertial origin>"; }
    if (const auto field = stage_inertia(node.body->tensor, body.inertia)) { return field; }
    out.inertial = body;
    return std::nullopt;
}

template <typename Scalar>
std::optional<std::string_view> stage_limit(double value, std::optional<Scalar>& out,
                                            std::string_view field)
{
    Scalar narrowed{};
    if (narrow_into(value, narrowed)) { return field; }
    out = narrowed;
    return std::nullopt;
}

template <typename Scalar>
std::optional<std::string_view> stage_limits(const meios::joint_limits<double>& limits,
                                             parsed_joint<Scalar>& out)
{
    if (const auto f = stage_limit(limits.lower, out.position_min, "<limit lower>")) { return f; }
    if (const auto f = stage_limit(limits.upper, out.position_max, "<limit upper>")) { return f; }
    if (const auto f = stage_limit(limits.velocity, out.velocity_max, "<limit velocity>")) { return f; }
    if (const auto f = stage_limit(limits.effort, out.effort_max, "<limit effort>")) { return f; }
    return std::nullopt;
}

template <typename Scalar>
std::optional<std::string_view> stage_joint(const meios::joint<double>& edge,
                                            parsed_joint_kind kind,
                                            parsed_joint<Scalar>& out)
{
    out.name = edge.name;
    out.kind = kind;
    out.parent_link = edge.parent;
    out.child_link = edge.child;
    if (narrow_vector(edge.axis, out.axis)) { return "<axis>"; }
    if (const auto field = stage_origin(edge.origin, out.origin)) { return field; }
    if (!edge.limits) { return std::nullopt; }
    return stage_limits(*edge.limits, out);
}

}

#endif
