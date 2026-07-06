#ifndef HPP_GUARD_CARTAN_URDF_SCHEMA_H
#define HPP_GUARD_CARTAN_URDF_SCHEMA_H

/// Tree-preserving internal model of a parsed URDF document.
///
/// parsed_model holds the full link and joint tree exactly as it appeared in
/// the source URDF, with parent and child link names resolved by string.
/// The model is the intermediate representation that the chain extractor
/// (a separate step) walks to produce a strictly-serial kinematic_chain.
/// Mobility, dynamics, and connectivity invariants beyond per-element well-
/// formedness are not enforced at this layer.

#include "cartan/types.h"
#include "cartan/lie/se3.h"

#include <string>
#include <vector>
#include <optional>

namespace cartan
{

/// Joint kinds the loader recognizes. Maps directly to the URDF joint type
/// attribute. Joint types outside this set (screw, floating, planar, mimic,
/// gearbox) are rejected at parse time with urdf_failure::unsupported_joint_type
/// or urdf_failure::mimic_joint_unsupported as appropriate.
enum class parsed_joint_kind
{
    fixed,
    revolute,
    continuous,
    prismatic
};

/// Inertial properties of a link as parsed from URDF. Aggregate-initializable.
/// mass is the link mass in kilograms; com is the center-of-mass position
/// expressed in the link frame; inertia is the symmetric 3x3 inertia tensor
/// (ixx, ixy, ixz, iyy, iyz, izz) about the link's COM in URDF convention.
template <typename Scalar = double>
struct parsed_inertial
{
    Scalar mass;
    vector3<Scalar> com;
    matrix3<Scalar> inertia;
};

/// A link as parsed from URDF. inertial is unset when the link declares no
/// <inertial> element.
template <typename Scalar = double>
struct parsed_link
{
    std::string name;
    std::optional<parsed_inertial<Scalar>> inertial{};
};

/// A joint as parsed from URDF. axis defaults to +X, matching the URDF spec
/// default of (1,0,0) when the <axis> element is absent on a revolute or
/// prismatic joint. origin is the transform from parent to joint frame at zero
/// joint position. position_min and position_max are unset when the URDF omits
/// <limit lower upper>; the chain extractor fills +/-infinity for continuous
/// joints and rejects a revolute or prismatic joint that leaves them unset.
/// velocity_max and effort_max mirror <limit velocity> and <limit effort> and
/// are preserved on the model for forward compatibility with the dynamics
/// module.
template <typename Scalar = double>
struct parsed_joint
{
    std::string name;
    parsed_joint_kind kind{parsed_joint_kind::fixed};
    std::string parent_link;
    std::string child_link;
    vector3<Scalar> axis{vector3<Scalar>::UnitX()};
    se3<Scalar> origin{se3<Scalar>::identity()};
    std::optional<Scalar> position_min{};
    std::optional<Scalar> position_max{};
    std::optional<Scalar> velocity_max{};
    std::optional<Scalar> effort_max{};
};

/// Parsed URDF document. robot_name carries the <robot name="..."> attribute;
/// links and joints preserve the source order. No connectivity validation is
/// performed here beyond per-element parse checks.
template <typename Scalar = double>
struct parsed_model
{
    std::string robot_name;
    std::vector<parsed_link<Scalar>> links;
    std::vector<parsed_joint<Scalar>> joints;
};

}

#endif
