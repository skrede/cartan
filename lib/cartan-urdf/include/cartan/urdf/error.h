#ifndef HPP_GUARD_CARTAN_URDF_ERROR_H
#define HPP_GUARD_CARTAN_URDF_ERROR_H

/// Diagnostic types for the URDF loader.
///
/// All URDF loader entry points return std::expected with urdf_error in the
/// failure channel. urdf_error carries a typed kind, a free-form detail
/// string, and an optional source location populated by the parser when the
/// failure can be tied to a specific element in the input XML.

#include <string>
#include <optional>

namespace cartan::urdf
{

/// Failure modes for the URDF loader.
///
/// Parse-time failures (malformed_xml, unsupported_joint_type,
/// unknown_link_reference, unknown_parent_link, mimic_joint_unsupported,
/// inertial_singular) fill urdf_error::location when they can be tied to a
/// specific element. Post-parse failures (branched_kinematic_tree,
/// link_not_found, sdf_not_supported) leave location unset.
enum class urdf_failure
{
    malformed_xml,             ///< XML was not well-formed; pugixml reports the offset.
    unsupported_joint_type,    ///< Joint type token is not one of fixed, revolute, continuous, prismatic.
    unknown_link_reference,    ///< Joint references a link name that was not declared.
    unknown_parent_link,       ///< Joint's parent link is not in the link set.
    branched_kinematic_tree,   ///< More than one leaf after fixed-joint merge; no single chain extractable.
    link_not_found,            ///< A load_options-requested base or tool link is missing from the URDF.
    mimic_joint_unsupported,   ///< Joint declares a mimic relation; coupled-joint kinematics is deferred.
    inertial_singular,         ///< Inertial declares a non-physical mass or inertia matrix.
    sdf_not_supported,         ///< SDF loading is not implemented; only URDF is currently supported.
    unknown_error              ///< Catch-all default; should not occur in normal operation.
};

/// Location of a URDF failure inside the source XML.
///
/// file is the absolute path passed to the parser (or empty for string-based
/// entry points). line is one-based, computed from the byte offset returned
/// by pugixml's xml_parse_result. element names the URDF element the failure
/// applies to (e.g. "joint", "link", "inertial"); it is empty when no
/// specific element can be identified.
struct urdf_source_location
{
    std::string file{};
    int line{0};
    std::string element{};
};

/// URDF loader diagnostic. kind classifies the failure mode; detail is a
/// human-readable explanation that may name the offending link or joint;
/// location is populated for parse-time failures and left unset for
/// post-parse failures.
struct urdf_error
{
    urdf_failure kind{urdf_failure::unknown_error};
    std::string detail{};
    std::optional<urdf_source_location> location{};
};

}

#endif
