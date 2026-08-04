#ifndef HPP_GUARD_CARTAN_URDF_ERROR_H
#define HPP_GUARD_CARTAN_URDF_ERROR_H

/// Diagnostic types for the URDF loader.
///
/// All URDF loader entry points return cartan::expected with urdf_error in the
/// failure channel. urdf_error carries a typed kind, a free-form detail
/// string, and an optional source location populated by the parser when the
/// failure can be tied to a specific element in the input XML.

#include <meios/diagnostic/diagnostic_code.h>

#include <string>
#include <optional>

namespace cartan
{

/// Failure modes for the URDF loader.
///
/// A failure raised while the description is being read carries both a source
/// location and the reader's own diagnostic code, which rides alongside kind
/// so a condition this taxonomy has no name for still reaches the caller as
/// what the reader actually reported. A failure raised afterwards by the chain
/// extractor (branched_kinematic_tree, link_not_found, sdf_not_supported,
/// missing_joint_limit, zero_axis, invalid_joint_limit, tool_link_unreachable)
/// carries neither: there is no element left to point at and no reader code to
/// quote.
///
/// The loader is the library's only untrusted-input surface, so every spec
/// violation is a strict rejection whose detail names the offending joint or
/// link. A continuous joint without a <limit> is not a violation (it is
/// unbounded by definition) and loads normally.
enum class urdf_failure
{
    malformed_xml,             ///< XML was not well-formed; the reader reports the offset.
    unsupported_joint_type,    ///< Joint type token is not one of fixed, revolute, continuous, prismatic.
    unknown_link_reference,    ///< Joint references a link name that was not declared.
    unknown_parent_link,       ///< Joint's parent link is not in the link set.
    branched_kinematic_tree,   ///< The tree does not reduce to one serial chain: more than one root, or more than one leaf after fixed-joint merge.
    link_not_found,            ///< A load_options-requested base or tool link is missing from the URDF.
    mimic_joint_unsupported,   ///< Joint declares a mimic relation; coupled-joint kinematics is deferred.
    inertial_singular,         ///< Inertial declares a non-physical mass or inertia matrix.
    sdf_not_supported,         ///< SDF loading is not implemented; only URDF is currently supported.
    cyclic_kinematic_tree,     ///< A joint is a self-loop (parent == child) or the tree contains a cycle.
    missing_joint_limit,       ///< A revolute or prismatic joint omits the required <limit lower upper>.
    invalid_joint_limit,       ///< A joint <limit> is self-contradictory: reversed bounds, or a negative velocity or effort.
    zero_axis,                 ///< A joint <axis> has zero magnitude and cannot be normalized.
    non_finite_value,          ///< A numeric attribute parsed to NaN or infinity, or is not representable in the chain's scalar type.
    duplicate_name,            ///< A link or joint name is declared more than once.
    multi_parent_link,         ///< A link is the child of more than one joint (non-tree topology).
    tool_link_unreachable,     ///< The requested tool link is not reached by the serial walk.
    unknown_error              ///< Catch-all default; should not occur in normal operation.
};

/// Location of a URDF failure inside the source XML.
///
/// file is the absolute path passed to the parser (or empty for string-based
/// entry points). line is one-based, as the reader reports it. element names
/// the URDF element the failure applies to (e.g. "joint", "link",
/// "inertial"); it is empty when no specific element can be identified.
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
    std::optional<meios::diagnostic_code> meios_code{};
};

}

#endif
