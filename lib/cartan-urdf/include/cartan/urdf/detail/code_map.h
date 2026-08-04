#ifndef HPP_GUARD_CARTAN_URDF_DETAIL_CODE_MAP_H
#define HPP_GUARD_CARTAN_URDF_DETAIL_CODE_MAP_H

/// Translation of the description reader's diagnostics into the loader's own
/// failure kinds and source locations.

#include "cartan/urdf/error.h"

#include <meios/diagnostic/load_error.h>
#include <meios/diagnostic/diagnostic_code.h>
#include <meios/diagnostic/source_location.h>

#include <string>
#include <utility>
#include <string_view>

namespace cartan::detail
{

/// The left column is the description reader's diagnostic taxonomy; the codes
/// listed below the mapped ones describe conditions this loader has no kind
/// for, and a caller learns the code itself rather than a nearest guess.
inline urdf_failure failure_from_code(meios::diagnostic_code code)
{
    switch (code)
    {
    case meios::diagnostic_code::xml_parse_error:
        return urdf_failure::malformed_xml;
    case meios::diagnostic_code::unknown_joint_type:
        return urdf_failure::unsupported_joint_type;
    case meios::diagnostic_code::undeclared_link:
        return urdf_failure::unknown_parent_link;
    case meios::diagnostic_code::invalid_topology:
        return urdf_failure::unknown_link_reference;
    case meios::diagnostic_code::multiple_parents:
        return urdf_failure::multi_parent_link;
    case meios::diagnostic_code::no_root_cycle:
        return urdf_failure::cyclic_kinematic_tree;
    case meios::diagnostic_code::link_on_cycle:
        return urdf_failure::cyclic_kinematic_tree;
    case meios::diagnostic_code::additional_root:
        return urdf_failure::branched_kinematic_tree;
    case meios::diagnostic_code::duplicate_name:
        return urdf_failure::duplicate_name;
    case meios::diagnostic_code::invalid_number:
        return urdf_failure::non_finite_value;
    case meios::diagnostic_code::missing_limit:
        return urdf_failure::missing_joint_limit;
    case meios::diagnostic_code::zero_axis:
        return urdf_failure::zero_axis;
    case meios::diagnostic_code::unspecified:
    case meios::diagnostic_code::cannot_open:
    case meios::diagnostic_code::non_robot_root:
    case meios::diagnostic_code::unreachable_link:
    case meios::diagnostic_code::duplicate_attribute:
    case meios::diagnostic_code::additional_root_element:
    case meios::diagnostic_code::trailing_content:
    case meios::diagnostic_code::comment_interrupting:
    case meios::diagnostic_code::undefined_material:
    case meios::diagnostic_code::unresolved_asset:
    case meios::diagnostic_code::malformed_asset_uri:
    case meios::diagnostic_code::lfs_pointer_asset:
    case meios::diagnostic_code::asset_write_failed:
    case meios::diagnostic_code::duplicate_asset_entry:
    case meios::diagnostic_code::unsupported_uri_scheme:
    case meios::diagnostic_code::unsupported_uri_authority:
    case meios::diagnostic_code::uncontained_asset:
    case meios::diagnostic_code::undefined_property:
    case meios::diagnostic_code::unresolved_find:
    case meios::diagnostic_code::unresolved_arg:
    case meios::diagnostic_code::unresolved_env:
    case meios::diagnostic_code::unknown_substitution:
    case meios::diagnostic_code::unterminated_substitution:
    case meios::diagnostic_code::expression_error:
    case meios::diagnostic_code::unsupported_expression:
    case meios::diagnostic_code::unresolved_include:
    case meios::diagnostic_code::xacro_parse_error:
    case meios::diagnostic_code::xacro_structural_error:
    case meios::diagnostic_code::expansion_budget_exceeded:
    case meios::diagnostic_code::vector_arity:
    case meios::diagnostic_code::unknown_element:
    case meios::diagnostic_code::unknown_attribute:
    case meios::diagnostic_code::extension_ignored:
    case meios::diagnostic_code::unsupported_version:
    case meios::diagnostic_code::empty_name:
    case meios::diagnostic_code::dangling_mimic:
    case meios::diagnostic_code::no_links:
    case meios::diagnostic_code::missing_required_field:
    case meios::diagnostic_code::missing_joint_type:
    case meios::diagnostic_code::missing_geometry:
    case meios::diagnostic_code::unknown_geometry_shape:
    case meios::diagnostic_code::invalid_mass:
    case meios::diagnostic_code::invalid_inertia:
        break;
    }
    return urdf_failure::unknown_error;
}

/// The reader also reports a column; this loader's location carries the name
/// of the element instead, which the caller supplies.
inline urdf_source_location location_of(const meios::source_location& at, std::string_view element)
{
    return urdf_source_location{at.file.string(), at.line, std::string(element)};
}

inline urdf_error failure_from(const meios::load_error& err)
{
    const urdf_failure kind = failure_from_code(err.code);
    std::string detail = err.message;
    if (kind == urdf_failure::unknown_error)
    {
        detail += " [";
        detail += meios::to_string(err.code);
        detail += "]";
    }
    return urdf_error{
        .kind = kind,
        .detail = std::move(detail),
        .location = location_of(err.loc, "document")};
}

}

#endif
