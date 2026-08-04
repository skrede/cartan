#ifndef HPP_GUARD_CARTAN_URDF_METADATA_H
#define HPP_GUARD_CARTAN_URDF_METADATA_H

/// User-facing types for the top-level URDF loader entry points.
///
/// load_options selects the base and tool link when the URDF tree is
/// ambiguous; an empty string indicates "let the loader auto-detect".
/// urdf_metadata holds the strings and inertial properties the
/// kinematic_chain itself does not carry. urdf_load_result is the success
/// type returned by the future load_urdf overload; in this layer it is
/// declared so downstream code can compose against the final shape, but the
/// chain extractor that fills it lands in a separate step.

#include "cartan/urdf/diagnostic.h"

#include "cartan/types.h"

#include "cartan/serial/chain/kinematic_chain.h"

#include <meios/urdf/load.h>

#include <meios/diagnostic/completeness.h>

#include <string>
#include <vector>
#include <optional>

namespace cartan
{

/// Reading policy for a robot description consumed as kinematics. It differs
/// from the description reader's own default in one place: an unresolvable
/// package:// asset is reported and the load continues, because a kinematic
/// chain reads no meshes and refusing over one would reject descriptions that
/// ship without them.
inline meios::load_options kinematics_description_defaults()
{
    meios::load_options opts{};
    opts.on_missing = meios::missing_asset::warn;
    return opts;
}

/// Optional overrides for the chain extraction step. When base_link or
/// tool_link is empty, the extractor auto-detects the unique root or leaf
/// in the post-fixed-joint-merge tree. When both are non-empty, the
/// extractor extracts the chain bounded by the named links; missing names
/// produce urdf_failure::link_not_found. description is the reader's own
/// options carried through rather than mirrored, so a caller fills args,
/// package_roots and the evaluation policy directly.
struct load_options
{
    load_options()
        : base_link()
        , tool_link()
        , description(kinematics_description_defaults())
    {
    }

    std::string base_link;
    std::string tool_link;
    meios::load_options description;
};

/// Inertial properties of a link, paired with the link's name for cross-
/// referencing by downstream consumers (dynamics, visualization, contact
/// reasoning). Mirrors parsed_inertial structurally but carries the link
/// name so the consumer can use it without keeping the full parsed_model
/// alive.
template <typename Scalar = double>
struct link_inertial
{
    std::string link_name;
    Scalar mass;
    vector3<Scalar> com;
    matrix3<Scalar> inertia;
};

/// Side-table of strings and inertial properties that accompany a loaded
/// kinematic_chain. The chain itself is string-free; everything that needs
/// a name (joint identifiers for diagnostics, link inertials for dynamics,
/// per-joint velocity and effort limits parsed from the URDF) lives here.
/// joint_names is indexed by joint position in the kinematic_chain;
/// velocity_max and effort_max are the same length and align with joint_names.
template <typename Scalar = double>
struct urdf_metadata
{
    std::string base_link_name;
    std::string tool_link_name;
    std::vector<std::string> joint_names;
    std::vector<link_inertial<Scalar>> link_inertials;
    std::vector<std::optional<Scalar>> velocity_max;
    std::vector<std::optional<Scalar>> effort_max;
};

/// Success type for the top-level URDF loader. chain is the strictly-serial
/// kinematic_chain extracted from the URDF; metadata is the accompanying
/// side-table; diagnostics are everything the description reader reported
/// along the way, tiered; claims are the reader's completeness assertions
/// about the model it produced. Aggregate-initializable, and the last two
/// members are appended rather than inserted so a positional initialization of
/// the first two keeps compiling.
///
/// Nothing in the loader branches on claims. A description that yields a
/// bit-exact chain can still fail to claim completeness::parsed -- irb120.urdf
/// does, over a <material> the reader does not recognize under <collision> --
/// so gating on a claim would refuse a correct robot over bookkeeping.
template <typename Scalar = double>
struct urdf_load_result
{
    kinematic_chain<Scalar, dynamic> chain;
    urdf_metadata<Scalar> metadata;
    std::vector<urdf_diagnostic> diagnostics{};
    meios::completeness claims{meios::completeness::none};
};

}

#endif
