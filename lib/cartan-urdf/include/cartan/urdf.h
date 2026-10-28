#ifndef HPP_GUARD_CARTAN_URDF_H
#define HPP_GUARD_CARTAN_URDF_H

/// Umbrella header for the cartan URDF loader.
///
/// Includes the diagnostic, schema, metadata, and chain-extractor entry points,
/// all of which declare their user-facing names directly under namespace cartan
/// so callers can write cartan::urdf_error, cartan::load_options, and
/// cartan::load_urdf.

#include "cartan/urdf/load.h"
#include "cartan/urdf/build.h"
#include "cartan/urdf/error.h"
#include "cartan/urdf/schema.h"
#include "cartan/urdf/metadata.h"
#include "cartan/urdf/diagnostic.h"

#include "cartan/urdf/detail/code_map.h"
#include "cartan/urdf/detail/diagnostic_sink.h"

#include "cartan/expected.h"

#include <meios/urdf/load.h>

#include <filesystem>

namespace cartan
{

/// Load a URDF or xacro document from disk and return the extracted kinematic
/// chain alongside its metadata. Reading failures (malformed XML, unresolved
/// substitutions, unsupported joint types, non-tree topology) and extractor
/// failures (branched tree, missing override link) flow through the same
/// cartan::expected channel using the same urdf_error type. A load that
/// succeeds still reports: the reader's diagnostics, tiered by severity, and
/// its completeness claims ride on the result, and neither gates the load.
template <typename Scalar = double>
inline cartan::expected<urdf_load_result<Scalar>, urdf_error>
load_urdf(const std::filesystem::path& path, const load_options& opts = {})
{
    // Only the log-sink overload is called. A diagnostic below the error tier
    // reaches a consumer through the sink and through nothing else, so the
    // convenience overload that supplies no sink cannot report one at all.
    detail::diagnostic_sink log;
    auto loaded = meios::load(path, opts.description, log);
    if (!loaded)
    {
        return cartan::unexpected(detail::failure_from(loaded.error()));
    }
    auto result = chain_from_model<Scalar>(loaded->robot, opts);
    if (result)
    {
        result->diagnostics = log.take_records();
        result->claims = loaded->claims;
    }
    return result;
}

/// Load a URDF or xacro document from disk and return the rigid transform from
/// its base link to its tool link.
///
/// This is the entry point for a description that poses no inverse-kinematics
/// problem, which load_urdf refuses with urdf_failure::no_movable_joint: an
/// assembly of links bolted together, a sensor bracket, a tool adapter. It is
/// defined for the shape the chain extractor walks -- one root, one leaf after
/// the fixed-joint merge -- and a description that branches is refused rather
/// than answered. It reads a description with mobile joints too, where the
/// transform is the chain's home pose.
///
/// It does not answer the pose of a named intermediate frame: no intermediate
/// frame survives loading, for any description, so there is nothing to name.
template <typename Scalar = double>
inline cartan::expected<se3<Scalar>, urdf_error>
load_urdf_transform(const std::filesystem::path& path, const load_options& opts = {})
{
    detail::diagnostic_sink log;
    auto loaded = meios::load(path, opts.description, log);
    if (!loaded)
    {
        return cartan::unexpected(detail::failure_from(loaded.error()));
    }
    return transform_from_model<Scalar>(loaded->robot, opts);
}

/// SDF loading is deferred; this entry point exists so the supported input
/// formats live behind a uniform pair of names. Returns
/// urdf_failure::sdf_not_supported unconditionally.
template <typename Scalar = double>
inline cartan::expected<urdf_load_result<Scalar>, urdf_error>
load_sdf(const std::filesystem::path&)
{
    return cartan::unexpected(urdf_error{
        .kind = urdf_failure::sdf_not_supported,
        .detail = "SDF loading is deferred; URDF is the supported input format.",
        .location = std::nullopt});
}

}

#endif
