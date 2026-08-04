#ifndef HPP_GUARD_CARTAN_URDF_H
#define HPP_GUARD_CARTAN_URDF_H

/// Umbrella header for the cartan URDF loader.
///
/// Includes the diagnostic, schema, metadata, parser, and chain-extractor
/// entry points, all of which declare their user-facing names directly under
/// namespace cartan so callers can write cartan::urdf_error, cartan::load_options,
/// and cartan::load_urdf.

#include "cartan/urdf/load.h"
#include "cartan/urdf/build.h"
#include "cartan/urdf/error.h"
#include "cartan/urdf/parser.h"
#include "cartan/urdf/schema.h"
#include "cartan/urdf/metadata.h"

#include "cartan/urdf/detail/code_map.h"

#include "cartan/expected.h"

#include <meios/urdf/load.h>

#include <meios/diagnostic/log_sink.h>

#include <filesystem>

namespace cartan
{

/// Load a URDF or xacro document from disk and return the extracted kinematic
/// chain alongside its metadata. Reading failures (malformed XML, unresolved
/// substitutions, unsupported joint types, non-tree topology) and extractor
/// failures (branched tree, missing override link) flow through the same
/// cartan::expected channel using the same urdf_error type.
template <typename Scalar = double>
inline cartan::expected<urdf_load_result<Scalar>, urdf_error>
load_urdf(const std::filesystem::path& path, const load_options& opts = {})
{
    // Only the log-sink overload is called. A diagnostic below the error tier
    // reaches a consumer through the sink and through nothing else, so the
    // convenience overload that supplies no sink cannot report one at all.
    meios::log_sink log;
    auto loaded = meios::load(path, opts.description, log);
    if (!loaded)
    {
        return cartan::unexpected(detail::failure_from(loaded.error()));
    }
    return chain_from_model<Scalar>(loaded->robot, opts);
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
