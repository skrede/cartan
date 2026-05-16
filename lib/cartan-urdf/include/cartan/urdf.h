#ifndef HPP_GUARD_CARTAN_URDF_H
#define HPP_GUARD_CARTAN_URDF_H

/// Umbrella header for the cartan URDF loader.
///
/// Includes the diagnostic, schema, metadata, parser, and chain-extractor
/// entry points and exposes the user-facing names directly under namespace
/// cartan so callers can write cartan::urdf_error, cartan::load_options, and
/// cartan::load_urdf without reaching into the nested cartan::urdf namespace.

#include "cartan/urdf/build.h"
#include "cartan/urdf/error.h"
#include "cartan/urdf/parser.h"
#include "cartan/urdf/schema.h"
#include "cartan/urdf/metadata.h"

#include <utility>
#include <expected>
#include <filesystem>

namespace cartan
{

using urdf::urdf_failure;
using urdf::urdf_source_location;
using urdf::urdf_error;
using urdf::load_options;
using urdf::urdf_metadata;
using urdf::urdf_load_result;

/// Load a URDF document from disk and return the extracted kinematic chain
/// alongside its metadata. Parser failures (malformed XML, unsupported joint
/// types, unknown link references, mimic joints, non-physical inertials) and
/// extractor failures (branched tree, missing override link) flow through the
/// same std::expected channel using the same urdf_error type.
template <typename Scalar = double>
[[nodiscard]] inline std::expected<urdf_load_result<Scalar>, urdf_error>
load_urdf(const std::filesystem::path& path, const load_options& opts = {})
{
    auto parsed = urdf::parse_urdf_file<Scalar>(path);
    if (!parsed)
    {
        return std::unexpected(std::move(parsed).error());
    }
    return urdf::build_chain<Scalar>(*parsed, opts);
}

/// SDF loading is deferred; this entry point exists so the supported input
/// formats live behind a uniform pair of names. Returns
/// urdf_failure::sdf_not_supported unconditionally.
template <typename Scalar = double>
[[nodiscard]] inline std::expected<urdf_load_result<Scalar>, urdf_error>
load_sdf(const std::filesystem::path&)
{
    return std::unexpected(urdf_error{
        .kind = urdf_failure::sdf_not_supported,
        .detail = "SDF loading is deferred; URDF is the supported input format.",
        .location = std::nullopt});
}

}

#endif
