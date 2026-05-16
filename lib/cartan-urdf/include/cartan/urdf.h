#ifndef HPP_GUARD_CARTAN_URDF_H
#define HPP_GUARD_CARTAN_URDF_H

/// Umbrella header for the cartan URDF loader.
///
/// Includes the diagnostic, schema, metadata, and parser entry points and
/// exposes the user-facing names directly under namespace cartan so callers
/// can write cartan::urdf_error and cartan::load_options without reaching
/// into the nested cartan::urdf namespace.

#include "cartan/urdf/error.h"
#include "cartan/urdf/parser.h"
#include "cartan/urdf/schema.h"
#include "cartan/urdf/metadata.h"

namespace cartan
{

using urdf::urdf_failure;
using urdf::urdf_source_location;
using urdf::urdf_error;
using urdf::load_options;
using urdf::urdf_metadata;
using urdf::urdf_load_result;

}

#endif
