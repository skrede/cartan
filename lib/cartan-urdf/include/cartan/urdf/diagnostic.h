#ifndef HPP_GUARD_CARTAN_URDF_DIAGNOSTIC_H
#define HPP_GUARD_CARTAN_URDF_DIAGNOSTIC_H

/// Severity-tiered diagnostic record for the URDF loader.
///
/// The description reader's own captured record carries a code, a location, a
/// message and a cause, but not the level it was reported at, so a consumer of
/// the reader's diagnostics vector cannot tell a dropped mesh from a debug
/// trace. This record exists to carry that level, which the loader observes on
/// the log sink it supplies and nowhere else.

#include "cartan/urdf/error.h"

#include <meios/diagnostic/level.h>
#include <meios/diagnostic/diagnostic_code.h>

#include <string>
#include <optional>

namespace cartan
{

/// Tier of a diagnostic reported while reading a robot description.
enum class urdf_severity
{
    error,   ///< The load failed, or would have but for a relaxed policy.
    warn,    ///< The model is usable but degraded; something did not resolve.
    info     ///< Detail a caller may log and has nothing to act on.
};

/// One diagnostic reported while reading a robot description. meios_code is
/// the reader's own code, carried unmapped so a caller can discriminate on the
/// exact condition rather than on the loader's coarser failure kinds; location
/// is unset for the reader's log arms that carry none.
struct urdf_diagnostic
{
    urdf_severity severity{urdf_severity::info};
    meios::diagnostic_code meios_code{meios::diagnostic_code::unspecified};
    std::optional<urdf_source_location> location{};
    std::string message{};
};

namespace detail
{

/// trace and debug are the reader's internal verbosity, which a kinematics
/// caller has nothing to do with; they collapse onto info rather than earning
/// a tier of their own.
inline urdf_severity severity_of(meios::level lvl)
{
    switch (lvl)
    {
    case meios::level::error:
        return urdf_severity::error;
    case meios::level::warn:
        return urdf_severity::warn;
    case meios::level::info:
    case meios::level::trace:
    case meios::level::debug:
        break;
    }
    return urdf_severity::info;
}

}

}

#endif
