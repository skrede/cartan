#ifndef HPP_GUARD_CARTAN_URDF_DETAIL_DIAGNOSTIC_SINK_H
#define HPP_GUARD_CARTAN_URDF_DETAIL_DIAGNOSTIC_SINK_H

/// Collector that keeps the severity of every diagnostic the description
/// reader reports.
///
/// All four log arms are overridden. The reader's base sink forwards its
/// four-argument arm to the three-argument one, dropping the code, and its
/// five-argument arm to the four-argument one, dropping the cause; overriding
/// fewer than four arms therefore loses the very field this collector exists
/// to keep.

#include "cartan/urdf/detail/code_map.h"

#include "cartan/urdf/error.h"
#include "cartan/urdf/diagnostic.h"

#include <meios/diagnostic/log_sink.h>
#include <meios/diagnostic/operation_failure.h>

#include <string>
#include <vector>
#include <utility>
#include <optional>

namespace cartan::detail
{

class diagnostic_sink : public meios::log_sink
{
public:
    diagnostic_sink()
        : m_records()
    {
    }

    void log(meios::level lvl, const std::string& message) override
    {
        capture(lvl, meios::diagnostic_code::unspecified, std::nullopt, message);
    }

    void log(meios::level lvl, const meios::source_location& location,
             const std::string& message) override
    {
        capture(lvl, meios::diagnostic_code::unspecified, location_of(location, ""), message);
    }

    void log(meios::level lvl, meios::diagnostic_code code,
             const meios::source_location& location, const std::string& message) override
    {
        capture(lvl, code, location_of(location, ""), message);
    }

    void log(meios::level lvl, meios::diagnostic_code code,
             const meios::source_location& location, const meios::operation_failure& cause,
             const std::string& message) override
    {
        capture(lvl, code, location_of(location, ""),
                message + " [" + std::string(meios::to_string(cause.operation)) + ": "
                    + cause.native.message() + "]");
    }

    const std::vector<urdf_diagnostic>& records() const
    {
        return m_records;
    }

    std::vector<urdf_diagnostic> take_records()
    {
        return std::move(m_records);
    }

private:
    std::vector<urdf_diagnostic> m_records;

    void capture(meios::level lvl, meios::diagnostic_code code,
                 std::optional<urdf_source_location> at, const std::string& message)
    {
        m_records.push_back(urdf_diagnostic{
            .severity = severity_of(lvl),
            .meios_code = code,
            .location = std::move(at),
            .message = message});
    }
};

}

#endif
