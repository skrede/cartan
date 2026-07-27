#include "harness.h"

#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <cartan/lie/se3.h>

#include <cartan/types.h>

#include <span>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace
{

using cartan::fuzzing::field_reader;

/// The chunk is read whether or not the bound is engaged, so the flag byte
/// changes which bounds the factory sees without shifting the fields behind it.
std::optional<double> optional_bound(field_reader& fields, bool engaged)
{
    const double value = fields.scalar();
    return engaged ? std::optional<double>(value) : std::nullopt;
}

/// A limits value the factory admitted has to survive the construction that
/// consumes it and answer a containment question about a decoded probe.
void drive_chain(const cartan::joint_limits<double>& limits, double probe)
{
    const auto axis = cartan::screw_axis<double>::revolute(
        cartan::vector3<double>::UnitZ(), cartan::vector3<double>::Zero());
    const cartan::kinematic_chain<double, cartan::dynamic> chain(
        cartan::se3<double>::identity(), {axis}, {limits});
    const auto span = chain.limits()[0].position_max() - chain.limits()[0].position_min();
    cartan::fuzzing::observe(span);
    cartan::fuzzing::observe(chain.limits()[0].contains_or(probe, false) ? 1.0 : 0.0);
    cartan::fuzzing::consume(chain);
}

}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    field_reader fields{std::span<const std::uint8_t>(data, size)};
    const double position_min = fields.scalar();
    const double position_max = fields.scalar();
    const std::uint8_t engaged = fields.byte();
    const auto velocity_max = optional_bound(fields, (engaged & 1) != 0);
    const auto effort_max = optional_bound(fields, (engaged & 2) != 0);
    const auto acceleration_max = optional_bound(fields, (engaged & 4) != 0);
    const double probe = fields.scalar();
    auto limits = cartan::joint_limits<double>::make(
        position_min, position_max, velocity_max, effort_max, acceleration_max);
    if (!limits.has_value())
    {
        return 0;
    }
    drive_chain(*limits, probe);
    return 0;
}
