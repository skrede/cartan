#include "harness.h"

#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <cartan/lie/se3.h>

#include <cartan/types.h>

#include <span>
#include <cmath>
#include <limits>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace
{

using cartan::fuzzing::field_reader;
using cartan::fuzzing::require;

/// A uniform chunk is an infinity with probability 2^-63, so a decoder that
/// only reinterprets chunks never reaches the encoding this library uses for an
/// unbounded joint, nor the coincident-infinity pair the factory refuses
/// because it bounds nothing. The selector reaches both. This is not a filter:
/// the decoded chunk passes through untouched on most selectors, and on none of
/// them is it inspected.
double with_specials(double value, std::uint8_t selector)
{
    switch (selector % 8)
    {
    case 0: return std::numeric_limits<double>::infinity();
    case 1: return -std::numeric_limits<double>::infinity();
    case 2: return std::numeric_limits<double>::quiet_NaN();
    default: return value;
    }
}

/// The chunk is read whether or not the bound is engaged, so the flag changes
/// which bounds the factory sees without shifting the fields behind it.
std::optional<double> optional_bound(field_reader& fields, std::uint8_t selector)
{
    const double value = with_specials(fields.scalar(), selector);
    return (selector & 128) != 0 ? std::optional<double>(value) : std::nullopt;
}

/// The position bounds a value carries are the ones handed in, neither is a
/// NaN, and together they describe a non-empty interval -- coincident only
/// where finite, since an infinity coincides with itself while bounding
/// nothing.
void check_bounds(
    const cartan::joint_limits<double>& limits, double position_min, double position_max)
{
    require(limits.position_min() == position_min, "position_min is carried unchanged");
    require(limits.position_max() == position_max, "position_max is carried unchanged");
    require(!std::isnan(limits.position_min()) && !std::isnan(limits.position_max()),
        "an admitted position bound is not a NaN");
    require(limits.position_min() < limits.position_max()
            || (limits.position_min() == limits.position_max()
                && std::isfinite(limits.position_min())),
        "admitted position bounds describe a non-empty interval");
}

/// A dynamic bound is engaged exactly where the caller engaged it, and an
/// engaged one is finite and non-negative.
void check_dynamic_bound(
    const std::optional<double>& carried,
    const std::optional<double>& handed,
    const char* contract)
{
    require(carried.has_value() == handed.has_value(), contract);
    if (carried.has_value())
    {
        require(*carried == *handed, contract);
        require(std::isfinite(*carried) && *carried >= 0.0, contract);
    }
}

/// contains() answers the containment question for a finite probe, declines it
/// for one that is not, and agrees with the bounds it was built from.
void check_containment(const cartan::joint_limits<double>& limits, double probe)
{
    const std::optional<bool> answer = limits.contains(probe);
    require(answer.has_value() == std::isfinite(probe),
        "contains answers exactly for a finite probe");
    if (answer.has_value())
    {
        require(*answer == (probe >= limits.position_min() && probe <= limits.position_max()),
            "contains agrees with the bounds it was built from");
    }
}

void drive_chain(const cartan::joint_limits<double>& limits)
{
    const auto axis = cartan::screw_axis<double>::revolute(
        cartan::vector3<double>::UnitZ(), cartan::vector3<double>::Zero());
    const cartan::kinematic_chain<double, cartan::dynamic> chain(
        cartan::se3<double>::identity(), {axis}, {limits});
    cartan::fuzzing::observe(
        chain.limits()[0].position_max() - chain.limits()[0].position_min());
    cartan::fuzzing::consume(chain);
}

}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    field_reader fields{std::span<const std::uint8_t>(data, size)};
    const std::uint8_t lower_shape = fields.byte();
    const std::uint8_t upper_shape = fields.byte();
    const double position_min = with_specials(fields.scalar(), lower_shape);
    const double position_max = with_specials(fields.scalar(), upper_shape);
    const auto velocity_max = optional_bound(fields, fields.byte());
    const auto effort_max = optional_bound(fields, fields.byte());
    const auto acceleration_max = optional_bound(fields, fields.byte());
    const double probe = with_specials(fields.scalar(), fields.byte());
    auto limits = cartan::joint_limits<double>::make(
        position_min, position_max, velocity_max, effort_max, acceleration_max);
    if (!limits.has_value())
    {
        return 0;
    }
    check_bounds(*limits, position_min, position_max);
    check_dynamic_bound(
        limits->velocity_max(), velocity_max, "velocity bound is carried unchanged");
    check_dynamic_bound(
        limits->effort_max(), effort_max, "effort bound is carried unchanged");
    check_dynamic_bound(
        limits->acceleration_max(), acceleration_max, "acceleration bound is carried unchanged");
    check_containment(*limits, probe);
    drive_chain(*limits);
    return 0;
}
