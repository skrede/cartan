#include "harness.h"

#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <cartan/lie/se3.h>

#include <cartan/types.h>

#include <span>
#include <cstddef>
#include <cstdint>

namespace
{

using cartan::fuzzing::field_reader;

/// A six-vector of raw chunks misses the factory's unit-norm window with
/// probability one -- measured: zero acceptances in twenty thousand uniform
/// inputs -- so a target that only ever hands one over fuzzes the rejection
/// test and nothing behind it. Two of the three shapes land inside the window
/// instead. Nothing is rejected or repaired on any path: the raw shape still
/// delivers whatever the bytes decoded to, and normalizing a zero or non-finite
/// triple yields a value the factory still has to refuse.
cartan::vector6<double> decode_axis(field_reader& fields)
{
    const std::uint8_t shape = fields.byte();
    cartan::vector3<double> angular(
        fields.scalar(), fields.scalar(), fields.scalar());
    cartan::vector3<double> linear(
        fields.scalar(), fields.scalar(), fields.scalar());
    if (shape % 3 == 1)
    {
        angular = angular.normalized();
    }
    if (shape % 3 == 2)
    {
        angular.setZero();
        linear = linear.normalized();
    }
    cartan::vector6<double> raw;
    raw << angular, linear;
    return raw;
}

/// An axis the factory admitted is finite and unit, so a single-joint chain
/// carrying it must construct and evaluate. Anything else is the factory
/// having admitted a value it should have refused.
void drive_chain(const cartan::screw_axis<double>& axis, double position)
{
    auto limits = cartan::joint_limits<double>::make(-1.0, 1.0);
    if (!limits.has_value())
    {
        return;
    }
    const cartan::kinematic_chain<double, cartan::dynamic> chain(
        cartan::se3<double>::identity(), {axis}, {*limits});
    cartan::fuzzing::observe(chain.axis(0).to_vector().squaredNorm());
    cartan::fuzzing::observe(chain.limits()[0].contains_or(position, false) ? 1.0 : 0.0);
    cartan::fuzzing::consume(chain);
}

}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    field_reader fields{std::span<const std::uint8_t>(data, size)};
    const cartan::vector6<double> raw = decode_axis(fields);
    const double position = fields.scalar();
    auto axis = cartan::screw_axis<double>::from_vector(raw);
    if (!axis.has_value())
    {
        return 0;
    }
    drive_chain(*axis, position);
    return 0;
}
