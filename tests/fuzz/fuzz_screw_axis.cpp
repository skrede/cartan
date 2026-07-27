#include "harness.h"

#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <cartan/lie/se3.h>

#include <cartan/types.h>
#include <cartan/detail/epsilon.h>

#include <span>
#include <cmath>
#include <limits>
#include <cstddef>
#include <cstdint>

namespace
{

using cartan::fuzzing::field_reader;
using cartan::fuzzing::require;

constexpr double k_window = cartan::detail::sqrt_epsilon_v<double>;

/// Multiples of the acceptance window's own width, so a shaped axis lands just
/// inside and just outside it rather than only at its center. A window sampled
/// only at its center never separates a factory that checks it from one that
/// does not.
double off_unit(std::uint8_t selector)
{
    return 1.0 + (static_cast<double>(selector % 5) - 2.0) * k_window;
}

/// A six-vector of raw chunks misses the factory's unit-norm window with
/// probability one, so a target that only ever hands one over exercises the
/// rejection test and nothing behind it. The shapes reach the window, both its
/// edges, the near-zero angular part whose looser prismatic reading the factory
/// documents, and an outright infinity. Nothing is rejected or repaired on any
/// path: the raw shape delivers whatever the bytes decoded to, and a shaped
/// vector that comes out non-unit is one the factory still has to refuse.
void shape_angular(std::uint8_t shape, std::uint8_t knob, cartan::vector3<double>& angular)
{
    switch (shape % 6)
    {
    case 1: angular = angular.stableNormalized(); return;
    case 2: angular = angular.stableNormalized() * off_unit(knob); return;
    case 3: angular = angular.stableNormalized() * (k_window * 0.5); return;
    case 4: angular.setZero(); return;
    case 5: angular(knob % 3) = std::numeric_limits<double>::infinity(); return;
    default: return;
    }
}

void shape_linear(std::uint8_t shape, std::uint8_t knob, cartan::vector3<double>& linear)
{
    const std::uint8_t kind = shape % 6;
    if (kind == 3 || kind == 4)
    {
        linear = linear.stableNormalized() * off_unit(knob);
    }
}

cartan::vector6<double> decode_axis(field_reader& fields)
{
    const std::uint8_t shape = fields.byte();
    const std::uint8_t knob = fields.byte();
    cartan::vector3<double> angular(
        fields.scalar(), fields.scalar(), fields.scalar());
    cartan::vector3<double> linear(
        fields.scalar(), fields.scalar(), fields.scalar());
    shape_angular(shape, knob, angular);
    shape_linear(shape, knob, linear);
    cartan::vector6<double> raw;
    raw << angular, linear;
    return raw;
}

/// An admitted axis is finite, carries the six components it was handed, and
/// holds a unit component in whichever of the two readings the factory took:
/// unit angular part above the window, unit linear part at or below it.
void check_contract(
    const cartan::screw_axis<double>& axis, const cartan::vector6<double>& handed)
{
    const cartan::vector6<double> carried = axis.to_vector();
    require(carried.allFinite(), "an admitted axis is finite");
    require((carried.array() == handed.array()).all(), "the six-vector is carried unchanged");
    const double angular = axis.omega().norm();
    const double linear = axis.v().norm();
    require(angular > k_window ? std::abs(angular - 1.0) <= k_window
                               : std::abs(linear - 1.0) <= k_window,
        "an admitted axis carries the unit component its reading promises");
}

/// An axis the factory admitted is finite and unit, so a single-joint chain
/// carrying it must construct and evaluate.
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
    const cartan::vector6<double> handed = decode_axis(fields);
    const double position = fields.scalar();
    auto axis = cartan::screw_axis<double>::from_vector(handed);
    if (!axis.has_value())
    {
        return 0;
    }
    check_contract(*axis, handed);
    drive_chain(*axis, position);
    return 0;
}
