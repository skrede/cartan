#include "harness.h"

#include "../fixtures/chain_factories.h"

#include <cartan/analytical/solver_2r.h>
#include <cartan/analytical/solver_3r.h>
#include <cartan/analytical/solver_6r.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>

#include <cartan/types.h>

#include <span>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <type_traits>

namespace
{

using cartan::fuzzing::field_reader;

/// Half-width of the in-workspace position band, in the fixture chains' linear unit.
/// Measured over twenty thousand uniform inputs: a band of 0.5 puts 19% of them
/// inside a solvable workspace, against 2% for a band of 2.0 and 3% when the
/// position is a raw chunk triple and nothing else.
constexpr double k_reach = 0.5;

/// The orientation is decoded from bytes onto a half-turn either side of the
/// identity. Driving the unchecked exponential map with a raw chunk would only
/// find the overflow that map already has, in a target whose subject is the
/// closed-form solvers behind it.
double scaled(std::uint8_t selector, double span)
{
    return (static_cast<double>(selector) / 128.0 - 1.0) * span;
}

/// The position is either a raw chunk triple -- which is where a non-finite or
/// astronomically distant target crosses into the solvers -- or a triple inside
/// the fixture chains' working radius. Raw chunks alone put the target outside
/// the workspace almost always, so almost nothing behind the reachability test
/// would ever run.
cartan::se3<double> decode_target(field_reader& fields)
{
    const std::uint8_t shape = fields.byte();
    const cartan::vector3<double> rotation(
        scaled(fields.byte(), std::numbers::pi_v<double>),
        scaled(fields.byte(), std::numbers::pi_v<double>),
        scaled(fields.byte(), std::numbers::pi_v<double>));
    const cartan::vector3<double> raw(
        fields.scalar(), fields.scalar(), fields.scalar());
    const cartan::vector3<double> inside(
        scaled(fields.byte(), k_reach),
        scaled(fields.byte(), k_reach),
        scaled(fields.byte(), k_reach));
    const cartan::vector3<double> position = (shape & 1) != 0 ? inside : raw;
    return cartan::se3<double>(cartan::so3<double>::exp(rotation), position);
}

template <typename Solver>
void drive_solver(const Solver& solver, const cartan::se3<double>& target)
{
    auto result = solver.solve(target);
    if (!result.has_value())
    {
        return;
    }
    double total = 0.0;
    for (const auto& solution : result.value())
    {
        total += solution.norm();
    }
    cartan::fuzzing::observe(total);
}

void solve_planar_2r(const cartan::se3<double>& target)
{
    static const auto chain = cartan::fixtures::make_planar_2r_static<double>();
    drive_solver(cartan::planar_2r_solver(chain), target);
}

void solve_spatial_3r(const cartan::se3<double>& target)
{
    static const auto chain = cartan::fixtures::make_spatial_3r_static<double>();
    drive_solver(cartan::spatial_3r_solver(chain), target);
}

void solve_pieper_6r(const cartan::se3<double>& target)
{
    static const auto chain = cartan::fixtures::make_abb_irb120_static<double>();
    using chain_type = std::remove_const_t<decltype(chain)>;
    auto solver = cartan::pieper_6r_solver<chain_type>::make(chain);
    if (solver.has_value())
    {
        drive_solver(*solver, target);
    }
}

}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    field_reader fields{std::span<const std::uint8_t>(data, size)};
    const cartan::se3<double> target = decode_target(fields);
    switch (fields.byte() % 3)
    {
    case 0: solve_planar_2r(target); break;
    case 1: solve_spatial_3r(target); break;
    default: solve_pieper_6r(target); break;
    }
    return 0;
}
