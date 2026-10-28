#ifndef HPP_GUARD_CARTAN_TESTS_BOUNDARY_FIXTURE_PINS_H
#define HPP_GUARD_CARTAN_TESTS_BOUNDARY_FIXTURE_PINS_H

// The two assertions that make a fixture's geometry observable, kept beside the
// pinned values rather than beside the factory list they are applied to.

#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>

namespace cartan::testing
{

/// The configuration every pinned pose is taken at. Non-zero because at the
/// zero configuration the end-effector is the home pose, which is one literal
/// out of the dozens a factory carries: no axis direction and no axis point
/// reaches the result there.
inline constexpr double k_pin_configuration = 0.3;

/// Wide enough for the float instantiation, whose largest measured departure
/// from the double result across all pinned factories is 2.0e-7, and four
/// orders of magnitude tighter than the smallest link length any factory
/// carries, so a transposed or truncated literal cannot slip through.
inline constexpr double k_pin_tolerance = 1e-5;

template <typename Chain>
void expect_chain(const Chain& chain, int joints, double x, double y, double z)
{
    using Scalar = typename Chain::scalar_type;
    REQUIRE(chain.num_joints() == joints);

    const Eigen::VectorX<Scalar> q =
        Eigen::VectorX<Scalar>::Constant(joints, Scalar(k_pin_configuration));
    auto fk = forward_kinematics(chain, q);
    REQUIRE(fk.has_value());

    const auto translation = fk->end_effector.translation();
    REQUIRE(fk->end_effector.matrix().allFinite());
    REQUIRE(std::abs(static_cast<double>(translation.x()) - x) < k_pin_tolerance);
    REQUIRE(std::abs(static_cast<double>(translation.y()) - y) < k_pin_tolerance);
    REQUIRE(std::abs(static_cast<double>(translation.z()) - z) < k_pin_tolerance);
}

/// Exact, not approximate: both halves of a twin pair are written from the same
/// decimal literals, so any difference at all is one of them having been edited
/// alone.
template <typename Left, typename Right>
void expect_same_geometry(const Left& left, const Right& right)
{
    using Scalar = typename Left::scalar_type;
    REQUIRE(left.num_joints() == right.num_joints());
    REQUIRE((left.home().matrix() - right.home().matrix()).isZero(Scalar(0)));

    for (int i = 0; i < left.num_joints(); ++i)
    {
        REQUIRE((left.axis(i).to_vector() - right.axis(i).to_vector()).isZero(Scalar(0)));
    }
}

}

#endif
