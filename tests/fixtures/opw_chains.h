#ifndef HPP_GUARD_CARTAN_TESTS_FIXTURES_OPW_CHAINS_H
#define HPP_GUARD_CARTAN_TESTS_FIXTURES_OPW_CHAINS_H

/// @file opw_chains.h
/// @brief Ortho-parallel spherical-wrist (OPW) chain fixtures.
///
/// Provides a REAL offset-shoulder, offset-elbow KR6 R900 SIXX screw model plus
/// the matching hand-derived `opw_parameters`. This is a distinct artifact from
/// the offset-free `kr6_sixx_geometry` in chain_factories.h: here the shoulder
/// axes 1 and 2 miss by `a1 = 0.025` (they intersect in the offset-free model),
/// and the forearm carries the nonzero elbow offset `a2 = -0.035`.
///
/// The two functions describe the same physical arm through two independent
/// code paths: `make_kr6_r900_opw_chain` builds a product-of-exponentials screw
/// model, while `kr6_r900_opw_parameters` feeds the closed-form OPW forward map.
/// The reconstruction test pins the parameter numerics by asserting the two
/// agree everywhere; the numbers below are trusted only because that gate holds,
/// not because they were read from a datasheet.

#include <cartan/types.h>
#include <cartan/analytical.h>
#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/joint_tags.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/chain/joint_limits.h>

#include <array>
#include <numbers>

namespace cartan::fixtures
{

/// Hand-derived ortho-parallel spherical-wrist parameters for the KUKA KR6 R900
/// SIXX. The a/b/c lengths follow the Brandstotter convention; `offsets` and
/// `sign_corrections` follow the standard KUKA zeroing (arm horizontal-forward
/// at the user home, shoulder internal angle +pi/2).
///
/// These numerics are pinned by the FK-reconstruction gate against
/// make_kr6_r900_opw_chain, NOT trusted from a datasheet: a wrong length, offset
/// or sign surfaces as a 1e-9 mismatch there rather than silently modeling the
/// wrong robot.
template <typename Scalar>
cartan::opw_parameters<Scalar> kr6_r900_opw_parameters()
{
    const Scalar half_pi = std::numbers::pi_v<Scalar> / Scalar(2);

    cartan::opw_parameters<Scalar> params{};
    params.a1 = Scalar(0.025);
    params.a2 = Scalar(-0.035);
    params.b  = Scalar(0);
    params.c1 = Scalar(0.400);
    params.c2 = Scalar(0.455);
    params.c3 = Scalar(0.420);
    params.c4 = Scalar(0.080);
    params.offsets = {Scalar(0), -half_pi, Scalar(0), Scalar(0), Scalar(0), Scalar(0)};
    params.sign_corrections = {-1, 1, 1, -1, 1, -1};
    return params;
}

/// Real offset-shoulder + offset-elbow KR6 R900 SIXX screw model (Z, Y, Y | X,
/// Y, X at the user home). The screw axes are the physical joint lines at the
/// user-home configuration q = 0, which the KUKA convention places with the arm
/// extended horizontally forward (shoulder internal angle +pi/2). At that pose
/// every axis is aligned with a base principal direction, so the joint tags are
/// exact.
///
/// Geometry encoded (independently of the closed-form parameter map):
///   - axis 1 about -z through the origin (sign_corrections[0] = -1);
///   - axis 2 (shoulder) about +y through (a1, 0, c1): the shoulder line is
///     offset by a1 = 0.025 along x, so axes 1 and 2 miss -- unlike the
///     offset-free kr6_sixx_geometry;
///   - axis 3 (elbow) about +y through (a1 + c2, 0, c1): the upper arm (length
///     c2) has swung forward at the home pose;
///   - the wrist center sits at (a1 + c2 + c3, 0, c1 - a2): the forearm carries
///     the nonzero elbow offset a2 = -0.035, which raises the wrist center by
///     -a2 relative to the elbow height. Axes 4/5/6 about -x / +y / -x all pass
///     through that center (spherical wrist).
/// Lengths: c1 = 0.400 base height, c2 = 0.455 upper arm, c3 = 0.420 forearm,
/// c4 = 0.080 wrist-to-flange. Joint limits are the realistic [-pi, pi].
template <typename Scalar>
auto make_kr6_r900_opw_chain()
    -> cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_y,
                            cartan::revolute_y, cartan::revolute_x,
                            cartan::revolute_y, cartan::revolute_x>
{
    using vec3 = cartan::vector3<Scalar>;

    const Scalar a1(0.025), a2(-0.035), c1(0.400), c2(0.455), c3(0.420),
        c4(0.080);
    const Scalar half_pi = std::numbers::pi_v<Scalar> / Scalar(2);

    // Physical joint lines at the user home (arm horizontal-forward).
    const vec3 shoulder(a1, Scalar(0), c1);
    const vec3 elbow(a1 + c2, Scalar(0), c1);
    const vec3 wrist_center(a1 + c2 + c3, Scalar(0), c1 - a2);

    // Base rotation, reversed relative to the internal convention.
    auto s0 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(-1)), vec3(Scalar(0), Scalar(0), Scalar(0)));
    // Offset shoulder: axis 2 misses axis 1 by a1 along x.
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), shoulder);
    // Elbow at the forward-swung end of the upper arm.
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), elbow);
    // Spherical wrist through the offset-elbow wrist center.
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(-1), Scalar(0), Scalar(0)), wrist_center);
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), wrist_center);
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(-1), Scalar(0), Scalar(0)), wrist_center);

    // Home pose: flange along the wrist approach axis (+x here), orientation the
    // 90-degree upper-arm rotation about y.
    auto home_rotation =
        cartan::so3<Scalar>::exp(vec3(Scalar(0), half_pi, Scalar(0)));
    vec3 flange(a1 + c2 + c3 + c4, Scalar(0), c1 - a2);
    auto home = cartan::se3<Scalar>(home_rotation, flange);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
    std::array<cartan::joint_limits<Scalar>, 6> limits = {
        lim, lim, lim, lim, lim, lim};

    return cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_y,
                                cartan::revolute_y, cartan::revolute_x,
                                cartan::revolute_y, cartan::revolute_x>(
        home, {s0, s1, s2, s3, s4, s5}, limits);
}

}

#endif
