#ifndef HPP_GUARD_CARTAN_TESTS_FIXTURES_ANALYTICAL_CHAINS_H
#define HPP_GUARD_CARTAN_TESTS_FIXTURES_ANALYTICAL_CHAINS_H

/// @file analytical_chains.h
/// @brief 6R chain fixtures exercising analytical-solver output hygiene.
///
/// The zero-offset PUMA in analytical_solver_6r_test.cpp uses a +z outer-wrist
/// axis and generous +/-10 rad joint limits, which hides two output defects:
///   1. An anti-parallel outer wrist (joint-6 axis opposing joint-4) needs a
///      sign flip in the symmetric-Euler extraction; without it the wrist
///      branch is dropped by FK verification and reachable poses fail to solve.
///   2. Solutions are returned as raw base +/- offset angles; with realistic
///      [-pi, pi] limits the out-of-range and duplicate-modulo-2*pi outputs
///      become observable.
///
/// These fixtures build (a) a PUMA whose joint-6 axis is (0, 0, -1) and (b) a
/// PUMA with realistic [-pi, pi] joint limits.

#include <cartan/types.h>
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

/// PUMA-type 6R chain (Z, Y, Y | Z, Y, Z) with an ANTI-PARALLEL outer wrist:
/// joint-6 rotates about (0, 0, -1) while joint-4 rotates about (0, 0, +1).
/// Axes 4, 5, 6 still intersect at the common wrist center, so the geometry is
/// Pieper-valid; only the outer-wrist direction is reversed.
template <typename Scalar>
auto make_anti_parallel_wrist_puma()
    -> cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_y,
                            cartan::revolute_y, cartan::revolute_z,
                            cartan::revolute_y, cartan::revolute_z>
{
    using vec3 = cartan::vector3<Scalar>;

    const Scalar d1(0.5), a2(0.4), a3(0.3), d6(0.1);
    vec3 wrist_point(a2 + a3, Scalar(0), d1);
    vec3 ee_point(a2 + a3 + d6, Scalar(0), d1);

    auto s0 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), vec3(Scalar(0), Scalar(0), d1));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), vec3(a2, Scalar(0), d1));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), wrist_point);
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), wrist_point);
    // Anti-parallel outer wrist: joint-6 axis points along -z.
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(-1)), wrist_point);

    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), ee_point);
    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
    std::array<cartan::joint_limits<Scalar>, 6> limits = {
        lim, lim, lim, lim, lim, lim};

    return cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_y,
                                cartan::revolute_y, cartan::revolute_z,
                                cartan::revolute_y, cartan::revolute_z>(
        home, {s0, s1, s2, s3, s4, s5}, limits);
}

/// PUMA-type 6R chain (Z, Y, Y | Z, Y, Z) with an OFFSET SHOULDER: axis 2 is
/// shifted by `a1` along x, so shoulder axes 1 and 2 no longer intersect (their
/// closest-approach distance is `a1`). The spherical wrist is intact. This is
/// the opw-class geometry the Pieper decomposition cannot solve; a valid
/// factory must reject it at construction rather than mis-report a per-pose
/// `unreachable`.
template <typename Scalar>
auto make_offset_shoulder_puma()
    -> cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_y,
                            cartan::revolute_y, cartan::revolute_z,
                            cartan::revolute_y, cartan::revolute_z>
{
    using vec3 = cartan::vector3<Scalar>;

    const Scalar d1(0.5), a1(0.15), a2(0.4), a3(0.3), d6(0.1);
    vec3 wrist_point(a1 + a2 + a3, Scalar(0), d1);
    vec3 ee_point(a1 + a2 + a3 + d6, Scalar(0), d1);

    auto s0 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), vec3(Scalar(0), Scalar(0), Scalar(0)));
    // Shoulder offset: axis 2 passes through (a1, 0, d1), off the axis-1 line.
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), vec3(a1, Scalar(0), d1));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), vec3(a1 + a2, Scalar(0), d1));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), wrist_point);
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), wrist_point);
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), wrist_point);

    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), ee_point);
    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
    std::array<cartan::joint_limits<Scalar>, 6> limits = {
        lim, lim, lim, lim, lim, lim};

    return cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_y,
                                cartan::revolute_y, cartan::revolute_z,
                                cartan::revolute_y, cartan::revolute_z>(
        home, {s0, s1, s2, s3, s4, s5}, limits);
}

/// PUMA-type 6R chain (Z, Y, Y | Z, Y, Z) with a NEAR-SPHERICAL wrist: axis 4
/// (direction y) is shifted by `wrist_offset` along x (perpendicular to its own
/// direction), so wrist axes 4, 5, 6 miss the common center by that distance.
/// With a `wrist_offset` between the old loose 1e-3 gate and the solve
/// tolerance, the chain passes the legacy sphericity check yet is not solvable
/// to the acceptance tolerance -- a valid factory must reject it at
/// construction.
template <typename Scalar>
auto make_near_spherical_wrist_puma(Scalar wrist_offset)
    -> cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_y,
                            cartan::revolute_y, cartan::revolute_z,
                            cartan::revolute_y, cartan::revolute_z>
{
    using vec3 = cartan::vector3<Scalar>;

    const Scalar d1(0.5), a2(0.4), a3(0.3), d6(0.1);
    vec3 wrist_point(a2 + a3, Scalar(0), d1);
    vec3 ee_point(a2 + a3 + d6, Scalar(0), d1);

    auto s0 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), vec3(Scalar(0), Scalar(0), d1));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), vec3(a2, Scalar(0), d1));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), wrist_point);
    // Perpendicular offset: axis 4 (dir y) shifted along x, breaking sphericity.
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(wrist_point.x() + wrist_offset, Scalar(0), d1));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), wrist_point);

    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), ee_point);
    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
    std::array<cartan::joint_limits<Scalar>, 6> limits = {
        lim, lim, lim, lim, lim, lim};

    return cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_y,
                                cartan::revolute_y, cartan::revolute_z,
                                cartan::revolute_y, cartan::revolute_z>(
        home, {s0, s1, s2, s3, s4, s5}, limits);
}

/// PUMA-type 6R chain (Z, Y, Y | Z, Y, Z) with realistic [-pi, pi] joint
/// limits. Geometry matches the canonical zero-offset PUMA; only the limits
/// differ so that un-wrapped / duplicate solutions become observable.
template <typename Scalar>
auto make_puma_realistic_limits()
    -> cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_y,
                            cartan::revolute_y, cartan::revolute_z,
                            cartan::revolute_y, cartan::revolute_z>
{
    using vec3 = cartan::vector3<Scalar>;

    const Scalar d1(0.5), a2(0.4), a3(0.3), d6(0.1);
    vec3 wrist_point(a2 + a3, Scalar(0), d1);
    vec3 ee_point(a2 + a3 + d6, Scalar(0), d1);

    auto s0 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), vec3(Scalar(0), Scalar(0), d1));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), vec3(a2, Scalar(0), d1));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), wrist_point);
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)), wrist_point);
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)), wrist_point);

    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), ee_point);
    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
    std::array<cartan::joint_limits<Scalar>, 6> limits = {
        lim, lim, lim, lim, lim, lim};

    return cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_y,
                                cartan::revolute_y, cartan::revolute_z,
                                cartan::revolute_y, cartan::revolute_z>(
        home, {s0, s1, s2, s3, s4, s5}, limits);
}

}

#endif
