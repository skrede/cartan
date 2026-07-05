#ifndef HPP_GUARD_CARTAN_TESTS_FIXTURES_PRISMATIC_CHAINS_H
#define HPP_GUARD_CARTAN_TESTS_FIXTURES_PRISMATIC_CHAINS_H

/// @file prismatic_chains.h
/// @brief Signed-direction prismatic chain fixtures exercising the axis-sign
///        path in the FK/Jacobian fast-path specializations.
///
/// The revolute-only robot factories in chain_factories.h never exercise the
/// prismatic direction sign: a prismatic joint pointing along +z passes even
/// when the specializations drop the sign carried in screw_axis::v(). These
/// fixtures build an R-P-P-R chain whose two prismatic joints point along the
/// NEGATIVE z and x directions so a dropped sign produces an observable error
/// against the generic se3::exp Product-of-Exponentials oracle.

#include <cartan/types.h>
#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/joint_tags.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <numbers>

namespace cartan::fixtures
{

// ===========================================================================
// Signed-direction prismatic chains
// ===========================================================================

/// R-P-P-R chain (4-DOF) with signed prismatic directions.
///
/// Joint 0: revolute about +z at the origin.
/// Joint 1: prismatic along -z (screw v = (0, 0, -1)).
/// Joint 2: prismatic along -x (screw v = (-1, 0, 0)).
/// Joint 3: revolute about +z at (1, 0, 0).
///
/// The negative prismatic directions are the point of the fixture: a fast
/// path that writes the axis basis vector instead of screw_axis::v() yields
/// FK translation of the wrong sign and Jacobian columns of the wrong sign.
template <typename Scalar>
auto make_rppr_signed_chain() -> cartan::kinematic_chain<Scalar, 4>
{
    using vec3 = cartan::vector3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::prismatic(
        vec3(Scalar(0), Scalar(0), Scalar(-1)));
    auto s3 = cartan::screw_axis<Scalar>::prismatic(
        vec3(Scalar(-1), Scalar(0), Scalar(0)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(1), Scalar(0), Scalar(0)));

    vec3 home_trans(Scalar(1), Scalar(0), Scalar(0));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> rot{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
    cartan::joint_limits<Scalar> pris{Scalar(-1), Scalar(1)};

    return cartan::kinematic_chain<Scalar, 4>(
        home, {s1, s2, s3, s4}, {rot, pris, pris, rot});
}

/// Compile-time-tagged variant of the signed R-P-P-R chain. Shares the runtime
/// screw/home/limit data with the kinematic_chain factory but drives the
/// static_chain (compile-time tag) FK/Jacobian dispatchers.
template <typename Scalar>
auto make_rppr_signed_static()
    -> cartan::static_chain<Scalar, cartan::revolute_z, cartan::prismatic_z,
                            cartan::prismatic_x, cartan::revolute_z>
{
    auto kc = make_rppr_signed_chain<Scalar>();
    return cartan::static_chain<Scalar, cartan::revolute_z, cartan::prismatic_z,
                                cartan::prismatic_x, cartan::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

/// Dynamic-sized variant of the signed R-P-P-R chain, driving the runtime
/// joint-kind dispatchers.
template <typename Scalar>
auto make_rppr_signed_dynamic()
    -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    return make_rppr_signed_chain<Scalar>().to_dynamic();
}

}

#endif
