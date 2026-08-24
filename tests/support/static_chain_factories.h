#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_STATIC_CHAIN_FACTORIES_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_STATIC_CHAIN_FACTORIES_H

// Compile-time-tagged twins of the nine benchmark geometries in
// tests/fixtures/chain_factories.h: same axes, home and bounds, with the joint
// kinds moved from runtime storage into the type so the tag-dispatched
// evaluation paths can be driven and compared against the generic ones.
//
// Each alias is named for its robot even where two robots share a tag list, so
// a helper's return type names the robot it builds.

#include "expected_helpers.h"

#include "../fixtures/chain_factories.h"

#include <cartan/serial/chain/joint_tags.h>
#include <cartan/serial/chain/static_chain.h>

namespace cartan::testing
{

template <typename Scalar>
using static_3r_planar_chain = static_chain<Scalar, revolute_z, revolute_z, revolute_z>;

template <typename Scalar>
static_3r_planar_chain<Scalar> make_3r_planar_static()
{
    auto kc = fixtures::make_3r_planar_chain<Scalar>();
    return unwrap(
        static_3r_planar_chain<Scalar>::make(kc.home(), kc.axes(), kc.limits()),
        "make_3r_planar_static");
}

template <typename Scalar>
using static_ur3e_chain = static_chain<Scalar,
    revolute_z, revolute_y, revolute_y,
    revolute_y, revolute_z, revolute_y>;

template <typename Scalar>
static_ur3e_chain<Scalar> make_ur3e_static()
{
    auto kc = fixtures::make_ur3e_chain<Scalar>();
    return unwrap(
        static_ur3e_chain<Scalar>::make(kc.home(), kc.axes(), kc.limits()),
        "make_ur3e_static");
}

template <typename Scalar>
using static_lbr_med14_chain = static_chain<Scalar,
    revolute_z, revolute_y, revolute_z, revolute_y,
    revolute_z, revolute_y, revolute_z>;

template <typename Scalar>
static_lbr_med14_chain<Scalar> make_lbr_med14_static()
{
    auto kc = fixtures::make_lbr_med14_chain<Scalar>();
    return unwrap(
        static_lbr_med14_chain<Scalar>::make(kc.home(), kc.axes(), kc.limits()),
        "make_lbr_med14_static");
}

template <typename Scalar>
using static_kr6_sixx_chain = static_chain<Scalar,
    revolute_z, revolute_y, revolute_y,
    revolute_x, revolute_y, revolute_x>;

template <typename Scalar>
static_kr6_sixx_chain<Scalar> make_kr6_sixx_static()
{
    auto kc = fixtures::make_kr6_sixx_chain<Scalar>();
    return unwrap(
        static_kr6_sixx_chain<Scalar>::make(kc.home(), kc.axes(), kc.limits()),
        "make_kr6_sixx_static");
}

template <typename Scalar>
using static_panda_chain = static_chain<Scalar,
    revolute_z, revolute_y, revolute_z, revolute_y,
    revolute_z, revolute_y, revolute_z>;

template <typename Scalar>
static_panda_chain<Scalar> make_panda_static()
{
    auto kc = fixtures::make_panda_chain<Scalar>();
    return unwrap(
        static_panda_chain<Scalar>::make(kc.home(), kc.axes(), kc.limits()),
        "make_panda_static");
}

template <typename Scalar>
using static_abb_irb120_chain = static_chain<Scalar,
    revolute_z, revolute_y, revolute_y,
    revolute_x, revolute_y, revolute_x>;

template <typename Scalar>
static_abb_irb120_chain<Scalar> make_abb_irb120_static()
{
    auto kc = fixtures::make_abb_irb120_chain<Scalar>();
    return unwrap(
        static_abb_irb120_chain<Scalar>::make(kc.home(), kc.axes(), kc.limits()),
        "make_abb_irb120_static");
}

template <typename Scalar>
using static_jaco2_chain = static_chain<Scalar,
    revolute_z, revolute_y, revolute_y,
    revolute_x, revolute_y, revolute_x>;

template <typename Scalar>
static_jaco2_chain<Scalar> make_jaco2_static()
{
    auto kc = fixtures::make_jaco2_chain<Scalar>();
    return unwrap(
        static_jaco2_chain<Scalar>::make(kc.home(), kc.axes(), kc.limits()),
        "make_jaco2_static");
}

template <typename Scalar>
using static_fetch_chain = static_chain<Scalar,
    revolute_z, revolute_y, revolute_x, revolute_y,
    revolute_x, revolute_y, revolute_x>;

template <typename Scalar>
static_fetch_chain<Scalar> make_fetch_static()
{
    auto kc = fixtures::make_fetch_chain<Scalar>();
    return unwrap(
        static_fetch_chain<Scalar>::make(kc.home(), kc.axes(), kc.limits()),
        "make_fetch_static");
}

template <typename Scalar>
using static_baxter_chain = static_chain<Scalar,
    revolute_z, revolute_y, revolute_x, revolute_y,
    revolute_x, revolute_y, revolute_x>;

template <typename Scalar>
static_baxter_chain<Scalar> make_baxter_static()
{
    auto kc = fixtures::make_baxter_chain<Scalar>();
    return unwrap(
        static_baxter_chain<Scalar>::make(kc.home(), kc.axes(), kc.limits()),
        "make_baxter_static");
}

}

#endif
