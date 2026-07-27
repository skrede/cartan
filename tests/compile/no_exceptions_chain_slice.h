#ifndef HPP_GUARD_CARTAN_TESTS_COMPILE_NO_EXCEPTIONS_CHAIN_SLICE_H
#define HPP_GUARD_CARTAN_TESTS_COMPILE_NO_EXCEPTIONS_CHAIN_SLICE_H

// The chain half of the exceptions-off compile gate: both validated chain
// factories and two checked kinematics entry points, each exercised on its
// success branch and on its failure branch. A gate that only took the success
// path would say nothing about the code generated for the failure branch, and
// the failure branch is where an ungated throw would sit.
//
// Every refusal below is consumed by branching on the result rather than
// through expected's accessor, which is the only channel a consumer without
// exceptions has.

#include "cartan/types.h"
#include "cartan/expected.h"

#include "cartan/lie/se3.h"
#include "cartan/lie/so3.h"

#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/static_chain.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <array>

namespace cartan::compile_gate
{

using single_joint_chain = static_chain<float, revolute_z>;
using single_joint_limits = std::array<joint_limits<float>, 1>;

inline se3<float> unit_home()
{
    return se3<float>(so3<float>::identity(), vector3<float>(1.0F, 0.0F, 0.0F));
}

inline std::array<screw_axis<float>, 1> axis_along(const vector3<float>& direction)
{
    return {screw_axis<float>::revolute(direction, vector3<float>(0.0F, 0.0F, 0.0F))};
}

inline expected<single_joint_limits, chain_failure> unit_limits()
{
    auto bounded = joint_limits<float>::make(-1.0F, 1.0F);
    if (!bounded.has_value())
    {
        return unexpected(bounded.error());
    }
    return single_joint_limits{*bounded};
}

inline float checked_limits_factory()
{
    float acc = 0.0F;

    auto bounded = joint_limits<float>::make(-1.0F, 1.0F);
    if (bounded.has_value())
    {
        acc += bounded->position_max();
    }

    auto reversed = joint_limits<float>::make(1.0F, -1.0F);
    if (!reversed.has_value())
    {
        acc += static_cast<float>(static_cast<int>(reversed.error()));
    }

    return acc;
}

inline float checked_chain_factory()
{
    float acc = 0.0F;

    auto limits = unit_limits();
    if (!limits.has_value())
    {
        return acc;
    }

    auto tagged = single_joint_chain::make(
        unit_home(), axis_along({0.0F, 0.0F, 1.0F}), *limits);
    if (tagged.has_value())
    {
        acc += tagged->home().translation().sum();
    }

    // A y-axis screw under a revolute_z tag is refused as a value.
    auto contradicted = single_joint_chain::make(
        unit_home(), axis_along({0.0F, 1.0F, 0.0F}), *limits);
    if (!contradicted.has_value())
    {
        acc += static_cast<float>(static_cast<int>(contradicted.error()));
    }

    return acc;
}

inline float checked_entry_points()
{
    float acc = 0.0F;

    auto limits = unit_limits();
    if (!limits.has_value())
    {
        return acc;
    }
    auto chain = single_joint_chain::make(
        unit_home(), axis_along({0.0F, 0.0F, 1.0F}), *limits);
    if (!chain.has_value())
    {
        return acc;
    }

    Eigen::Vector<float, 1> q;
    q << 0.25F;
    auto fk = forward_kinematics(*chain, q);
    if (fk.has_value())
    {
        acc += fk->end_effector.translation().sum();
        auto jacobian = space_jacobian(*chain, *fk);
        if (jacobian.has_value())
        {
            acc += jacobian->sum();
        }
    }

    const Eigen::VectorXf ill_sized = Eigen::VectorXf::Zero(2);
    auto refused = forward_kinematics(*chain, ill_sized);
    if (!refused.has_value())
    {
        acc += static_cast<float>(static_cast<int>(refused.error()));
    }

    return acc;
}

/// kinematic_chain's constructor and its bounds-checked axis() reach a
/// fail-stop rather than a throw when exceptions are unavailable, and that
/// branch is behind a preprocessor condition no other build compiles.
inline float chain_constructor_guard()
{
    auto limits = unit_limits();
    if (!limits.has_value())
    {
        return 0.0F;
    }
    const kinematic_chain<float, 1> chain(
        unit_home(), axis_along({0.0F, 0.0F, 1.0F}), *limits);
    return chain.axis(0).to_vector().sum();
}

}

#endif
