#ifndef HPP_GUARD_CARTAN_TESTS_FIXTURES_REDUNDANT_CHAINS_H
#define HPP_GUARD_CARTAN_TESTS_FIXTURES_REDUNDANT_CHAINS_H

/// @file redundant_chains.h
/// @brief Fixtures for kinematically redundant and high-DOF serial chains.
///
/// These chains exercise IK solver-layer code paths that only trigger when a
/// chain has more degrees of freedom than the six of task space:
///   - a 7-DOF chain has a 6x7 body Jacobian, i.e. a one-dimensional null
///     space, which stresses the null-space limit-enforcement projection;
///   - a 12-DOF chain has more joints than the Halton base table historically
///     provided, which stresses the seed generator's base lookup.

#include "chain_factories.h"

#include <cartan/types.h>
#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <vector>
#include <utility>
#include <numbers>

namespace cartan::fixtures
{

/// Redundant 7-DOF arm (runtime DOF).
///
/// A 7-DOF chain produces a 6x7 body Jacobian: one degree of redundancy and
/// thus a one-dimensional Jacobian kernel. Reuses the LBR Med 14 kinematics.
template <typename Scalar>
auto make_redundant_7r_chain_dynamic() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    return make_lbr_med14_chain<Scalar>().to_dynamic();
}

/// High-DOF 12-joint arm (runtime DOF).
///
/// Twelve revolute joints, more than any six-dimensional task requires, so the
/// chain is highly redundant. The joint count exceeds ten, which is the case
/// that stresses a fixed-size Halton base table.
template <typename Scalar>
auto make_highdof_12r_chain_dynamic() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    using vec3 = cartan::vector3<Scalar>;
    constexpr int dof = 12;

    std::vector<cartan::screw_axis<Scalar>> axes;
    std::vector<cartan::joint_limits<Scalar>> limits;
    axes.reserve(dof);
    limits.reserve(dof);

    const cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    for (int i = 0; i < dof; ++i)
    {
        // Alternate the joint axis between +z and +y so the axis stack is not
        // fully parallel, and step each joint one decimeter higher than the
        // previous along world z. The precise geometry is immaterial here; the
        // fixture exists to exercise joint counts beyond six and beyond ten.
        const Scalar height = Scalar(0.1) * static_cast<Scalar>(i);
        const vec3 axis = (i % 2 == 0)
            ? vec3(Scalar(0), Scalar(0), Scalar(1))
            : vec3(Scalar(0), Scalar(1), Scalar(0));
        axes.push_back(cartan::screw_axis<Scalar>::revolute(
            axis, vec3(Scalar(0), Scalar(0), height)));
        limits.push_back(lim);
    }

    const Scalar tip = Scalar(0.1) * static_cast<Scalar>(dof);
    auto home = cartan::se3<Scalar>(
        cartan::so3<Scalar>::identity(),
        vec3(Scalar(0), Scalar(0), tip));

    return cartan::kinematic_chain<Scalar, cartan::dynamic>(
        home, std::move(axes), std::move(limits));
}

}

#endif
