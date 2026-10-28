#ifndef HPP_GUARD_CARTAN_TESTS_COMPILE_NO_EXCEPTIONS_KINEMATICS_SLICE_H
#define HPP_GUARD_CARTAN_TESTS_COMPILE_NO_EXCEPTIONS_KINEMATICS_SLICE_H

// The evaluation half of the exceptions-off compile gate: every checked
// kinematics entry point, on its success branch and, where the argument shape
// makes one reachable, on its failure branch.
//
// Each family is instantiated for both the compile-time-tagged chain and the
// runtime one, because those are distinct templates: covering one leaves the
// other's code generation unexamined, which is how the body Jacobian and the
// velocity entry point stayed uncovered while a bare-metal consumer already
// called the first of them.

#include "no_exceptions_chain_slice.h"

#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/velocity.h"
#include "cartan/serial/fk/forward_kinematics.h"

namespace cartan::compile_gate
{

inline float checked_forward_kinematics()
{
    float acc = 0.0F;

    auto chain = unit_chain();
    if (!chain.has_value())
    {
        return acc;
    }

    auto fk = forward_kinematics(*chain, unit_joint_vector(0.25F));
    if (fk.has_value())
    {
        acc += fk->end_effector.translation().sum();
    }

    auto refused = forward_kinematics(*chain, Eigen::VectorXf::Zero(2));
    if (!refused.has_value())
    {
        acc += static_cast<float>(static_cast<int>(refused.error()));
    }

    return acc;
}

/// Both Jacobian families on the compile-time-tagged chain. The space and body
/// forms are separate templates: instantiating one leaves the other's code
/// generation unexamined, which is how the body form stayed uncovered while a
/// bare-metal consumer already called it.
inline float checked_jacobians()
{
    float acc = 0.0F;

    auto chain = unit_chain();
    if (!chain.has_value())
    {
        return acc;
    }
    auto fk = forward_kinematics(*chain, unit_joint_vector(0.25F));
    if (!fk.has_value())
    {
        return acc;
    }

    auto space = space_jacobian(*chain, *fk);
    auto body = body_jacobian(*chain, *fk);
    acc += space.has_value() ? space->sum() : 0.0F;
    acc += body.has_value() ? body->sum() : 0.0F;

    return acc;
}

/// The runtime-chain overloads, which are distinct templates from the tagged
/// ones above and are the pair the bare-metal translation unit actually calls.
inline float runtime_chain_kinematics(const kinematic_chain<float, 1>& chain)
{
    auto fk = forward_kinematics(chain, unit_joint_vector(0.25F));
    if (!fk.has_value())
    {
        return 0.0F;
    }
    auto body = body_jacobian(chain, *fk);
    return body.has_value() ? body->sum() : 0.0F;
}

inline float checked_velocity()
{
    auto chain = unit_runtime_chain();
    if (!chain.has_value())
    {
        return 0.0F;
    }

    float acc = 0.0F;
    auto twist = end_effector_velocity(
        *chain, unit_joint_vector(0.25F), unit_joint_vector(0.5F));
    if (twist.has_value())
    {
        acc += twist->sum();
    }

    auto refused = end_effector_velocity(
        *chain, unit_joint_vector(0.25F), Eigen::VectorXf::Zero(2));
    if (!refused.has_value())
    {
        acc += static_cast<float>(static_cast<int>(refused.error()));
    }

    return acc + runtime_chain_kinematics(*chain);
}

}

#endif
