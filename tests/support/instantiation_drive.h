#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_INSTANTIATION_DRIVE_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_INSTANTIATION_DRIVE_H

// The bounded drive the instantiation targets share: a combination is
// constructed, set up and stepped once, which is what instantiates the member
// bodies that including a class template leaves unchecked.
//
// The two-iteration, four-work-unit bound is what keeps that probe from turning
// into a solve, and the assertions read a status rather than a solution because
// an under-actuated chain legitimately fails to reach a general pose.

#include "kinematics_helpers.h"

#include <cartan/serial/ik/ik_status.h>

#include <cartan/lie/se3.h>

#include <cartan/serial/chain/joint_state.h>

#include <catch2/catch_test_macros.hpp>

namespace cartan::testing
{

template <typename Scalar>
convergence_criteria<Scalar> bounded_criteria()
{
    convergence_criteria<Scalar> criteria;
    criteria.max_iterations_per_attempt = 2;
    criteria.max_total_work_units = 4;
    return criteria;
}

template <typename Chain>
typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type
seeded_configuration(const Chain& chain)
{
    using scalar_type = typename Chain::scalar_type;
    using position_type = typename joint_state<scalar_type, Chain::joints>::position_type;

    position_type q = position_type::Zero(chain.num_joints());
    for (int i = 0; i < chain.num_joints(); ++i)
    {
        q(i) = scalar_type(0.1) * scalar_type(i + 1);
    }
    return q;
}

template <typename Chain>
se3<typename Chain::scalar_type> reachable_target(const Chain& chain)
{
    return fk_at(chain, seeded_configuration(chain)).end_effector;
}

template <typename Policy, typename Chain>
void drive_stepped(const Chain& chain, const se3<typename Chain::scalar_type>& target)
{
    using scalar_type = typename Chain::scalar_type;

    Policy policy;
    policy.setup(chain, target, seeded_configuration(chain), bounded_criteria<scalar_type>());
    step_result<scalar_type> stepped = policy.step(chain, 1);

    CHECK(policy.status() != ik_status::not_initialized);
    CHECK(stepped.status != ik_status::not_initialized);
    CHECK(policy.error_norm() >= scalar_type(0));
}

}

#endif
