#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_SETUP_VALIDATION_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_SETUP_VALIDATION_H

/// The precondition every solver's setup() establishes once, so its iteration
/// loop can call the unchecked kinematics entry points.

#include "cartan/serial/ik/ik_status.h"

#include "cartan/lie/se3.h"

#include "cartan/expected.h"

namespace cartan::detail
{

/// Shape and finiteness of everything a solve reads from its caller, tested
/// once before the first forward-kinematics call.
///
/// Reports an `ik_status` rather than an `ik_failure` because the caller's
/// immediate need is a value to latch into its status member; the runner maps
/// status onto failure reason where it builds the error, which is where that
/// mapping already lives.
template <typename Chain, typename Scalar, typename Policy, typename Vector>
cartan::expected<void, ik_status> validate_solve_inputs(
    const Chain& chain,
    const se3<Scalar, Policy>& target,
    const Vector& q0)
{
    if (q0.size() != chain.num_joints())
    {
        return cartan::unexpected(ik_status::dimension_mismatch);
    }
    if (!q0.allFinite()
        || !target.translation().allFinite()
        || !target.rotation().quaternion_ref().coeffs().allFinite())
    {
        return cartan::unexpected(ik_status::non_finite_input);
    }
    return {};
}

/// The chain is a parameter of every solve policy's step(), not the object
/// setup() validated, so the setup-time shape check does not by itself bind the
/// chain the iteration reads. Comparing the joint count against the one setup
/// recorded restores the binding for an integer compare, without the
/// per-iteration shape scan the unchecked hot path exists to avoid.
template <typename Chain>
ik_status chain_bound_status(ik_status status, int setup_joints, const Chain& chain)
{
    if (status != ik_status::running)
    {
        return status;
    }
    return chain.num_joints() == setup_joints ? status : ik_status::dimension_mismatch;
}

/// The two statuses a failed precondition latches. A wrapper that restarts on a
/// terminal inner status tests this to tell a caller's bad argument, which no
/// fresh seed can repair, from a stalled or diverged attempt, which one can.
constexpr bool is_precondition_failure(ik_status status)
{
    return status == ik_status::dimension_mismatch
        || status == ik_status::non_finite_input;
}

/// The statuses from which no iteration may run: a failed precondition, or a
/// setup that never happened.
constexpr bool is_setup_failure(ik_status status)
{
    return status == ik_status::not_initialized || is_precondition_failure(status);
}

/// The failure reason a latched setup status is reported as.
constexpr ik_failure setup_failure_reason(ik_status status)
{
    if (status == ik_status::dimension_mismatch)
    {
        return ik_failure::dimension_mismatch;
    }
    if (status == ik_status::non_finite_input)
    {
        return ik_failure::non_finite_input;
    }
    return ik_failure::not_initialized;
}

}

#endif
