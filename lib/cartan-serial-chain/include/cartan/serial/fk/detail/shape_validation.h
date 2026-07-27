#ifndef HPP_GUARD_CARTAN_SERIAL_FK_DETAIL_SHAPE_VALIDATION_H
#define HPP_GUARD_CARTAN_SERIAL_FK_DETAIL_SHAPE_VALIDATION_H

/// Shape and finiteness predicates shared by the checked kinematics entry
/// points.

#include "cartan/expected.h"

#include "cartan/serial/chain/chain_failure.h"

namespace cartan::detail
{

/// Length first, then finiteness on the raw input, before any arithmetic can
/// touch it: an infinity multiplied by a zero becomes a NaN, so a guard placed
/// after the accumulation sees a NaN where it expected an infinity and lets it
/// through.
template <typename Chain, typename Vector>
cartan::expected<void, chain_failure>
check_joint_vector(const Chain& chain, const Vector& values)
{
    if (values.size() != chain.num_joints())
    {
        return cartan::unexpected(chain_failure::dimension_mismatch);
    }
    if (!values.allFinite())
    {
        return cartan::unexpected(chain_failure::non_finite_input);
    }
    return {};
}

/// The two joint vectors carry the same contract; the pair of names exists so a
/// call site says which of them it is validating.
template <typename Chain, typename Vector>
cartan::expected<void, chain_failure>
check_joint_positions(const Chain& chain, const Vector& q)
{
    return check_joint_vector(chain, q);
}

template <typename Chain, typename Vector>
cartan::expected<void, chain_failure>
check_joint_velocities(const Chain& chain, const Vector& dq)
{
    return check_joint_vector(chain, dq);
}

/// Structural precondition of the Jacobian family, which takes a cached
/// forward-kinematics result rather than a joint vector.
///
/// Length equality is not provenance: a result of the right length computed
/// from a different chain of the same joint count satisfies this predicate.
/// Binding a result to the chain that produced it is deliberately out of scope.
/// The contents are not examined either — the result is library-produced, and
/// the entry points able to produce a nonfinite one are checked at their own
/// boundary.
template <typename Chain, typename FkResult>
cartan::expected<void, chain_failure>
check_fk_shape(const Chain& chain, const FkResult& fk)
{
    if (fk.num_joints() != chain.num_joints())
    {
        return cartan::unexpected(chain_failure::dimension_mismatch);
    }
    return {};
}

}

#endif
