#ifndef HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_FK_VERIFICATION_H
#define HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_FK_VERIFICATION_H

#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include "cartan/lie/se3.h"

namespace cartan::detail
{

/// FK back-verification used by every analytical solver to filter candidate
/// solutions. Templated against the chain concept so both static_chain and
/// dynamic kinematic_chain consumers share one implementation. The candidate
/// vector q is templated on its compile-time size N (independent of the
/// chain's own `joints` constant) because the solvers always produce
/// algorithm-sized fixed vectors (2 for 2R, 3 for 3R, 6 for 6R) while the
/// chain itself may be statically- or dynamically-sized; the helper bridges
/// the two by constructing a runtime-sized copy of q when the chain is
/// dynamic, and forwarding directly otherwise.
template <chain Chain, int N>
bool verify_analytical_solution(
    const Chain& chain,
    const Eigen::Vector<typename Chain::scalar_type, N>& q,
    const se3<typename Chain::scalar_type>& target,
    bool check_orientation,
    typename Chain::scalar_type position_tolerance = typename Chain::scalar_type(1e-6),
    typename Chain::scalar_type orientation_tolerance = typename Chain::scalar_type(1e-6))
{
    using Scalar = typename Chain::scalar_type;

    auto fk = [&]
    {
        if constexpr (Chain::joints == N)
        {
            return forward_kinematics(chain, q);
        }
        else
        {
            // Chain is dynamic (or has a different compile-time N than the
            // solver's algorithmic joint count); construct a runtime-sized
            // copy of q before invoking the chain-concept FK overload.
            Eigen::Vector<Scalar, Eigen::Dynamic> q_dyn(N);
            for (int i = 0; i < N; ++i)
            {
                q_dyn(i) = q(i);
            }
            return forward_kinematics(chain, q_dyn);
        }
    }();

    Scalar position_error = (fk.end_effector.translation() - target.translation()).norm();
    if (position_error >= position_tolerance)
        return false;
    if (!check_orientation)
        return true;
    Scalar orientation_error = (fk.end_effector.rotation().inverse() * target.rotation()).log().norm();
    return orientation_error < orientation_tolerance;
}

}

#endif
