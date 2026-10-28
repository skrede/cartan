#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_CONVERGENCE_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_CONVERGENCE_H

#include "cartan/serial/ik/ik_status.h"

#include <Eigen/Core>

namespace cartan::detail
{

/// The stopping test every solve policy shares: raw angular and linear
/// component norms of the body-frame error. A configured task weight steers
/// the step, never the gate, so all policies agree on what "converged" means.
template <typename Scalar>
bool is_converged_unweighted(
    const Eigen::Vector<Scalar, 6>& body_error,
    const convergence_criteria<Scalar>& criteria)
{
    return body_error.template head<3>().norm() < criteria.orientation_tol
        && body_error.template tail<3>().norm() < criteria.position_tol;
}

}

#endif
