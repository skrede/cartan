#ifndef HPP_GUARD_LIEPP_SERIAL_IK_DETAIL_CONVERGENCE_H
#define HPP_GUARD_LIEPP_SERIAL_IK_DETAIL_CONVERGENCE_H

#include "liepp/serial/ik/ik_types.h"
#include "liepp/serial/ik/error_weight.h"

#include <Eigen/Core>

namespace liepp::detail
{

/// Weighted convergence check for IK policies using error_weight.
///
/// Used by lm, projected_lm, and lbfgsb policies that apply
/// position/orientation weighting to the body-frame error.
template <typename Scalar>
bool is_converged(
    const Eigen::Vector<Scalar, 6>& body_error,
    const error_weight<Scalar>& weight,
    const convergence_criteria<Scalar>& criteria)
{
    return weight.weighted_angular_norm(body_error) < criteria.orientation_tol
        && weight.weighted_linear_norm(body_error) < criteria.position_tol;
}

/// Unweighted convergence check for IK policies using raw norms.
///
/// Used by dls and newton_raphson policies that check angular and
/// linear components of the body-frame error directly.
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
