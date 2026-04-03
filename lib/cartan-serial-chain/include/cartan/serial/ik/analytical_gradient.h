#ifndef HPP_GUARD_CARTAN_SERIAL_IK_ANALYTICAL_GRADIENT_H
#define HPP_GUARD_CARTAN_SERIAL_IK_ANALYTICAL_GRADIENT_H

#include "cartan/types.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/ik/error_weight.h"
#include "cartan/lie/se3_left_jacobian.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics.h"

namespace cartan
{

/// Result of IK objective evaluation: scalar objective, body-frame error,
/// and weighted error (for convergence checking).
template <typename Scalar = double>
struct gradient_result
{
    Scalar objective;
    vector6<Scalar> body_error;
    vector6<Scalar> weighted_error;
};

/// IK objective and gradient computation using SE(3) log Jacobian.
///
/// Objective: f(q) = 0.5 * ||W * log(T_target^{-1} * FK(q))||^2
///
/// Gradient:  df/dq = J_b^T * J_log_inv^T * W^T * W * V_b
///
/// where V_b = log(T_target^{-1} * FK(q)),
///       J_log_inv = se3_left_jacobian_inv(V_b),
///       J_b = body Jacobian,
///       W = diag(weights).
///
/// Per Phase 09 fix: the gradient through log() requires J_log_inv.
/// This is the default ObjectivePolicy for L-BFGS-B.
template <chain Chain>
struct ik_se3_objective
{
    using Scalar = typename Chain::scalar_type;
    static constexpr int N = Chain::joints;
    using position_type = typename joint_state<Scalar, N>::position_type;

    /// Evaluate objective only (no gradient computation).
    [[nodiscard]] static gradient_result<Scalar> evaluate(
        const Chain& chain,
        const se3<Scalar>& target,
        const position_type& q,
        const error_weight<Scalar>& weight = {})
    {
        auto fk = forward_kinematics(chain, q);
        auto V_b = (target.inverse() * fk.end_effector).log();

        auto W_V = weight.apply(V_b);
        Scalar obj = Scalar(0.5) * W_V.squaredNorm();

        return {obj, V_b, W_V};
    }

    /// Evaluate objective and analytical gradient w.r.t. q.
    [[nodiscard]] static auto evaluate_with_gradient(
        const Chain& chain,
        const se3<Scalar>& target,
        const position_type& q,
        const error_weight<Scalar>& weight = {})
    {
        auto fk = forward_kinematics(chain, q);
        auto V_b = (target.inverse() * fk.end_effector).log();

        // Right-trivialized differential of log: J_r^{-1}(V_b) = J_l^{-1}(-V_b)
        auto J_log_inv = se3_left_jacobian_inv(vector6<Scalar>(-V_b));
        auto J_b = body_jacobian(chain, fk);

        auto W_V = weight.apply(V_b);
        Scalar obj = Scalar(0.5) * W_V.squaredNorm();

        vector6<Scalar> W2_V = weight.weights.cwiseProduct(W_V);
        position_type grad = J_b.transpose() * (J_log_inv.transpose() * W2_V);

        struct result
        {
            gradient_result<Scalar> info;
            position_type gradient;
        };
        return result{{obj, V_b, W_V}, grad};
    }
};

}

#endif
