#ifndef HPP_GUARD_LIEPP_SERIAL_IK_DETAIL_NABLAPP_PROBLEM_H
#define HPP_GUARD_LIEPP_SERIAL_IK_DETAIL_NABLAPP_PROBLEM_H

/// @file detail/nablapp_problem.h
/// @brief Adapter wrapping liepp IK problem as nablapp problem formulation.
///
/// Satisfies nablapp::objective, nablapp::differentiable, and
/// nablapp::bound_constrained concepts so that nablapp solvers
/// (kraft_slsqp_policy, bobyqa_policy) can optimize liepp IK objectives.

#include "liepp/serial/ik/error_weight.h"
#include "liepp/serial/ik/analytical_gradient.h"

#include "liepp/lie/se3.h"
#include "liepp/serial/chain/chain_concept.h"

#include <Eigen/Core>

namespace liepp::detail
{

/// Adapts a liepp IK problem (chain + target + weight) to the nablapp
/// problem formulation concepts (objective, differentiable, bound_constrained).
///
/// Uses ik_se3_objective for consistent objective and gradient evaluation
/// with SE(3) log Jacobian analytical gradient.
template <chain Chain>
class nablapp_ik_problem
{
    using Scalar = typename Chain::scalar_type;
    static constexpr int N = Chain::joints;
    using position_type = typename joint_state<Scalar, N>::position_type;

public:
    nablapp_ik_problem(
        const Chain& chain,
        const se3<Scalar>& target,
        const error_weight<Scalar>& weight)
        : m_chain{&chain}
        , m_target{target}
        , m_weight{weight}
    {}

    int dimension() const
    {
        return m_chain->num_joints();
    }

    double value(const Eigen::VectorXd& x) const
    {
        auto q = to_position(x);
        auto result = ik_se3_objective<Chain>::evaluate(*m_chain, m_target, q, m_weight);
        return static_cast<double>(result.objective);
    }

    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& g) const
    {
        auto q = to_position(x);
        auto result = ik_se3_objective<Chain>::evaluate_with_gradient(
            *m_chain, m_target, q, m_weight);

        int n = m_chain->num_joints();
        g.resize(n);
        for (int i = 0; i < n; ++i)
        {
            g[i] = static_cast<double>(result.gradient(i));
        }
    }

    Eigen::VectorXd lower_bounds() const
    {
        int n = m_chain->num_joints();
        Eigen::VectorXd lb(n);
        for (int i = 0; i < n; ++i)
        {
            lb[i] = static_cast<double>(m_chain->limits()[static_cast<std::size_t>(i)].position_min);
        }
        return lb;
    }

    Eigen::VectorXd upper_bounds() const
    {
        int n = m_chain->num_joints();
        Eigen::VectorXd ub(n);
        for (int i = 0; i < n; ++i)
        {
            ub[i] = static_cast<double>(m_chain->limits()[static_cast<std::size_t>(i)].position_max);
        }
        return ub;
    }

private:
    position_type to_position(const Eigen::VectorXd& x) const
    {
        int n = m_chain->num_joints();
        if constexpr (N == dynamic)
        {
            position_type q(n);
            for (int i = 0; i < n; ++i)
            {
                q[i] = static_cast<Scalar>(x[i]);
            }
            return q;
        }
        else
        {
            position_type q;
            for (int i = 0; i < n; ++i)
            {
                q[i] = static_cast<Scalar>(x[i]);
            }
            return q;
        }
    }

    const Chain* m_chain;
    se3<Scalar> m_target;
    error_weight<Scalar> m_weight;
};

}

#endif
