#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_NABLAPP_PROBLEM_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_NABLAPP_PROBLEM_H

/// @file detail/nablapp_problem.h
/// @brief Adapter wrapping cartan IK problem as nablapp problem formulation.
///
/// Satisfies nablapp::objective, nablapp::differentiable, and
/// nablapp::bound_constrained concepts so that nablapp solvers
/// (kraft_slsqp_policy, bobyqa_policy) can optimize cartan IK objectives.

#include "cartan/serial/ik/error_weight.h"
#include "cartan/serial/ik/analytical_gradient.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/chain_concept.h"

#include <Eigen/Core>

namespace cartan::detail
{

/// Adapts a cartan IK problem (chain + target + weight) to the nablapp
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
    static constexpr int problem_dimension = N;

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

    template <typename Derived>
    double value(const Eigen::MatrixBase<Derived>& x) const
    {
        auto q = to_position(x);
        auto result = ik_se3_objective<Chain>::evaluate(*m_chain, m_target, q, m_weight);
        return static_cast<double>(result.objective);
    }

    template <typename DerivedIn, typename DerivedOut>
    void gradient(const Eigen::MatrixBase<DerivedIn>& x, Eigen::MatrixBase<DerivedOut>& g) const
    {
        auto q = to_position(x);
        auto result = ik_se3_objective<Chain>::evaluate_with_gradient(
            *m_chain, m_target, q, m_weight);

        int n = m_chain->num_joints();
        for (int i = 0; i < n; ++i)
        {
            g[i] = static_cast<double>(result.gradient(i));
        }
    }

    Eigen::Vector<double, N> lower_bounds() const
    {
        int n = m_chain->num_joints();
        Eigen::Vector<double, N> lb;
        if constexpr (N == dynamic)
        {
            lb.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            lb[i] = static_cast<double>(m_chain->limits()[static_cast<std::size_t>(i)].position_min);
        }
        return lb;
    }

    Eigen::Vector<double, N> upper_bounds() const
    {
        int n = m_chain->num_joints();
        Eigen::Vector<double, N> ub;
        if constexpr (N == dynamic)
        {
            ub.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            ub[i] = static_cast<double>(m_chain->limits()[static_cast<std::size_t>(i)].position_max);
        }
        return ub;
    }

    template <typename Derived>
    position_type to_position(const Eigen::MatrixBase<Derived>& x) const
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

private:
    const Chain* m_chain;
    se3<Scalar> m_target;
    error_weight<Scalar> m_weight;
};

}

#endif
