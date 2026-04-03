#ifndef HPP_GUARD_LIEPP_SERIAL_IK_DETAIL_NABLAPP_CONSTRAINED_PROBLEM_H
#define HPP_GUARD_LIEPP_SERIAL_IK_DETAIL_NABLAPP_CONSTRAINED_PROBLEM_H

/// @file detail/nablapp_constrained_problem.h
/// @brief Inequality-constrained adapter (formulation B) for nablapp solvers.
///
/// Satisfies nablapp::objective, nablapp::differentiable, and
/// nablapp::constrained. Joint limits are expressed as 2n inequality
/// constraints: q_i - q_min >= 0 and q_max - q_i >= 0.

#include "liepp/serial/ik/error_weight.h"
#include "liepp/serial/ik/analytical_gradient.h"

#include "liepp/lie/se3.h"
#include "liepp/serial/chain/chain_concept.h"

#include <Eigen/Core>

#include <cstddef>

namespace liepp::detail
{

/// Inequality-constrained IK problem adapter for nablapp (formulation B).
///
/// Provides objective, gradient, and inequality constraints encoding
/// joint limits as g_i(q) >= 0. No equality constraints.
template <chain Chain>
class nablapp_constrained_ik_problem
{
    using Scalar = typename Chain::scalar_type;
    static constexpr int N = Chain::joints;
    using position_type = typename joint_state<Scalar, N>::position_type;

public:
    static constexpr int problem_dimension = N;

    nablapp_constrained_ik_problem(
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

    int num_equality() const
    {
        return 0;
    }

    int num_inequality() const
    {
        return 2 * m_chain->num_joints();
    }

    template <typename DerivedIn, typename DerivedOut>
    void constraints(const Eigen::MatrixBase<DerivedIn>& x, Eigen::MatrixBase<DerivedOut>& c) const
    {
        int n = m_chain->num_joints();
        for (int i = 0; i < n; ++i)
        {
            c[i] = x[i] - static_cast<double>(
                m_chain->limits()[static_cast<std::size_t>(i)].position_min);
            c[n + i] = static_cast<double>(
                m_chain->limits()[static_cast<std::size_t>(i)].position_max) - x[i];
        }
    }

    template <typename DerivedIn, typename DerivedOut>
    void constraint_jacobian(const Eigen::MatrixBase<DerivedIn>& /*x*/, Eigen::MatrixBase<DerivedOut>& J) const
    {
        int n = m_chain->num_joints();
        J.setZero();
        for (int i = 0; i < n; ++i)
        {
            J(i, i) = 1.0;
            J(n + i, i) = -1.0;
        }
    }

private:
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

    const Chain* m_chain;
    se3<Scalar> m_target;
    error_weight<Scalar> m_weight;
};

}

#endif
