#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_ARGMIN_UNCONSTRAINED_PROBLEM_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_ARGMIN_UNCONSTRAINED_PROBLEM_H

/// @file detail/argmin_unconstrained_problem.h
/// @brief Unconstrained adapter (formulation A) for argmin solvers.
///
/// Satisfies argmin::objective and argmin::differentiable without
/// bounds. Joint limits are enforced externally by the solve policy
/// via clamping after each step.

#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/solver/detail/analytical_gradient.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/chain_concept.h"

#include <Eigen/Core>

namespace cartan::detail
{

/// Unconstrained IK problem adapter for argmin (formulation A).
///
/// Provides objective and gradient only. No bounds are exposed to
/// the solver; the calling solve policy is responsible for clamping
/// the iterate to joint limits after each argmin step.
template <chain Chain>
class argmin_unconstrained_ik_problem
{
    using Scalar = typename Chain::scalar_type;
    static constexpr int N = Chain::joints;
    using position_type = typename joint_state<Scalar, N>::position_type;

public:
    static constexpr int problem_dimension = N;

    argmin_unconstrained_ik_problem(
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
