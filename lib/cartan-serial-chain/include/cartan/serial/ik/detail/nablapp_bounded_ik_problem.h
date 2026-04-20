#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_NABLAPP_BOUNDED_IK_PROBLEM_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_NABLAPP_BOUNDED_IK_PROBLEM_H

/// @file detail/nablapp_bounded_ik_problem.h
/// @brief Bound-constrained adapter for nablapp solvers that treat joint
///        limits as box constraints (no inequality constraints).
///
/// Satisfies nablapp::objective, nablapp::differentiable, and
/// nablapp::bound_constrained. Joint limits are returned from
/// lower_bounds() / upper_bounds() so policies with native box-constrained
/// handling (mma_policy, gcmma_policy, lbfgsb_policy) use the natural
/// formulation rather than encoding limits as 2n redundant inequality
/// constraints. num_equality() / num_inequality() both return zero so
/// the same adapter is usable by MMA / GCMMA (which assert the equality
/// count is zero and skip constraint evaluation when the inequality
/// count is zero).

#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/solver/detail/analytical_gradient.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/chain/joint_state.h"

#include <Eigen/Core>

#include <cstddef>

namespace cartan::detail
{

/// Bound-constrained IK problem adapter for nablapp.
///
/// Exposes the IK SE(3) log-error objective and analytical gradient,
/// plus joint limits as box bounds. No inequality or equality
/// constraints are reported, keeping the formulation clean for native
/// bound-constrained policies (MMA / GCMMA / L-BFGS-B).
template <chain Chain>
class nablapp_bounded_ik_problem
{
    using Scalar = typename Chain::scalar_type;
    static constexpr int N = Chain::joints;
    using position_type = typename joint_state<Scalar, N>::position_type;

public:
    static constexpr int problem_dimension = N;

    nablapp_bounded_ik_problem(
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

    int num_equality() const { return 0; }
    int num_inequality() const { return 0; }

    // Stub methods required by nablapp::constrained concept (MMA / GCMMA
    // static_assert constrained<Problem>). Never invoked at runtime
    // because both counts above are zero, but must be present for the
    // concept to be satisfied.
    template <typename DerivedIn, typename DerivedOut>
    void constraints(const Eigen::MatrixBase<DerivedIn>&, Eigen::MatrixBase<DerivedOut>&) const
    {}

    template <typename DerivedIn, typename DerivedOut>
    void constraint_jacobian(const Eigen::MatrixBase<DerivedIn>&, Eigen::MatrixBase<DerivedOut>&) const
    {}

    Eigen::Vector<double, N> lower_bounds() const
    {
        int n = m_chain->num_joints();
        Eigen::Vector<double, N> lo;
        if constexpr (N == dynamic)
        {
            lo.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            lo[i] = static_cast<double>(
                m_chain->limits()[static_cast<std::size_t>(i)].position_min);
        }
        return lo;
    }

    Eigen::Vector<double, N> upper_bounds() const
    {
        int n = m_chain->num_joints();
        Eigen::Vector<double, N> hi;
        if constexpr (N == dynamic)
        {
            hi.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            hi[i] = static_cast<double>(
                m_chain->limits()[static_cast<std::size_t>(i)].position_max);
        }
        return hi;
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
