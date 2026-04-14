#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_NABLAPP_LEAST_SQUARES_PROBLEM_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_NABLAPP_LEAST_SQUARES_PROBLEM_H

/// @file detail/nablapp_least_squares_problem.h
/// @brief Least-squares adapter for nablapp LM solver.
///
/// Satisfies nablapp::objective and nablapp::least_squares concepts.
/// Exposes the 6-element SE(3) body-frame error as the residual vector
/// and the body Jacobian as the LM Jacobian.

#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/chain_concept.h"

#include <Eigen/Core>

namespace cartan::detail
{

/// Least-squares IK problem adapter for nablapp LM solver.
///
/// The residual vector is the 6-element body-frame error V_b = log(T_target^{-1} * FK(q)).
/// The Jacobian is the body Jacobian J_b (6 x n), which is the first-order
/// approximation of d(V_b)/dq (neglecting the SE(3) log Jacobian correction,
/// which is accurate near convergence).
template <chain Chain>
class nablapp_ik_least_squares_problem
{
    using Scalar = typename Chain::scalar_type;
    static constexpr int N = Chain::joints;
    using position_type = typename joint_state<Scalar, N>::position_type;

public:
    static constexpr int problem_dimension = N;

    nablapp_ik_least_squares_problem(
        const Chain& chain,
        const se3<Scalar>& target)
        : m_chain{&chain}
        , m_target{target}
    {}

    int dimension() const
    {
        return m_chain->num_joints();
    }

    int num_residuals() const
    {
        return 6;
    }

    template <typename Derived>
    double value(const Eigen::MatrixBase<Derived>& x) const
    {
        auto q = to_position(x);
        auto fk = forward_kinematics(*m_chain, q);
        auto V_b = (m_target.inverse() * fk.end_effector).log();
        return 0.5 * static_cast<double>(V_b.squaredNorm());
    }

    template <typename DerivedIn, typename DerivedOut>
    void residuals(const Eigen::MatrixBase<DerivedIn>& x, Eigen::MatrixBase<DerivedOut>& r) const
    {
        auto q = to_position(x);
        auto fk = forward_kinematics(*m_chain, q);
        auto V_b = (m_target.inverse() * fk.end_effector).log();
        for (int i = 0; i < 6; ++i)
        {
            r[i] = static_cast<double>(V_b(i));
        }
    }

    template <typename DerivedIn, typename DerivedOut>
    void jacobian(const Eigen::MatrixBase<DerivedIn>& x, Eigen::MatrixBase<DerivedOut>& J) const
    {
        auto q = to_position(x);
        auto fk = forward_kinematics(*m_chain, q);
        auto J_b = body_jacobian(*m_chain, fk);
        int n = m_chain->num_joints();
        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                J(i, j) = static_cast<double>(J_b(i, j));
            }
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
};

}

#endif
