#ifndef HPP_GUARD_LIEPP_SERIAL_FK_JACOBIAN_H
#define HPP_GUARD_LIEPP_SERIAL_FK_JACOBIAN_H

/// @file jacobian.h
/// @brief Space and body Jacobian computation for kinematic chains.
///
/// The space Jacobian J_s maps joint velocities to the end-effector
/// spatial twist. The body Jacobian J_b maps to the body-frame twist.
/// Both reuse cached intermediate products from fk_result to avoid
/// redundant exp() calls.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 5, p. 174-190.

#include "liepp/types.h"

#include "liepp/serial/fk/fk_result.h"

#include "liepp/serial/chain/chain_concept.h"
#include "liepp/serial/chain/kinematic_chain.h"

#include <utility>
#include <type_traits>

namespace liepp
{

/// Jacobian matrix type: 6xN fixed for compile-time N, 6xDynamic otherwise.
template <typename Scalar, int N>
using jacobian_matrix = std::conditional_t<
    N == dynamic,
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic>,
    Eigen::Matrix<Scalar, 6, N>>;

namespace detail
{

/// Runtime-loop space Jacobian for dynamic or large (N >= 8) chains.
/// J_si = Ad_{T_{i-1}}(S_i), where T_0 = I (identity).
/// Lynch & Park, Modern Robotics, Eq. 5.11, p. 178.
template <typename Scalar, int N>
jacobian_matrix<Scalar, N> space_jacobian_loop(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk)
{
    const auto& axes = chain.axes();
    int n = chain.num_joints();

    jacobian_matrix<Scalar, N> J;
    if constexpr (N == dynamic)
    {
        J.resize(6, n);
    }

    // Column 0: J_s0 = S_0 (Ad_{T_{-1}} = Ad_I = I)
    J.col(0) = axes[0].to_vector();

    // Columns 1..n-1: J_si = Ad_{T_{i-1}} * S_i
    for (int i = 1; i < n; ++i)
    {
        J.col(i) = fk.intermediates[static_cast<std::size_t>(i - 1)].adjoint()
                    * axes[static_cast<std::size_t>(i)].to_vector();
    }

    return J;
}

/// Compile-time unrolled space Jacobian for fixed-size chains (N=1-7).
/// Uses index_sequence fold expression for zero-overhead expansion.
/// Lynch & Park, Modern Robotics, Eq. 5.11, p. 178.
template <typename Scalar, int N, std::size_t... Is>
jacobian_matrix<Scalar, N> space_jacobian_unrolled_impl(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk,
    std::index_sequence<Is...>)
{
    const auto& axes = chain.axes();
    jacobian_matrix<Scalar, N> J;

    ((J.col(static_cast<int>(Is)) =
        (Is == 0)
            ? axes[0].to_vector()
            : fk.intermediates[Is - 1].adjoint() * axes[Is].to_vector()),
     ...);

    return J;
}

/// Entry point for compile-time unrolled space Jacobian.
template <typename Scalar, int N>
jacobian_matrix<Scalar, N> space_jacobian_unrolled(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk)
{
    return space_jacobian_unrolled_impl(
        chain, fk, std::make_index_sequence<static_cast<std::size_t>(N)>{});
}

} // namespace detail

/// Space Jacobian: J_si(q) = Ad_{T_{i-1}}(S_i).
///
/// Maps joint velocities to end-effector spatial twist: V_s = J_s(q) * dq.
/// Uses cached intermediate products from fk_result for efficiency.
///
/// For fixed-size chains with N=1-7 joints, dispatches to a compile-time
/// unrolled fold expression. For dynamic or larger chains, uses a runtime loop.
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.11, p. 178.
///
/// @tparam N      Number of joints (compile-time), or liepp::dynamic.
/// @tparam Scalar Floating-point type.
/// @param chain   Kinematic chain with screw axes.
/// @param fk      Forward kinematics result with cached intermediates.
/// @return        6xN space Jacobian matrix.
template <typename Scalar, int N>
jacobian_matrix<Scalar, N> space_jacobian(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk)
{
    if constexpr (N >= 1 && N <= 7)
    {
        return detail::space_jacobian_unrolled(chain, fk);
    }
    else
    {
        return detail::space_jacobian_loop(chain, fk);
    }
}

/// Body Jacobian: J_b(q) = [Ad_{T^{-1}}] * J_s(q).
///
/// Maps joint velocities to end-effector body-frame twist: V_b = J_b(q) * dq.
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.22, p. 185.
///
/// @tparam N      Number of joints (compile-time), or liepp::dynamic.
/// @tparam Scalar Floating-point type.
/// @param chain   Kinematic chain with screw axes.
/// @param fk      Forward kinematics result with cached intermediates.
/// @return        6xN body Jacobian matrix.
template <typename Scalar, int N>
jacobian_matrix<Scalar, N> body_jacobian(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk)
{
    auto J_s = space_jacobian(chain, fk);
    matrix6<Scalar> Ad_inv = fk.end_effector.inverse().adjoint();
    return Ad_inv * J_s;
}

/// Generic space Jacobian for any chain type satisfying the chain concept.
///
/// J_si(q) = Ad_{T_{i-1}}(S_i), where T_0 = I (identity).
/// The existing kinematic_chain overload wins via partial ordering.
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.11, p. 178.
template <chain Chain>
[[nodiscard]] jacobian_matrix<typename Chain::scalar_type, Chain::joints>
space_jacobian(
    const Chain& chain,
    const fk_result<typename Chain::scalar_type, Chain::joints>& fk)
{
    using Scalar = typename Chain::scalar_type;
    constexpr int N = Chain::joints;
    int n = chain.num_joints();

    jacobian_matrix<Scalar, N> J;

    J.col(0) = chain.axis(0).to_vector();

    for (int i = 1; i < n; ++i)
    {
        J.col(i) = fk.intermediates[static_cast<std::size_t>(i - 1)].adjoint()
                    * chain.axis(i).to_vector();
    }

    return J;
}

/// Generic body Jacobian for any chain type satisfying the chain concept.
///
/// J_b(q) = [Ad_{T^{-1}}] * J_s(q).
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.22, p. 185.
template <chain Chain>
[[nodiscard]] jacobian_matrix<typename Chain::scalar_type, Chain::joints>
body_jacobian(
    const Chain& chain,
    const fk_result<typename Chain::scalar_type, Chain::joints>& fk)
{
    auto J_s = space_jacobian(chain, fk);
    matrix6<typename Chain::scalar_type> Ad_inv = fk.end_effector.inverse().adjoint();
    return Ad_inv * J_s;
}

}

#endif
