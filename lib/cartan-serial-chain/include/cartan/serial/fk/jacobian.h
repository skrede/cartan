#ifndef HPP_GUARD_CARTAN_SERIAL_FK_JACOBIAN_H
#define HPP_GUARD_CARTAN_SERIAL_FK_JACOBIAN_H

/// Space and body Jacobian computation for kinematic chains.
///
/// The space Jacobian J_s maps joint velocities to the end-effector
/// spatial twist. The body Jacobian J_b maps to the body-frame twist.
/// Both reuse cached intermediate products from fk_result to avoid
/// redundant exp() calls.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 5, p. 174-190.

#include "cartan/types.h"

#include "cartan/serial/fk/fk_result.h"
#include "cartan/serial/fk/detail/shape_validation.h"
#include "cartan/serial/fk/detail/axis_specializations.h"

#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/chain/static_chain.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include <tuple>
#include <utility>
#include <type_traits>

namespace cartan
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

    if (n == 0)
    {
        return J;
    }

    // Column 0: J_s0 = S_0 (Ad_{T_{-1}} = Ad_I = I)
    detail::jacobian_column_identity_runtime(chain.kind(0), J.col(0), axes[0]);

    // Columns 1..n-1: J_si = Ad_{T_{i-1}} * S_i
    for (int i = 1; i < n; ++i)
    {
        detail::jacobian_column_runtime(
            chain.kind(i),
            J.col(i),
            fk.intermediates[static_cast<std::size_t>(i - 1)],
            axes[static_cast<std::size_t>(i)]);
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

    ([&]<std::size_t I>()
    {
        if constexpr (I == 0)
        {
            jacobian_column_identity_runtime(
                chain.kind(0), J.col(0), axes[0]);
        }
        else
        {
            jacobian_column_runtime(
                chain.kind(static_cast<int>(I)),
                J.col(static_cast<int>(I)),
                fk.intermediates[I - 1],
                axes[I]);
        }
    }.template operator()<Is>(), ...);

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

}

/// Space Jacobian for a caller that has already established that fk holds one
/// intermediate product per joint of chain. This family takes no joint vector,
/// so its structural precondition is on the provenance of the cached result:
/// a result with fewer intermediates than the chain has joints reads past the
/// end of the cache, and one with more yields a matrix for a different chain.
/// Neither is checked here, and violating either is undefined behavior.
///
/// The suffix marks a structural precondition between arguments, and is a
/// different claim from the `trusted` vocabulary, which marks a mathematical
/// invariant carried by one value.
///
/// J_si(q) = Ad_{T_{i-1}}(S_i). For fixed-size chains with N=1-7 joints,
/// dispatches to a compile-time unrolled fold expression. For dynamic or larger
/// chains, uses a runtime loop.
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.11, p. 178.
template <typename Scalar, int N>
jacobian_matrix<Scalar, N> space_jacobian_unchecked(
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

/// Space Jacobian: J_si(q) = Ad_{T_{i-1}}(S_i).
///
/// Maps joint velocities to end-effector spatial twist: V_s = J_s(q) * dq.
/// Uses cached intermediate products from fk_result for efficiency.
template <typename Scalar, int N>
cartan::expected<jacobian_matrix<Scalar, N>, chain_failure> space_jacobian(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk)
{
    auto shape = detail::check_fk_shape(chain, fk);
    if (!shape)
    {
        return cartan::unexpected(shape.error());
    }
    return space_jacobian_unchecked(chain, fk);
}

/// Body Jacobian under the same unchecked precondition on fk as the space
/// Jacobian above.
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.22, p. 185.
template <typename Scalar, int N>
jacobian_matrix<Scalar, N> body_jacobian_unchecked(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk)
{
    auto J_s = space_jacobian_unchecked(chain, fk);
    matrix6<Scalar> Ad_inv = fk.end_effector.inverse().adjoint();
    return Ad_inv * J_s;
}

/// Body Jacobian: J_b(q) = [Ad_{T^{-1}}] * J_s(q).
///
/// Maps joint velocities to end-effector body-frame twist: V_b = J_b(q) * dq.
template <typename Scalar, int N>
cartan::expected<jacobian_matrix<Scalar, N>, chain_failure> body_jacobian(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk)
{
    auto shape = detail::check_fk_shape(chain, fk);
    if (!shape)
    {
        return cartan::unexpected(shape.error());
    }
    return body_jacobian_unchecked(chain, fk);
}

/// Specialized space Jacobian for static_chain exploiting compile-time
/// joint tag knowledge, under the same unchecked precondition on fk. Computes
/// each column via axis-specific adjoint-screw helpers that use column
/// extraction and sparse multiply instead of the full 6x6 adjoint matrix.
template <typename Scalar, joint_tag... Joints>
jacobian_matrix<Scalar, static_cast<int>(sizeof...(Joints))>
space_jacobian_unchecked(
    const static_chain<Scalar, Joints...>& chain,
    const fk_result<Scalar, static_cast<int>(sizeof...(Joints))>& fk)
{
    constexpr int N = static_cast<int>(sizeof...(Joints));
    jacobian_matrix<Scalar, N> J;

    [&]<std::size_t... Is>(std::index_sequence<Is...>)
    {
        using joint_tuple = std::tuple<Joints...>;
        ((
            [&]<std::size_t I>()
            {
                using Joint = std::tuple_element_t<I, joint_tuple>;
                if constexpr (I == 0)
                {
                    detail::jacobian_column_identity<Joint>(
                        J.col(0), chain.axis(0));
                }
                else
                {
                    detail::jacobian_column<Joint>(
                        J.col(static_cast<int>(I)),
                        fk.intermediates[I - 1],
                        chain.axis(static_cast<int>(I)));
                }
            }.template operator()<Is>()
        ), ...);
    }(std::make_index_sequence<static_cast<std::size_t>(N)>{});

    return J;
}

/// Specialized space Jacobian for static_chain.
template <typename Scalar, joint_tag... Joints>
cartan::expected<jacobian_matrix<Scalar, static_cast<int>(sizeof...(Joints))>, chain_failure>
space_jacobian(
    const static_chain<Scalar, Joints...>& chain,
    const fk_result<Scalar, static_cast<int>(sizeof...(Joints))>& fk)
{
    auto shape = detail::check_fk_shape(chain, fk);
    if (!shape)
    {
        return cartan::unexpected(shape.error());
    }
    return space_jacobian_unchecked(chain, fk);
}

/// Specialized body Jacobian for static_chain, under the same unchecked
/// precondition on fk. Delegates to the specialized space Jacobian and applies
/// Ad_{T^{-1}}.
template <typename Scalar, joint_tag... Joints>
jacobian_matrix<Scalar, static_cast<int>(sizeof...(Joints))>
body_jacobian_unchecked(
    const static_chain<Scalar, Joints...>& chain,
    const fk_result<Scalar, static_cast<int>(sizeof...(Joints))>& fk)
{
    auto J_s = space_jacobian_unchecked(chain, fk);
    matrix6<Scalar> Ad_inv = fk.end_effector.inverse().adjoint();
    return Ad_inv * J_s;
}

/// Specialized body Jacobian for static_chain.
template <typename Scalar, joint_tag... Joints>
cartan::expected<jacobian_matrix<Scalar, static_cast<int>(sizeof...(Joints))>, chain_failure>
body_jacobian(
    const static_chain<Scalar, Joints...>& chain,
    const fk_result<Scalar, static_cast<int>(sizeof...(Joints))>& fk)
{
    auto shape = detail::check_fk_shape(chain, fk);
    if (!shape)
    {
        return cartan::unexpected(shape.error());
    }
    return body_jacobian_unchecked(chain, fk);
}

namespace detail
{

// kinematic_chain and static_chain each carry their own space_jacobian /
// body_jacobian overload. MSVC does not rank the concrete-type overload ahead of
// this concept-constrained one by partial ordering and reports the call ambiguous
// (C2668), so the generic overloads exclude those two types explicitly.
template <typename T>
inline constexpr bool is_jacobian_specialized_v = false;
template <typename Scalar, int N>
inline constexpr bool is_jacobian_specialized_v<kinematic_chain<Scalar, N>> = true;
template <typename Scalar, joint_tag... Joints>
inline constexpr bool is_jacobian_specialized_v<static_chain<Scalar, Joints...>> = true;

}

/// Generic space Jacobian for any chain type satisfying the chain concept,
/// under the same unchecked precondition on fk as the overloads above.
///
/// J_si(q) = Ad_{T_{i-1}}(S_i), where T_0 = I (identity).
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.11, p. 178.
template <chain Chain>
    requires (!detail::is_jacobian_specialized_v<Chain>)
jacobian_matrix<typename Chain::scalar_type, Chain::joints>
space_jacobian_unchecked(
    const Chain& chain,
    const fk_result<typename Chain::scalar_type, Chain::joints>& fk)
{
    using Scalar = typename Chain::scalar_type;
    constexpr int N = Chain::joints;
    int n = chain.num_joints();

    jacobian_matrix<Scalar, N> J;
    if constexpr (N == dynamic)
    {
        J.resize(6, n);
    }

    if (n == 0)
    {
        return J;
    }

    J.col(0) = chain.axis(0).to_vector();

    for (int i = 1; i < n; ++i)
    {
        J.col(i) = fk.intermediates[static_cast<std::size_t>(i - 1)].adjoint()
                    * chain.axis(i).to_vector();
    }

    return J;
}

/// Generic space Jacobian for any chain type satisfying the chain concept.
template <chain Chain>
    requires (!detail::is_jacobian_specialized_v<Chain>)
cartan::expected<jacobian_matrix<typename Chain::scalar_type, Chain::joints>, chain_failure>
space_jacobian(
    const Chain& chain,
    const fk_result<typename Chain::scalar_type, Chain::joints>& fk)
{
    auto shape = detail::check_fk_shape(chain, fk);
    if (!shape)
    {
        return cartan::unexpected(shape.error());
    }
    return space_jacobian_unchecked(chain, fk);
}

/// Generic body Jacobian for any chain type satisfying the chain concept,
/// under the same unchecked precondition on fk.
///
/// J_b(q) = [Ad_{T^{-1}}] * J_s(q).
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.22, p. 185.
template <chain Chain>
    requires (!detail::is_jacobian_specialized_v<Chain>)
jacobian_matrix<typename Chain::scalar_type, Chain::joints>
body_jacobian_unchecked(
    const Chain& chain,
    const fk_result<typename Chain::scalar_type, Chain::joints>& fk)
{
    auto J_s = space_jacobian_unchecked(chain, fk);
    matrix6<typename Chain::scalar_type> Ad_inv = fk.end_effector.inverse().adjoint();
    return Ad_inv * J_s;
}

/// Generic body Jacobian for any chain type satisfying the chain concept.
template <chain Chain>
    requires (!detail::is_jacobian_specialized_v<Chain>)
cartan::expected<jacobian_matrix<typename Chain::scalar_type, Chain::joints>, chain_failure>
body_jacobian(
    const Chain& chain,
    const fk_result<typename Chain::scalar_type, Chain::joints>& fk)
{
    auto shape = detail::check_fk_shape(chain, fk);
    if (!shape)
    {
        return cartan::unexpected(shape.error());
    }
    return body_jacobian_unchecked(chain, fk);
}

}

#endif
