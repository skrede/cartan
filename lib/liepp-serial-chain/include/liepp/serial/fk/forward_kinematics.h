#ifndef HPP_GUARD_LIEPP_SERIAL_FK_FORWARD_KINEMATICS_H
#define HPP_GUARD_LIEPP_SERIAL_FK_FORWARD_KINEMATICS_H

/// @file forward_kinematics.h
/// @brief Forward kinematics via Product of Exponentials (PoE).
///
/// Computes T(q) = exp([S1]q1) * ... * exp([Sn]qn) * M with intermediate
/// product caching for Jacobian reuse. Fixed-size chains (N=1-7) use
/// compile-time unrolled fold expressions; dynamic/large chains use a
/// runtime loop.
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138:
///   T(q) = exp([S1]q1) * exp([S2]q2) * ... * exp([Sn]qn) * M
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 4, p. 119-158.

#include "liepp/serial/fk/fk_result.h"
#include "liepp/serial/fk/detail/axis_specializations.h"

#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/chain/chain_concept.h"
#include "liepp/serial/chain/static_chain.h"
#include "liepp/serial/chain/kinematic_chain.h"

#include <tuple>
#include <utility>

namespace liepp
{

namespace detail
{

/// Runtime-loop PoE accumulation for dynamic or large (N >= 8) chains.
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138.
template <typename Scalar, int N>
fk_result<Scalar, N> fk_loop(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q)
{
    fk_result<Scalar, N> result;
    const auto& axes = chain.axes();
    int n = chain.num_joints();

    auto accum = se3<Scalar>::identity();

    if constexpr (N == dynamic)
    {
        result.intermediates.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i)
        {
            accum = accum * se3<Scalar>::exp(axes[static_cast<std::size_t>(i)].to_vector() * q(i));
            result.intermediates.push_back(accum);
        }
    }
    else
    {
        for (int i = 0; i < n; ++i)
        {
            accum = accum * se3<Scalar>::exp(axes[static_cast<std::size_t>(i)].to_vector() * q(i));
            result.intermediates[static_cast<std::size_t>(i)] = accum;
        }
    }

    result.end_effector = accum * chain.home();
    return result;
}

/// Compile-time unrolled PoE accumulation for fixed-size chains (N=1-7).
/// Uses index_sequence fold expression for zero-overhead expansion.
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138.
template <typename Scalar, int N, std::size_t... Is>
fk_result<Scalar, N> fk_unrolled_impl(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q,
    std::index_sequence<Is...>)
{
    fk_result<Scalar, N> result;
    const auto& axes = chain.axes();

    auto accum = se3<Scalar>::identity();

    ((accum = accum * se3<Scalar>::exp(axes[Is].to_vector() * q(static_cast<int>(Is))),
      result.intermediates[Is] = accum), ...);

    result.end_effector = accum * chain.home();
    return result;
}

/// Entry point for compile-time unrolled PoE.
template <typename Scalar, int N>
fk_result<Scalar, N> fk_unrolled(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q)
{
    return fk_unrolled_impl(chain, q, std::make_index_sequence<static_cast<std::size_t>(N)>{});
}

} // namespace detail

/// Compute forward kinematics for a kinematic chain at joint configuration q.
///
/// Returns the end-effector SE(3) pose and all intermediate products
/// for Jacobian computation reuse.
///
/// For fixed-size chains with N=1-7 joints, dispatches to a compile-time
/// unrolled fold expression. For dynamic or larger chains, uses a runtime loop.
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138:
///   T(q) = exp([S1]q1) * ... * exp([Sn]qn) * M
///
/// @tparam N      Number of joints (compile-time), or liepp::dynamic.
/// @tparam Scalar Floating-point type.
/// @param chain   Kinematic chain with screw axes and home configuration.
/// @param q       Joint position vector.
/// @return        fk_result containing end_effector pose and intermediates.
template <typename Scalar, int N>
fk_result<Scalar, N> forward_kinematics(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q)
{
    if constexpr (N >= 1 && N <= 7)
    {
        return detail::fk_unrolled(chain, q);
    }
    else
    {
        return detail::fk_loop(chain, q);
    }
}

/// Specialized forward kinematics for static_chain exploiting compile-time
/// joint tag knowledge. Each joint's SE3 exponential uses axis-specific
/// quaternion construction and sparse left Jacobian entries instead of the
/// generic Rodrigues exponential map.
///
/// Wins over the generic chain overload via partial ordering on
/// static_chain<Scalar, Joints...>.
template <typename Scalar, joint_tag... Joints>
[[nodiscard]] fk_result<Scalar, static_cast<int>(sizeof...(Joints))>
forward_kinematics(
    const static_chain<Scalar, Joints...>& chain,
    const typename joint_state<Scalar, static_cast<int>(sizeof...(Joints))>::position_type& q)
{
    constexpr int N = static_cast<int>(sizeof...(Joints));
    fk_result<Scalar, N> result;
    auto accum = se3<Scalar>::identity();

    [&]<std::size_t... Is>(std::index_sequence<Is...>)
    {
        using joint_tuple = std::tuple<Joints...>;
        ((
            [&]<std::size_t I>()
            {
                using Joint = std::tuple_element_t<I, joint_tuple>;
                accum = accum * detail::exp_joint<Joint>(
                    q(static_cast<int>(I)), chain.axis(static_cast<int>(I)));
                result.intermediates[I] = accum;
            }.template operator()<Is>()
        ), ...);
    }(std::make_index_sequence<static_cast<std::size_t>(N)>{});

    result.end_effector = accum * chain.home();
    return result;
}

/// Generic forward kinematics for any chain type satisfying the chain concept.
///
/// Uses per-element axis(i) access and a runtime loop. The existing
/// kinematic_chain overload is more constrained and wins for kinematic_chain
/// arguments via partial ordering; this overload handles static_chain and
/// any future chain types.
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138.
template <chain Chain>
[[nodiscard]] fk_result<typename Chain::scalar_type, Chain::joints>
forward_kinematics(
    const Chain& chain,
    const typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type& q)
{
    using Scalar = typename Chain::scalar_type;
    constexpr int N = Chain::joints;

    fk_result<Scalar, N> result;
    auto accum = se3<Scalar>::identity();
    int n = chain.num_joints();

    for (int i = 0; i < n; ++i)
    {
        accum = accum * se3<Scalar>::exp(chain.axis(i).to_vector() * q(i));
        result.intermediates[static_cast<std::size_t>(i)] = accum;
    }

    result.end_effector = accum * chain.home();
    return result;
}

}

#endif
