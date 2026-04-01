#ifndef HPP_GUARD_LIEPP_SERIAL_IK_LIMITS_POLICY_H
#define HPP_GUARD_LIEPP_SERIAL_IK_LIMITS_POLICY_H

/// @file limits_policy.h
/// @brief Joint limit enforcement policies for IK solvers.
///
/// Three stateless policy structs controlling how joint limits are enforced
/// on the hot path: no enforcement, hard clamping, or null-space projection
/// toward joint midpoints for redundant chains.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.3 (null-space).
///            Decisions D-06, D-07 (IK-08).

#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/fk/jacobian.h"
#include "liepp/serial/chain/kinematic_chain.h"

#include <Eigen/SVD>

#include <concepts>
#include <algorithm>

namespace liepp
{

/// Concept detecting whether a limits policy has an extended enforce signature
/// that accepts the body Jacobian and its SVD (for null-space projection).
template <typename P, typename Scalar, int N>
concept has_extended_enforce = requires(
    typename joint_state<Scalar, N>::position_type& q,
    const typename kinematic_chain<Scalar, N>::limits_storage& limits,
    const jacobian_matrix<Scalar, N>& J_b,
    const Eigen::JacobiSVD<jacobian_matrix<Scalar, N>>& svd)
{
    { P::template enforce_extended<Scalar, N>(q, limits, J_b, svd) };
};

/// No-op limit policy: applies no enforcement.
/// Use when the stepper itself handles constraints (e.g., SQP with box bounds).
struct no_limits
{
    template <typename Scalar, int N>
    static void enforce(
        typename joint_state<Scalar, N>::position_type&,
        const typename kinematic_chain<Scalar, N>::limits_storage&)
    {
    }
};

/// Hard clamping policy: clamps each q(i) to [position_min, position_max].
/// Simple and robust, but may cause discontinuities at boundaries.
struct clamp_limits
{
    template <typename Scalar, int N>
    static void enforce(
        typename joint_state<Scalar, N>::position_type& q,
        const typename kinematic_chain<Scalar, N>::limits_storage& limits)
    {
        for (int i = 0; i < static_cast<int>(q.size()); ++i)
        {
            q(i) = std::clamp(
                q(i),
                limits[static_cast<std::size_t>(i)].position_min,
                limits[static_cast<std::size_t>(i)].position_max);
        }
    }
};

/// Null-space projection policy: pushes joints toward midpoints via
/// null-space gradient projection, then safety-clamps.
///
/// For redundant chains (DOF > 6), the null space of the Jacobian allows
/// joint motion that does not affect the end-effector. This policy uses
/// the null space to bias joints toward their range midpoints, improving
/// distance from joint limits without degrading the primary IK task.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.3, p. 235-237.
///            Liegeois, A., "Automatic Supervisory Control of Configuration
///            and Behavior of Multibody Mechanisms," 1977.
struct null_space_limits
{
    /// Simple fallback: just clamp (when Jacobian/SVD not available).
    template <typename Scalar, int N>
    static void enforce(
        typename joint_state<Scalar, N>::position_type& q,
        const typename kinematic_chain<Scalar, N>::limits_storage& limits)
    {
        clamp_limits::enforce<Scalar, N>(q, limits);
    }

    /// Extended enforce with null-space projection.
    ///
    /// Gradient: dq_null(i) = -gain * (q(i) - q_mid(i)) / (q_range(i)^2)
    /// Projection: dq_proj = V_null * V_null^T * dq_null
    /// Then safety clamp.
    template <typename Scalar, int N>
    static void enforce_extended(
        typename joint_state<Scalar, N>::position_type& q,
        const typename kinematic_chain<Scalar, N>::limits_storage& limits,
        const jacobian_matrix<Scalar, N>&,
        const Eigen::JacobiSVD<jacobian_matrix<Scalar, N>>& svd,
        Scalar gain = Scalar(0.5))
    {
        int n = static_cast<int>(q.size());
        using vec_type = typename joint_state<Scalar, N>::position_type;

        // Null-space gradient: push each joint toward midpoint
        vec_type dq_null;
        if constexpr (N == dynamic)
        {
            dq_null.resize(n);
        }

        for (int i = 0; i < n; ++i)
        {
            Scalar q_mid = (limits[static_cast<std::size_t>(i)].position_min
                          + limits[static_cast<std::size_t>(i)].position_max) / Scalar(2);
            Scalar q_range = limits[static_cast<std::size_t>(i)].position_max
                           - limits[static_cast<std::size_t>(i)].position_min;
            dq_null(i) = -gain * (q(i) - q_mid) / (q_range * q_range);
        }

        // Null-space projector from SVD: V_null * V_null^T
        auto V = svd.matrixV();
        int rank = static_cast<int>(svd.rank());

        if (rank < n)
        {
            auto V_null = V.rightCols(n - rank);
            vec_type dq_proj = V_null * (V_null.transpose() * dq_null);
            q += dq_proj;
        }

        // Safety clamp
        clamp_limits::enforce<Scalar, N>(q, limits);
    }
};

}

#endif
