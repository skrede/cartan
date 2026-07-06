#ifndef HPP_GUARD_CARTAN_SERIAL_FK_JACOBIAN_MATRIX_H
#define HPP_GUARD_CARTAN_SERIAL_FK_JACOBIAN_MATRIX_H

/// Space Jacobian computed from matrix-form FK intermediates.
///
/// `space_jacobian(chain, fk_matrix_result)` consumes Matrix3-form
/// intermediates and skips the per-column quaternion->matrix conversion
/// that the quaternion-form Jacobian pays. Empirically 3-4x faster on
/// 6/7-DOF chains than `space_jacobian(chain, fk_result)`.
///
/// Lynch & Park, Modern Robotics, Eq. 5.11, p. 178.

#include "cartan/types.h"

#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics_matrix.h"
#include "cartan/serial/fk/detail/axis_specializations.h"

#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/chain/static_chain.h"
#include "cartan/serial/chain/kinematic_chain.h"

namespace cartan
{

namespace detail
{

/// Compile-time tag-dispatched space-Jacobian column from a matrix-form
/// intermediate. Mirrors `jacobian_column_matrix(joint_kind, ...)` but
/// drops the runtime switch when the joint type is known statically.
template <joint_tag JointTag, typename Scalar, typename ColExpr>
inline void jacobian_column_matrix(
    ColExpr&& col,
    const pose_matrix<Scalar>& T_prev,
    const screw_axis<Scalar>& axis)
{
    const auto& R = T_prev.R;
    const auto& p = T_prev.p;

    if constexpr (std::same_as<JointTag, revolute_x>)
    {
        Scalar a = axis.omega()(0);
        vector3<Scalar> R_omega = a * R.col(0);
        vector3<Scalar> R_v;
        R_v.noalias() = R * axis.v();
        col.template head<3>() = R_omega;
        col.template tail<3>() = p.cross(R_omega) + R_v;
    }
    else if constexpr (std::same_as<JointTag, revolute_y>)
    {
        Scalar a = axis.omega()(1);
        vector3<Scalar> R_omega = a * R.col(1);
        vector3<Scalar> R_v;
        R_v.noalias() = R * axis.v();
        col.template head<3>() = R_omega;
        col.template tail<3>() = p.cross(R_omega) + R_v;
    }
    else if constexpr (std::same_as<JointTag, revolute_z>)
    {
        Scalar a = axis.omega()(2);
        vector3<Scalar> R_omega = a * R.col(2);
        vector3<Scalar> R_v;
        R_v.noalias() = R * axis.v();
        col.template head<3>() = R_omega;
        col.template tail<3>() = p.cross(R_omega) + R_v;
    }
    else if constexpr (std::same_as<JointTag, prismatic_x>
                       || std::same_as<JointTag, prismatic_y>
                       || std::same_as<JointTag, prismatic_z>)
    {
        col.template head<3>().setZero();
        col.template tail<3>() = R * axis.v();
    }
}

/// Compute one space-Jacobian column from a matrix-form intermediate.
/// J_si = Ad_{T_{i-1}} * S_i without forming the full 6x6 adjoint.
template <typename Scalar, typename ColExpr>
inline void jacobian_column_matrix(
    joint_kind kind,
    ColExpr&& col,
    const pose_matrix<Scalar>& T_prev,
    const screw_axis<Scalar>& axis)
{
    const auto& R = T_prev.R;
    const auto& p = T_prev.p;

    switch (kind)
    {
        case joint_kind::revolute_x:
        {
            Scalar a = axis.omega()(0);
            vector3<Scalar> R_omega = a * R.col(0);
            vector3<Scalar> R_v;
            R_v.noalias() = R * axis.v();
            col.template head<3>() = R_omega;
            col.template tail<3>() = p.cross(R_omega) + R_v;
            return;
        }
        case joint_kind::revolute_y:
        {
            Scalar a = axis.omega()(1);
            vector3<Scalar> R_omega = a * R.col(1);
            vector3<Scalar> R_v;
            R_v.noalias() = R * axis.v();
            col.template head<3>() = R_omega;
            col.template tail<3>() = p.cross(R_omega) + R_v;
            return;
        }
        case joint_kind::revolute_z:
        {
            Scalar a = axis.omega()(2);
            vector3<Scalar> R_omega = a * R.col(2);
            vector3<Scalar> R_v;
            R_v.noalias() = R * axis.v();
            col.template head<3>() = R_omega;
            col.template tail<3>() = p.cross(R_omega) + R_v;
            return;
        }
        case joint_kind::prismatic_x:
        case joint_kind::prismatic_y:
        case joint_kind::prismatic_z:
            col.template head<3>().setZero();
            col.template tail<3>() = R * axis.v();
            return;
        case joint_kind::general:
        default:
        {
            // Generic Ad * S
            vector3<Scalar> R_omega;
            R_omega.noalias() = R * axis.omega();
            vector3<Scalar> R_v;
            R_v.noalias() = R * axis.v();
            col.template head<3>() = R_omega;
            col.template tail<3>() = p.cross(R_omega) + R_v;
            return;
        }
    }
}

}

/// Space Jacobian from matrix-form FK result.
template <typename Scalar, int N>
jacobian_matrix<Scalar, N> space_jacobian(
    const kinematic_chain<Scalar, N>& chain,
    const fk_matrix_result<Scalar, N>& fk)
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

    detail::jacobian_column_identity_runtime(chain.kind(0), J.col(0), axes[0]);

    for (int i = 1; i < n; ++i)
    {
        detail::jacobian_column_matrix(
            chain.kind(i),
            J.col(i),
            fk.intermediates[static_cast<std::size_t>(i - 1)],
            axes[static_cast<std::size_t>(i)]);
    }

    return J;
}

/// Space Jacobian from matrix-form FK result for a static_chain.
/// Compile-time joint tags allow per-tag specialization to drop the
/// runtime switch in the hot loop.
template <typename Scalar, joint_tag... Joints>
jacobian_matrix<Scalar, static_cast<int>(sizeof...(Joints))>
space_jacobian(
    const static_chain<Scalar, Joints...>& chain,
    const fk_matrix_result<Scalar, static_cast<int>(sizeof...(Joints))>& fk)
{
    constexpr int N = static_cast<int>(sizeof...(Joints));
    const auto& axes = chain.axes();
    jacobian_matrix<Scalar, N> J;

    [&]<std::size_t... Is>(std::index_sequence<Is...>)
    {
        using joint_tuple = std::tuple<Joints...>;
        ((
            [&]<std::size_t I>()
            {
                if constexpr (I == 0)
                {
                    using Joint = std::tuple_element_t<0, joint_tuple>;
                    detail::jacobian_column_identity<Joint>(J.col(0), axes[0]);
                }
                else
                {
                    using Joint = std::tuple_element_t<I, joint_tuple>;
                    detail::jacobian_column_matrix<Joint>(
                        J.col(static_cast<int>(I)),
                        fk.intermediates[I - 1],
                        axes[I]);
                }
            }.template operator()<Is>()
        ), ...);
    }(std::make_index_sequence<static_cast<std::size_t>(N)>{});

    return J;
}

}

#endif
