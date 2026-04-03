#ifndef HPP_GUARD_CARTAN_SERIAL_FK_DETAIL_AXIS_SPECIALIZATIONS_H
#define HPP_GUARD_CARTAN_SERIAL_FK_DETAIL_AXIS_SPECIALIZATIONS_H

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"
#include "cartan/lie/se3.h"
#include "cartan/lie/so3.h"

#include "cartan/serial/chain/joint_tags.h"
#include "cartan/serial/chain/screw_axis.h"

#include <cmath>
#include <concepts>

namespace cartan::detail
{

/// Per-joint SE3 exponential exploiting compile-time axis knowledge.
///
/// For revolute joints, builds the axis-aligned quaternion directly
/// (2 trig calls) and computes translation via sparse left Jacobian
/// entries. For prismatic joints, returns identity rotation with
/// axis-aligned translation. Avoids generic Rodrigues exponential.
template <joint_tag JointTag, typename Scalar>
[[nodiscard]] se3<Scalar> exp_joint(Scalar q, const screw_axis<Scalar>& axis)
{
    if constexpr (std::same_as<JointTag, revolute_z>)
    {
        Scalar s = axis.omega()(2);
        Scalar theta = s * q;
        Scalar half_theta = theta / Scalar(2);

        auto rot = so3<Scalar>(quaternion<Scalar>(
            std::cos(half_theta), Scalar(0), Scalar(0), std::sin(half_theta)));

        Scalar theta_sq = theta * theta;
        Scalar sinc, omcc;
        if (theta_sq < epsilon_v<Scalar>)
        {
            sinc = Scalar(1) - theta_sq / Scalar(6);
            omcc = theta / Scalar(2) - theta * theta_sq / Scalar(24);
        }
        else
        {
            sinc = std::sin(theta) / theta;
            omcc = (Scalar(1) - std::cos(theta)) / theta;
        }

        vector3<Scalar> rho = axis.v() * q;
        vector3<Scalar> t;
        t(0) = sinc * rho(0) - omcc * rho(1);
        t(1) = omcc * rho(0) + sinc * rho(1);
        t(2) = rho(2);

        return se3<Scalar>(rot, t);
    }
    else if constexpr (std::same_as<JointTag, revolute_y>)
    {
        Scalar s = axis.omega()(1);
        Scalar theta = s * q;
        Scalar half_theta = theta / Scalar(2);

        auto rot = so3<Scalar>(quaternion<Scalar>(
            std::cos(half_theta), Scalar(0), std::sin(half_theta), Scalar(0)));

        Scalar theta_sq = theta * theta;
        Scalar sinc, omcc;
        if (theta_sq < epsilon_v<Scalar>)
        {
            sinc = Scalar(1) - theta_sq / Scalar(6);
            omcc = theta / Scalar(2) - theta * theta_sq / Scalar(24);
        }
        else
        {
            sinc = std::sin(theta) / theta;
            omcc = (Scalar(1) - std::cos(theta)) / theta;
        }

        vector3<Scalar> rho = axis.v() * q;
        vector3<Scalar> t;
        t(0) = sinc * rho(0) + omcc * rho(2);
        t(1) = rho(1);
        t(2) = -omcc * rho(0) + sinc * rho(2);

        return se3<Scalar>(rot, t);
    }
    else if constexpr (std::same_as<JointTag, revolute_x>)
    {
        Scalar s = axis.omega()(0);
        Scalar theta = s * q;
        Scalar half_theta = theta / Scalar(2);

        auto rot = so3<Scalar>(quaternion<Scalar>(
            std::cos(half_theta), std::sin(half_theta), Scalar(0), Scalar(0)));

        Scalar theta_sq = theta * theta;
        Scalar sinc, omcc;
        if (theta_sq < epsilon_v<Scalar>)
        {
            sinc = Scalar(1) - theta_sq / Scalar(6);
            omcc = theta / Scalar(2) - theta * theta_sq / Scalar(24);
        }
        else
        {
            sinc = std::sin(theta) / theta;
            omcc = (Scalar(1) - std::cos(theta)) / theta;
        }

        vector3<Scalar> rho = axis.v() * q;
        vector3<Scalar> t;
        t(0) = rho(0);
        t(1) = sinc * rho(1) - omcc * rho(2);
        t(2) = omcc * rho(1) + sinc * rho(2);

        return se3<Scalar>(rot, t);
    }
    else if constexpr (std::same_as<JointTag, prismatic_x>)
    {
        vector3<Scalar> t = vector3<Scalar>::Zero();
        t(0) = q;
        return se3<Scalar>(so3<Scalar>::identity(), t);
    }
    else if constexpr (std::same_as<JointTag, prismatic_y>)
    {
        vector3<Scalar> t = vector3<Scalar>::Zero();
        t(1) = q;
        return se3<Scalar>(so3<Scalar>::identity(), t);
    }
    else if constexpr (std::same_as<JointTag, prismatic_z>)
    {
        vector3<Scalar> t = vector3<Scalar>::Zero();
        t(2) = q;
        return se3<Scalar>(so3<Scalar>::identity(), t);
    }
}

/// Jacobian column at identity transform (i.e., for joint index 0).
///
/// For the first joint, Ad_{I} * S = S, so the column is just the
/// screw axis 6-vector. Avoids constructing the identity rotation matrix.
template <joint_tag JointTag, typename Scalar, typename ColExpr>
void jacobian_column_identity(
    ColExpr&& col,
    const screw_axis<Scalar>& axis)
{
    if constexpr (JointTag::is_revolute)
    {
        col.template head<3>() = axis.omega();
        col.template tail<3>() = axis.v();
    }
    else
    {
        col.template head<3>().setZero();
        if constexpr (std::same_as<JointTag, prismatic_x>)
            col.template tail<3>() = vector3<Scalar>(Scalar(1), Scalar(0), Scalar(0));
        else if constexpr (std::same_as<JointTag, prismatic_y>)
            col.template tail<3>() = vector3<Scalar>(Scalar(0), Scalar(1), Scalar(0));
        else
            col.template tail<3>() = vector3<Scalar>(Scalar(0), Scalar(0), Scalar(1));
    }
}

/// Per-joint adjoint-screw column exploiting compile-time axis knowledge.
///
/// Computes J_si = Ad_{T_{i-1}} * S_i without forming the full 6x6 adjoint.
/// For revolute joints, R * omega is a column extraction from R, and
/// R * v exploits the known sparsity pattern of v per axis.
/// For prismatic joints, the angular part is zero and linear part is a
/// column extraction from R.
template <joint_tag JointTag, typename Scalar, typename ColExpr>
void jacobian_column(
    ColExpr&& col,
    const se3<Scalar>& T_prev,
    const screw_axis<Scalar>& axis)
{
    if constexpr (std::same_as<JointTag, revolute_z>)
    {
        Scalar s = axis.omega()(2);
        matrix3<Scalar> R = T_prev.rotation().matrix();
        const auto& p = T_prev.translation();

        vector3<Scalar> R_omega = s * R.col(2);
        col.template head<3>() = R_omega;

        const auto& v = axis.v();
        vector3<Scalar> R_v = v(0) * R.col(0) + v(1) * R.col(1) + v(2) * R.col(2);
        col.template tail<3>() = p.cross(R_omega) + R_v;
    }
    else if constexpr (std::same_as<JointTag, revolute_y>)
    {
        Scalar s = axis.omega()(1);
        matrix3<Scalar> R = T_prev.rotation().matrix();
        const auto& p = T_prev.translation();

        vector3<Scalar> R_omega = s * R.col(1);
        col.template head<3>() = R_omega;

        const auto& v = axis.v();
        vector3<Scalar> R_v = v(0) * R.col(0) + v(1) * R.col(1) + v(2) * R.col(2);
        col.template tail<3>() = p.cross(R_omega) + R_v;
    }
    else if constexpr (std::same_as<JointTag, revolute_x>)
    {
        Scalar s = axis.omega()(0);
        matrix3<Scalar> R = T_prev.rotation().matrix();
        const auto& p = T_prev.translation();

        vector3<Scalar> R_omega = s * R.col(0);
        col.template head<3>() = R_omega;

        const auto& v = axis.v();
        vector3<Scalar> R_v = v(0) * R.col(0) + v(1) * R.col(1) + v(2) * R.col(2);
        col.template tail<3>() = p.cross(R_omega) + R_v;
    }
    else if constexpr (std::same_as<JointTag, prismatic_x>)
    {
        matrix3<Scalar> R = T_prev.rotation().matrix();
        col.template head<3>().setZero();
        col.template tail<3>() = R.col(0);
    }
    else if constexpr (std::same_as<JointTag, prismatic_y>)
    {
        matrix3<Scalar> R = T_prev.rotation().matrix();
        col.template head<3>().setZero();
        col.template tail<3>() = R.col(1);
    }
    else if constexpr (std::same_as<JointTag, prismatic_z>)
    {
        matrix3<Scalar> R = T_prev.rotation().matrix();
        col.template head<3>().setZero();
        col.template tail<3>() = R.col(2);
    }
}

/// Thin wrapper that satisfies the chain concept but prevents the
/// static_chain overloads from being selected. Used to force the
/// generic chain-concept FK/Jacobian path for benchmarking.
template <typename ChainType>
struct generic_chain_wrapper
{
    const ChainType& wrapped;

    using scalar_type = typename ChainType::scalar_type;
    static constexpr int joints = ChainType::joints;

    [[nodiscard]] const auto& home() const { return wrapped.home(); }
    [[nodiscard]] int num_joints() const { return wrapped.num_joints(); }
    [[nodiscard]] const auto& axis(int i) const { return wrapped.axis(i); }
    [[nodiscard]] const auto& axes() const { return wrapped.axes(); }
    [[nodiscard]] const auto& limits() const { return wrapped.limits(); }
};

}

#endif
