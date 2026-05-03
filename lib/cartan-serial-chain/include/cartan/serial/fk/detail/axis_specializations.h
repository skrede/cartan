#ifndef HPP_GUARD_CARTAN_SERIAL_FK_DETAIL_AXIS_SPECIALIZATIONS_H
#define HPP_GUARD_CARTAN_SERIAL_FK_DETAIL_AXIS_SPECIALIZATIONS_H

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"
#include "cartan/lie/se3.h"
#include "cartan/lie/so3.h"

#include "cartan/serial/chain/joint_tags.h"
#include "cartan/serial/chain/joint_kind.h"
#include "cartan/serial/chain/screw_axis.h"

#include <cmath>
#include <concepts>

namespace cartan::detail
{

/// Single-call sin+cos returning both via reference. Avoids two libm calls.
/// Uses GCC/clang `__builtin_sincos*` intrinsics which the compiler emits
/// as a single sincos libm call (or fsincos instruction) per invocation.
inline void fk_sincos(double x, double& s, double& c) { __builtin_sincos(x, &s, &c); }
inline void fk_sincos(float x, float& s, float& c) { __builtin_sincosf(x, &s, &c); }
inline void fk_sincos(long double x, long double& s, long double& c) { __builtin_sincosl(x, &s, &c); }

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

/// Runtime dispatch for per-joint SE(3) exponential.
///
/// Used by kinematic_chain's FK loop to route into the same compile-time
/// specializations as static_chain, based on the cached joint_kind. The
/// general fallback uses the standard se3::exp path.
template <typename Scalar>
[[nodiscard]] se3<Scalar> exp_joint_runtime(
    joint_kind kind,
    Scalar q,
    const screw_axis<Scalar>& axis)
{
    switch (kind)
    {
        case joint_kind::revolute_x:  return exp_joint<revolute_x>(q, axis);
        case joint_kind::revolute_y:  return exp_joint<revolute_y>(q, axis);
        case joint_kind::revolute_z:  return exp_joint<revolute_z>(q, axis);
        case joint_kind::prismatic_x: return exp_joint<prismatic_x>(q, axis);
        case joint_kind::prismatic_y: return exp_joint<prismatic_y>(q, axis);
        case joint_kind::prismatic_z: return exp_joint<prismatic_z>(q, axis);
        case joint_kind::general:
        default:
            return se3<Scalar>::exp(axis.to_vector() * q);
    }
}

/// Per-joint SE(3) exponential returning rotation as 3x3 matrix directly,
/// bypassing the quaternion form. Matrix-form composition (R*R, t+R*t) is
/// significantly faster than quaternion product + quaternion-vec rotation
/// on AVX2-class hardware due to better SIMD/ILP. The trig and translation
/// algebra mirrors `exp_joint`; only the rotation representation changes.
template <joint_tag JointTag, typename Scalar>
inline void exp_joint_matrix(
    Scalar q,
    const screw_axis<Scalar>& axis,
    matrix3<Scalar>& R,
    vector3<Scalar>& t)
{
    if constexpr (std::same_as<JointTag, revolute_z>)
    {
        Scalar s_ax = axis.omega()(2);
        Scalar theta = s_ax * q;
        Scalar half_theta = theta / Scalar(2);
        Scalar sin_h, cos_h;
        fk_sincos(half_theta, sin_h, cos_h);
        Scalar sin_t = Scalar(2) * sin_h * cos_h;
        Scalar cos_t = Scalar(1) - Scalar(2) * sin_h * sin_h;
        R << cos_t, -sin_t, Scalar(0),
             sin_t,  cos_t, Scalar(0),
             Scalar(0), Scalar(0), Scalar(1);
        Scalar theta_sq = theta * theta;
        Scalar sinc, omcc;
        if (theta_sq < epsilon_v<Scalar>)
        {
            sinc = Scalar(1) - theta_sq / Scalar(6);
            omcc = theta / Scalar(2) - theta * theta_sq / Scalar(24);
        }
        else
        {
            Scalar sinc_h = sin_h / half_theta;
            sinc = sinc_h * cos_h;
            omcc = sinc_h * sin_h;
        }
        vector3<Scalar> rho = axis.v() * q;
        t(0) = sinc * rho(0) - omcc * rho(1);
        t(1) = omcc * rho(0) + sinc * rho(1);
        t(2) = rho(2);
    }
    else if constexpr (std::same_as<JointTag, revolute_y>)
    {
        Scalar s_ax = axis.omega()(1);
        Scalar theta = s_ax * q;
        Scalar half_theta = theta / Scalar(2);
        Scalar sin_h, cos_h;
        fk_sincos(half_theta, sin_h, cos_h);
        Scalar sin_t = Scalar(2) * sin_h * cos_h;
        Scalar cos_t = Scalar(1) - Scalar(2) * sin_h * sin_h;
        R << cos_t,  Scalar(0), sin_t,
             Scalar(0), Scalar(1), Scalar(0),
            -sin_t,  Scalar(0), cos_t;
        Scalar theta_sq = theta * theta;
        Scalar sinc, omcc;
        if (theta_sq < epsilon_v<Scalar>)
        {
            sinc = Scalar(1) - theta_sq / Scalar(6);
            omcc = theta / Scalar(2) - theta * theta_sq / Scalar(24);
        }
        else
        {
            Scalar sinc_h = sin_h / half_theta;
            sinc = sinc_h * cos_h;
            omcc = sinc_h * sin_h;
        }
        vector3<Scalar> rho = axis.v() * q;
        t(0) = sinc * rho(0) + omcc * rho(2);
        t(1) = rho(1);
        t(2) = -omcc * rho(0) + sinc * rho(2);
    }
    else if constexpr (std::same_as<JointTag, revolute_x>)
    {
        Scalar s_ax = axis.omega()(0);
        Scalar theta = s_ax * q;
        Scalar half_theta = theta / Scalar(2);
        Scalar sin_h, cos_h;
        fk_sincos(half_theta, sin_h, cos_h);
        Scalar sin_t = Scalar(2) * sin_h * cos_h;
        Scalar cos_t = Scalar(1) - Scalar(2) * sin_h * sin_h;
        R << Scalar(1), Scalar(0), Scalar(0),
             Scalar(0), cos_t, -sin_t,
             Scalar(0), sin_t,  cos_t;
        Scalar theta_sq = theta * theta;
        Scalar sinc, omcc;
        if (theta_sq < epsilon_v<Scalar>)
        {
            sinc = Scalar(1) - theta_sq / Scalar(6);
            omcc = theta / Scalar(2) - theta * theta_sq / Scalar(24);
        }
        else
        {
            Scalar sinc_h = sin_h / half_theta;
            sinc = sinc_h * cos_h;
            omcc = sinc_h * sin_h;
        }
        vector3<Scalar> rho = axis.v() * q;
        t(0) = rho(0);
        t(1) = sinc * rho(1) - omcc * rho(2);
        t(2) = omcc * rho(1) + sinc * rho(2);
    }
    else if constexpr (std::same_as<JointTag, prismatic_x>)
    {
        R.setIdentity();
        t.setZero();
        t(0) = q;
    }
    else if constexpr (std::same_as<JointTag, prismatic_y>)
    {
        R.setIdentity();
        t.setZero();
        t(1) = q;
    }
    else if constexpr (std::same_as<JointTag, prismatic_z>)
    {
        R.setIdentity();
        t.setZero();
        t(2) = q;
    }
}

/// Runtime dispatch for matrix-form per-joint SE(3) exponential.
template <typename Scalar>
inline void exp_joint_matrix_runtime(
    joint_kind kind,
    Scalar q,
    const screw_axis<Scalar>& axis,
    matrix3<Scalar>& R,
    vector3<Scalar>& t)
{
    switch (kind)
    {
        case joint_kind::revolute_x:  exp_joint_matrix<revolute_x>(q, axis, R, t); return;
        case joint_kind::revolute_y:  exp_joint_matrix<revolute_y>(q, axis, R, t); return;
        case joint_kind::revolute_z:  exp_joint_matrix<revolute_z>(q, axis, R, t); return;
        case joint_kind::prismatic_x: exp_joint_matrix<prismatic_x>(q, axis, R, t); return;
        case joint_kind::prismatic_y: exp_joint_matrix<prismatic_y>(q, axis, R, t); return;
        case joint_kind::prismatic_z: exp_joint_matrix<prismatic_z>(q, axis, R, t); return;
        case joint_kind::general:
        default:
        {
            auto se = exp_joint_runtime(kind, q, axis);
            R = se.rotation().matrix();
            t = se.translation();
            return;
        }
    }
}

/// Runtime dispatch for the first space-Jacobian column at identity.
template <typename Scalar, typename ColExpr>
void jacobian_column_identity_runtime(
    joint_kind kind,
    ColExpr&& col,
    const screw_axis<Scalar>& axis)
{
    switch (kind)
    {
        case joint_kind::revolute_x:  jacobian_column_identity<revolute_x>(col, axis); return;
        case joint_kind::revolute_y:  jacobian_column_identity<revolute_y>(col, axis); return;
        case joint_kind::revolute_z:  jacobian_column_identity<revolute_z>(col, axis); return;
        case joint_kind::prismatic_x: jacobian_column_identity<prismatic_x>(col, axis); return;
        case joint_kind::prismatic_y: jacobian_column_identity<prismatic_y>(col, axis); return;
        case joint_kind::prismatic_z: jacobian_column_identity<prismatic_z>(col, axis); return;
        case joint_kind::general:
        default:
            col = axis.to_vector();
            return;
    }
}

/// Runtime dispatch for non-first space-Jacobian columns.
/// Computes J_si = Ad_{T_{i-1}} * S_i without materializing the 6x6 adjoint
/// when the axis is principal-axis-aligned.
template <typename Scalar, typename ColExpr>
void jacobian_column_runtime(
    joint_kind kind,
    ColExpr&& col,
    const se3<Scalar>& T_prev,
    const screw_axis<Scalar>& axis)
{
    switch (kind)
    {
        case joint_kind::revolute_x:  jacobian_column<revolute_x>(col, T_prev, axis); return;
        case joint_kind::revolute_y:  jacobian_column<revolute_y>(col, T_prev, axis); return;
        case joint_kind::revolute_z:  jacobian_column<revolute_z>(col, T_prev, axis); return;
        case joint_kind::prismatic_x: jacobian_column<prismatic_x>(col, T_prev, axis); return;
        case joint_kind::prismatic_y: jacobian_column<prismatic_y>(col, T_prev, axis); return;
        case joint_kind::prismatic_z: jacobian_column<prismatic_z>(col, T_prev, axis); return;
        case joint_kind::general:
        default:
            col = T_prev.adjoint() * axis.to_vector();
            return;
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
