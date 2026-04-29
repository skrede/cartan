#ifndef HPP_GUARD_CARTAN_LIE_SE3_H
#define HPP_GUARD_CARTAN_LIE_SE3_H

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"

#include "cartan/lie/so3.h"
#include "cartan/lie/policy.h"
#include "cartan/lie/hat_vee.h"

#include <cmath>
#include <string>
#include <expected>

namespace cartan
{

/// 3D rigid body transformation group SE(3), parameterized by scalar type and policy.
/// Internal representation: SO(3) rotation + R^3 translation (D-05 pattern).
/// 7 Scalars total (4 quaternion + 3 translation).
/// Reference: Lynch & Park, Modern Robotics, Ch. 3.3, p. 86-106.
///            Barfoot, State Estimation for Robotics, Ch. 8, p. 280-300.
template <typename Scalar, typename Policy = strict_policy>
class se3
{
public:
    /// Construct from rotation and translation components.
    se3(const so3<Scalar, Policy>& rot, const vector3<Scalar>& trans)
        : m_rotation(rot)
        , m_translation(trans)
    {
    }

    /// Construct from another-policy se3 without renormalizing the rotation.
    /// Caller must ensure the source rotation is unit; debug builds validate
    /// via the so3 trusted constructor's assert.
    template <typename P2>
    se3(const se3<Scalar, P2>& other, trusted_unit_t)
        : m_rotation(other.rotation().quaternion_ref(), trusted_unit)
        , m_translation(other.translation())
    {
    }

    /// Group composition without renormalizing the result rotation.
    /// Returns fast_policy se3; caller takes responsibility for accumulated
    /// drift. Designed for FK chain accumulators where N successive unit
    /// quaternion products keep ||q||^2 - 1 bounded by O(N * eps).
    template <typename P2>
    [[nodiscard]] se3<Scalar, fast_policy>
    compose_trusted(const se3<Scalar, P2>& rhs) const
    {
        quaternion<Scalar> q = m_rotation.quaternion_ref()
                             * rhs.rotation().quaternion_ref();
        vector3<Scalar> t = m_rotation.act(rhs.translation()) + m_translation;
        return se3<Scalar, fast_policy>(
            so3<Scalar, fast_policy>(q), t);
    }

    /// Exponential map: se(3) twist -> SE(3) transform.
    /// Twist V = (omega, rho) uses omega-first convention (D-11).
    /// Ref: Lynch & Park, Modern Robotics, Prop. 3.25/Eq. 3.88, p. 103.
    ///      Barfoot, State Estimation for Robotics, Eq. 8.33, p. 289.
    [[nodiscard]] static se3 exp(const vector6<Scalar>& v)
    {
        vector3<Scalar> omega = v.template head<3>();
        vector3<Scalar> rho = v.template tail<3>();

        auto [C, J] = detail::exp_with_left_jacobian<Scalar>(omega);
        vector3<Scalar> t = J * rho;

        return se3(so3<Scalar, Policy>(C.quaternion_ref()), t);
    }

    /// Logarithmic map: SE(3) transform -> se(3) twist.
    /// Returns V = (omega, rho) in omega-first convention.
    /// Ref: Lynch & Park, Modern Robotics, Eq. 3.91-3.92, p. 104.
    ///      Barfoot, State Estimation for Robotics, Eq. 8.35, p. 290.
    [[nodiscard]] vector6<Scalar> log() const
    {
        vector3<Scalar> omega = m_rotation.log();
        matrix3<Scalar> J_inv = so3<Scalar, Policy>::left_jacobian_inv(omega);
        vector3<Scalar> rho = J_inv * m_translation;

        vector6<Scalar> result;
        result.template head<3>() = omega;
        result.template tail<3>() = rho;
        return result;
    }

    /// Group composition: T1 * T2.
    /// Result uses the stricter of the two policies (D-08).
    /// Ref: Lynch & Park, Modern Robotics, homogeneous transform composition.
    template <typename P2>
    [[nodiscard]] auto operator*(const se3<Scalar, P2>& rhs) const
        -> se3<Scalar, stricter_policy<Policy, P2>>
    {
        using RP = stricter_policy<Policy, P2>;
        auto new_rot = so3<Scalar, RP>(
            quaternion<Scalar>(
                m_rotation.quaternion_ref() * rhs.rotation().quaternion_ref()));
        vector3<Scalar> new_trans = m_rotation.act(rhs.translation()) + m_translation;
        return se3<Scalar, RP>(new_rot, new_trans);
    }

    /// Group inverse: T^{-1} = (R^{-1}, -R^{-1} * t).
    /// Ref: Lynch & Park, Modern Robotics, Eq. 3.64, p. 94.
    [[nodiscard]] se3 inverse() const
    {
        auto rot_inv = m_rotation.inverse();
        return se3(rot_inv, -(rot_inv.act(m_translation)));
    }

    /// Adjoint representation: 6x6 matrix.
    /// Omega-first twist ordering:
    ///   [Ad_T] = [R        0   ]
    ///            [[p]R     R   ]
    /// Ref: Lynch & Park, Modern Robotics, Def. 3.20, p. 98.
    [[nodiscard]] matrix6<Scalar> adjoint() const
    {
        matrix6<Scalar> Ad;
        Ad.setZero();
        matrix3<Scalar> R = m_rotation.matrix();
        Ad.template block<3, 3>(0, 0) = R;
        Ad.template block<3, 3>(3, 0) = hat(m_translation) * R;
        Ad.template block<3, 3>(3, 3) = R;
        return Ad;
    }

    /// Coadjoint representation: Ad_T^{-T} = (Ad_{T^{-1}})^T.
    /// Computed as inverse().adjoint().transpose() for correctness.
    /// Ref: Derived from general coadjoint definition Ad_T^{-T}.
    [[nodiscard]] matrix6<Scalar> coadjoint() const
    {
        return inverse().adjoint().transpose();
    }

    /// Convert to 4x4 homogeneous transformation matrix.
    /// Ref: Lynch & Park, Modern Robotics, Eq. 3.60-3.61.
    [[nodiscard]] matrix4<Scalar> matrix() const
    {
        matrix4<Scalar> T;
        T.setZero();
        T.template block<3, 3>(0, 0) = m_rotation.matrix();
        T.template block<3, 1>(0, 3) = m_translation;
        T(3, 3) = Scalar(1);
        return T;
    }

    /// Access the rotation component.
    [[nodiscard]] const so3<Scalar, Policy>& rotation() const { return m_rotation; }

    /// Access the translation component.
    [[nodiscard]] const vector3<Scalar>& translation() const { return m_translation; }

    /// Identity element: no rotation, no translation.
    [[nodiscard]] static se3 identity()
    {
        return se3(so3<Scalar, Policy>::identity(), vector3<Scalar>::Zero());
    }

    /// Construct from 4x4 homogeneous matrix with validation (D-09).
    /// Validates rotation block is SO(3) and bottom row is (0,0,0,1).
    /// Ref: SE(3) matrix structure, Lynch & Park, Modern Robotics, p. 86.
    [[nodiscard]] static std::expected<se3, std::string>
    from_matrix(const matrix4<Scalar>& T)
    {
        Scalar tol = detail::sqrt_epsilon_v<Scalar>;

        // Check bottom row is [0, 0, 0, 1]
        if (std::abs(T(3, 0)) > tol || std::abs(T(3, 1)) > tol ||
            std::abs(T(3, 2)) > tol || std::abs(T(3, 3) - Scalar(1)) > tol)
        {
            return std::unexpected("Bottom row is not [0, 0, 0, 1]");
        }

        // Validate rotation block
        matrix3<Scalar> R = T.template block<3, 3>(0, 0);
        auto rot_result = so3<Scalar, Policy>::from_matrix(R);
        if (!rot_result.has_value())
        {
            return std::unexpected("Rotation block invalid: " + rot_result.error());
        }

        vector3<Scalar> trans = T.template block<3, 1>(0, 3);
        return se3(rot_result.value(), trans);
    }

    /// Transform a 3D point: R * p + t.
    /// Ref: SE(3) action on R^3.
    [[nodiscard]] vector3<Scalar> act(const vector3<Scalar>& p) const
    {
        return m_rotation.act(p) + m_translation;
    }

private:
    so3<Scalar, Policy> m_rotation;
    vector3<Scalar> m_translation;
};

}

#endif
