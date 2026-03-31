#ifndef HPP_GUARD_LIEPP_LIE_SO3_H
#define HPP_GUARD_LIEPP_LIE_SO3_H

#include "liepp/types.h"
#include "liepp/detail/epsilon.h"

#include "liepp/lie/policy.h"
#include "liepp/lie/hat_vee.h"

#include <cmath>
#include <string>
#include <cassert>
#include <expected>

namespace liepp
{

namespace detail
{

/// SO(3) logarithmic map implementation via quaternion atan2 approach.
/// Avoids theta~pi eigenvector branch entirely; only theta~0 needs Taylor.
/// Ref: Barfoot, State Estimation for Robotics, Eq. 8.22/8.28, p. 284-286.
template <typename Scalar>
[[nodiscard]] vector3<Scalar> so3_log_impl(quaternion<Scalar> q)
{
    // Canonicalize to w >= 0 hemisphere (double-cover)
    if (q.w() < Scalar(0))
    {
        q.coeffs() = -q.coeffs();
    }

    Scalar n = q.vec().norm();

    Scalar k;
    if (n < sqrt_epsilon_v<Scalar>)
    {
        // Taylor branch: 2*atan2(n,w)/n ~ 2/w - 2*n^2/(3*w^3)
        Scalar w = q.w();
        k = Scalar(2) / w - Scalar(2) * n * n / (Scalar(3) * w * w * w);
    }
    else
    {
        k = Scalar(2) * std::atan2(n, q.w()) / n;
    }

    return k * q.vec();
}

/// SO(3) left Jacobian implementation.
/// Ref: Barfoot, State Estimation for Robotics, Eq. 8.82b, p. 298.
template <typename Scalar>
[[nodiscard]] matrix3<Scalar> so3_left_jacobian_impl(const vector3<Scalar>& phi)
{
    Scalar phi_sq = phi.squaredNorm();
    matrix3<Scalar> I = matrix3<Scalar>::Identity();

    if (phi_sq < sqrt_epsilon_v<Scalar>)
    {
        // Taylor: J_l ~ I + hat(phi)/2 + hat(phi)^2/6
        matrix3<Scalar> phi_hat = hat(phi);
        return I + Scalar(0.5) * phi_hat + phi_hat * phi_hat / Scalar(6);
    }

    Scalar phi_norm = std::sqrt(phi_sq);
    vector3<Scalar> a = phi / phi_norm;
    matrix3<Scalar> a_hat = hat(a);

    Scalar s = std::sin(phi_norm);
    Scalar c = std::cos(phi_norm);

    // J_l = sin(phi)/phi * I + (1 - sin(phi)/phi) * a*a^T + (1-cos(phi))/phi * hat(a)
    return (s / phi_norm) * I
         + (Scalar(1) - s / phi_norm) * (a * a.transpose())
         + ((Scalar(1) - c) / phi_norm) * a_hat;
}

/// SO(3) inverse left Jacobian implementation.
/// Ref: Barfoot, State Estimation for Robotics, Eq. 8.84, p. 299.
template <typename Scalar>
[[nodiscard]] matrix3<Scalar> so3_left_jacobian_inv_impl(const vector3<Scalar>& phi)
{
    Scalar phi_sq = phi.squaredNorm();
    matrix3<Scalar> I = matrix3<Scalar>::Identity();

    if (phi_sq < sqrt_epsilon_v<Scalar>)
    {
        // Taylor: J_l^{-1} ~ I - hat(phi)/2 + hat(phi)^2/12
        matrix3<Scalar> phi_hat = hat(phi);
        return I - Scalar(0.5) * phi_hat + phi_hat * phi_hat / Scalar(12);
    }

    Scalar phi_norm = std::sqrt(phi_sq);
    Scalar half_phi = phi_norm / Scalar(2);
    vector3<Scalar> a = phi / phi_norm;
    matrix3<Scalar> a_hat = hat(a);

    // J_l^{-1} = phi/2 * cot(phi/2) * I + (1 - phi/2*cot(phi/2)) * a*a^T - phi/2 * hat(a)
    Scalar cot_half = std::cos(half_phi) / std::sin(half_phi);
    Scalar half_phi_cot = half_phi * cot_half;

    return half_phi_cot * I
         + (Scalar(1) - half_phi_cot) * (a * a.transpose())
         - half_phi * a_hat;
}

} // namespace detail

/// 3D rotation group SO(3), parameterized by scalar type and policy.
/// Internal representation: unit quaternion (Eigen::Quaternion<Scalar>).
/// Reference: Lynch & Park, Modern Robotics, Ch. 3.2, p. 68-82.
///            Barfoot, State Estimation for Robotics, Ch. 8, p. 280-300.
template <typename Scalar, typename Policy = strict_policy>
class so3
{
public:
    /// Construct from quaternion. Strict policy normalizes to unit length.
    /// Reference: Unit quaternion represents rotation; ||q|| = 1.
    explicit so3(const quaternion<Scalar>& q)
        : m_quaternion(q)
    {
        if constexpr (Policy::normalize_on_construct)
        {
            if constexpr (Policy::assert_valid)
            {
                assert(m_quaternion.squaredNorm() > detail::epsilon_v<Scalar>);
            }
            m_quaternion.normalize();
        }
    }

    /// Exponential map: so(3) -> SO(3) via quaternion form.
    /// phi is the axis-angle vector (axis * angle).
    /// Ref: Barfoot, State Estimation for Robotics, Eq. 8.23, p. 285.
    ///      Lynch & Park, Modern Robotics, Prop. 3.11/Eq. 3.51, p. 82.
    [[nodiscard]] static so3 exp(const vector3<Scalar>& phi)
    {
        Scalar theta_sq = phi.squaredNorm();
        quaternion<Scalar> q;

        if (theta_sq < detail::epsilon_v<Scalar>)
        {
            // Taylor: sin(theta/2)/theta ~ 0.5 - theta_sq/48
            //         cos(theta/2) ~ 1 - theta_sq/8
            Scalar sinc_half = Scalar(0.5) - theta_sq / Scalar(48);
            q.w() = Scalar(1) - theta_sq / Scalar(8);
            q.vec() = sinc_half * phi;
        }
        else
        {
            Scalar theta = std::sqrt(theta_sq);
            Scalar half_theta = theta / Scalar(2);
            q.w() = std::cos(half_theta);
            q.vec() = (std::sin(half_theta) / theta) * phi;
        }

        q.normalize();
        return so3(q);
    }

    /// Logarithmic map: SO(3) -> so(3) via quaternion atan2 approach.
    /// Avoids theta~pi eigenvector branch; only theta~0 needs Taylor.
    /// Ref: Barfoot, State Estimation for Robotics, Eq. 8.22/8.28, p. 284-286.
    [[nodiscard]] vector3<Scalar> log() const
    {
        return detail::so3_log_impl(m_quaternion);
    }

    /// Group composition via Hamilton quaternion product.
    /// Result uses stricter of two policies (D-08).
    /// Ref: Lynch & Park, Modern Robotics, rotation composition, p. 70.
    template <typename P2>
    [[nodiscard]] auto operator*(const so3<Scalar, P2>& rhs) const
        -> so3<Scalar, stricter_policy<Policy, P2>>
    {
        return so3<Scalar, stricter_policy<Policy, P2>>(
            quaternion<Scalar>(m_quaternion * rhs.quaternion_ref()));
    }

    /// Group inverse via quaternion conjugate: q^{-1} = q*.
    /// Ref: Lynch & Park, Modern Robotics, p. 70.
    [[nodiscard]] so3 inverse() const
    {
        return so3(m_quaternion.conjugate());
    }

    /// Adjoint representation: Ad_R = R (the rotation matrix itself for SO(3)).
    /// Ref: Lynch & Park, Modern Robotics, p. 75.
    [[nodiscard]] matrix3<Scalar> adjoint() const
    {
        return m_quaternion.toRotationMatrix();
    }

    /// Coadjoint representation: Ad_R^{-T} = R for SO(3) (since R is orthogonal).
    /// Ref: Derived from general coadjoint definition; SO(3) is compact.
    [[nodiscard]] matrix3<Scalar> coadjoint() const
    {
        return m_quaternion.toRotationMatrix();
    }

    /// Left Jacobian of SO(3).
    /// Ref: Barfoot, State Estimation for Robotics, Eq. 8.82b, p. 298.
    [[nodiscard]] static matrix3<Scalar> left_jacobian(const vector3<Scalar>& phi)
    {
        return detail::so3_left_jacobian_impl(phi);
    }

    /// Right Jacobian: J_r(phi) = J_l(-phi).
    /// Ref: Barfoot, State Estimation for Robotics, Eq. 8.82a, p. 298.
    [[nodiscard]] static matrix3<Scalar> right_jacobian(const vector3<Scalar>& phi)
    {
        return detail::so3_left_jacobian_impl(vector3<Scalar>(-phi));
    }

    /// Inverse left Jacobian.
    /// Ref: Barfoot, State Estimation for Robotics, Eq. 8.84, p. 299.
    [[nodiscard]] static matrix3<Scalar> left_jacobian_inv(const vector3<Scalar>& phi)
    {
        return detail::so3_left_jacobian_inv_impl(phi);
    }

    /// Inverse right Jacobian: J_r^{-1}(phi) = J_l^{-1}(-phi).
    /// Ref: Barfoot, State Estimation for Robotics, Eq. 8.84.
    [[nodiscard]] static matrix3<Scalar> right_jacobian_inv(const vector3<Scalar>& phi)
    {
        return detail::so3_left_jacobian_inv_impl(vector3<Scalar>(-phi));
    }

    /// Convert to 3x3 rotation matrix.
    /// Ref: Lynch & Park, Modern Robotics, Eq. 3.10 (rotation matrix from quaternion).
    [[nodiscard]] matrix3<Scalar> matrix() const
    {
        return m_quaternion.toRotationMatrix();
    }

    /// Access the internal quaternion (read-only).
    /// Named quaternion_ref() to avoid collision with Eigen type alias.
    [[nodiscard]] const quaternion<Scalar>& quaternion_ref() const
    {
        return m_quaternion;
    }

    /// Identity element: zero rotation (unit quaternion w=1).
    [[nodiscard]] static so3 identity()
    {
        return so3(quaternion<Scalar>(Scalar(1), Scalar(0), Scalar(0), Scalar(0)));
    }

    /// Construct from 3x3 rotation matrix with validation (D-09).
    /// Checks R^T*R ~= I and det(R) ~= 1.
    /// Ref: Rotation matrix properties, Lynch & Park, Modern Robotics, p. 23-24.
    [[nodiscard]] static std::expected<so3, std::string>
    from_matrix(const matrix3<Scalar>& R)
    {
        Scalar tol = detail::sqrt_epsilon_v<Scalar>;

        matrix3<Scalar> RtR = R.transpose() * R;
        if ((RtR - matrix3<Scalar>::Identity()).norm() > tol)
        {
            return std::unexpected("Matrix is not orthogonal: R^T * R deviates from identity");
        }

        if (std::abs(R.determinant() - Scalar(1)) > tol)
        {
            return std::unexpected("Matrix has determinant != 1: not a proper rotation");
        }

        quaternion<Scalar> q(R);
        return so3(q);
    }

    /// Construct from quaternion with validation (D-09).
    /// Checks ||q|| ~= 1.
    [[nodiscard]] static std::expected<so3, std::string>
    from_quaternion(const quaternion<Scalar>& q)
    {
        if (std::abs(q.squaredNorm() - Scalar(1)) > detail::sqrt_epsilon_v<Scalar>)
        {
            return std::unexpected("Quaternion is not unit: ||q||^2 deviates from 1");
        }

        return so3(q);
    }

    /// Rotate a 3D vector: R * v.
    /// Ref: SO(3) action on R^3 via quaternion rotation.
    [[nodiscard]] vector3<Scalar> act(const vector3<Scalar>& v) const
    {
        return m_quaternion * v;
    }

private:
    quaternion<Scalar> m_quaternion;
};

} // namespace liepp

#endif
