#ifndef HPP_GUARD_CARTAN_LIE_SE2_H
#define HPP_GUARD_CARTAN_LIE_SE2_H

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"

#include "cartan/lie/so2.h"
#include "cartan/lie/policy.h"

#include <cmath>
#include <string>
#include <expected>

namespace cartan
{

/// 2D rigid body transformation group SE(2), parameterized by scalar type and policy.
/// Internal representation: SO(2) rotation + R^2 translation (D-05 pattern).
/// Reference: Lynch & Park, Modern Robotics, Ch. 3.3.1, p. 86-90.
template <typename Scalar, typename Policy = strict_policy>
class se2
{
public:
    /// Construct from rotation and translation components.
    se2(const so2<Scalar, Policy>& rot, const vector2<Scalar>& trans)
        : m_rotation(rot)
        , m_translation(trans)
    {
    }

    /// Exponential map: se(2) twist -> SE(2) transform.
    /// Twist V = (omega, vx, vy) uses omega-first convention (D-11).
    /// Handles omega~0 via Taylor expansion to avoid division by zero (Pitfall 6).
    /// Reference: Lynch & Park, Modern Robotics, Def. 3.14, p. 88.
    [[nodiscard]] static se2 exp(const vector3<Scalar>& v)
    {
        Scalar omega = v(0);
        Scalar vx = v(1);
        Scalar vy = v(2);

        if (std::abs(omega) < detail::sqrt_epsilon_v<Scalar>)
        {
            // Taylor branch: pure translation (or near-zero rotation)
            // When omega -> 0: sin(omega)/omega -> 1, (1-cos(omega))/omega -> 0
            // Reference: Lynch & Park, p. 88, limiting case.
            return se2(
                so2<Scalar, Policy>::identity(),
                vector2<Scalar>(vx, vy));
        }

        // General case
        // Reference: Lynch & Park, Modern Robotics, Eq. 3.77-3.78, p. 88.
        auto rot = so2<Scalar, Policy>::exp(omega);

        Scalar s = std::sin(omega);
        Scalar c = std::cos(omega);

        // V matrix coefficients: [[sin/omega, -(1-cos)/omega], [(1-cos)/omega, sin/omega]]
        Scalar a = s / omega;
        Scalar b = (Scalar(1) - c) / omega;

        vector2<Scalar> trans;
        trans(0) = a * vx - b * vy;
        trans(1) = b * vx + a * vy;

        return se2(rot, trans);
    }

    /// Logarithmic map: SE(2) transform -> se(2) twist.
    /// Returns V = (omega, vx, vy) in omega-first convention.
    /// Handles omega~0 via Taylor expansion.
    /// Reference: Lynch & Park, Modern Robotics, p. 89-90.
    [[nodiscard]] vector3<Scalar> log() const
    {
        Scalar omega = m_rotation.log();
        vector3<Scalar> result;
        result(0) = omega;

        if (std::abs(omega) < detail::sqrt_epsilon_v<Scalar>)
        {
            // Taylor branch: V^{-1} -> I when omega -> 0
            // Reference: Lynch & Park, p. 89, limiting case.
            result(1) = m_translation(0);
            result(2) = m_translation(1);
            return result;
        }

        // General case: compute V^{-1} * translation
        // V = [[sin/omega, -(1-cos)/omega], [(1-cos)/omega, sin/omega]]
        // V^{-1} = (1/det) * [[sin/omega, (1-cos)/omega], [-(1-cos)/omega, sin/omega]]
        // where det = sin^2/omega^2 + (1-cos)^2/omega^2 = 1/omega^2 * (sin^2 + (1-cos)^2)
        //           = 1/omega^2 * 2*(1-cos) = 2*(1-cos)/omega^2
        // Simplification: V^{-1} = (1/2) * [[omega*sin/(1-cos), omega], [-omega, omega*sin/(1-cos)]]
        // Using half-angle: omega*sin/(2*(1-cos)) = omega/(2*tan(omega/2)) = omega/2 * cot(omega/2)
        // Reference: Lynch & Park, p. 89-90 (adapted for 2D).
        Scalar half_omega = omega / Scalar(2);
        Scalar half_cot = half_omega / std::tan(half_omega);

        // V^{-1} = [[half_cot, half_omega], [-half_omega, half_cot]]
        // But we multiply by (1): V^{-1} * t
        result(1) = half_cot * m_translation(0) + half_omega * m_translation(1);
        result(2) = -half_omega * m_translation(0) + half_cot * m_translation(1);

        return result;
    }

    /// Group inverse: T^{-1} = (R^{-1}, -R^{-1} * t).
    /// Reference: Lynch & Park, Modern Robotics, Eq. 3.64, adapted for 2D.
    [[nodiscard]] se2 inverse() const
    {
        auto rot_inv = m_rotation.inverse();
        return se2(rot_inv, -(rot_inv.matrix() * m_translation));
    }

    /// Group composition: T1 * T2.
    /// Result uses the stricter of the two policies (D-08).
    /// Reference: Lynch & Park, Modern Robotics, homogeneous transform composition.
    template <typename P2>
    [[nodiscard]] auto operator*(const se2<Scalar, P2>& rhs) const
        -> se2<Scalar, stricter_policy<Policy, P2>>
    {
        using ResultPolicy = stricter_policy<Policy, P2>;
        auto new_rot = so2<Scalar, ResultPolicy>(
            (m_rotation * rhs.rotation()).cos_angle(),
            (m_rotation * rhs.rotation()).sin_angle());
        vector2<Scalar> new_trans =
            m_rotation.matrix() * rhs.translation() + m_translation;
        return se2<Scalar, ResultPolicy>(new_rot, new_trans);
    }

    /// Convert to 3x3 homogeneous transformation matrix.
    /// Reference: Lynch & Park, Modern Robotics, Eq. 3.60-3.61.
    [[nodiscard]] Eigen::Matrix<Scalar, 3, 3> matrix() const
    {
        Eigen::Matrix<Scalar, 3, 3> T;
        T.setZero();
        T.template block<2, 2>(0, 0) = m_rotation.matrix();
        T.template block<2, 1>(0, 2) = m_translation;
        T(2, 2) = Scalar(1);
        return T;
    }

    /// Adjoint representation: 3x3 matrix acting on se(2) twists (omega, vx, vy).
    /// For SE(2): Ad_T = [[1, 0, 0], [ty, cos, -sin], [-tx, sin, cos]]
    /// where the twist is (omega, vx, vy) omega-first.
    /// Reference: Derived from Lynch & Park, Modern Robotics, Def. 3.20, adapted for 2D.
    [[nodiscard]] Eigen::Matrix<Scalar, 3, 3> adjoint() const
    {
        Eigen::Matrix<Scalar, 3, 3> Ad;
        Scalar c = m_rotation.cos_angle();
        Scalar s = m_rotation.sin_angle();
        Scalar tx = m_translation(0);
        Scalar ty = m_translation(1);

        // Ad = [[1,   0,    0  ],
        //       [ty,  cos, -sin],
        //       [-tx, sin,  cos]]
        Ad << Scalar(1), Scalar(0), Scalar(0),
              ty,        c,         -s,
              -tx,       s,          c;

        return Ad;
    }

    /// Identity element: no rotation, no translation.
    [[nodiscard]] static se2 identity()
    {
        return se2(so2<Scalar, Policy>::identity(), vector2<Scalar>::Zero());
    }

    /// Construct from 3x3 homogeneous matrix with validation (D-09).
    /// Validates rotation block is SO(2) and bottom row is [0, 0, 1].
    /// Reference: SE(2) matrix structure, Lynch & Park, p. 86.
    [[nodiscard]] static std::expected<se2, std::string>
    from_matrix(const Eigen::Matrix<Scalar, 3, 3>& T)
    {
        Scalar tol = detail::sqrt_epsilon_v<Scalar>;

        // Check bottom row is [0, 0, 1]
        if (std::abs(T(2, 0)) > tol || std::abs(T(2, 1)) > tol ||
            std::abs(T(2, 2) - Scalar(1)) > tol)
        {
            return std::unexpected("Bottom row is not [0, 0, 1]");
        }

        // Validate rotation block
        matrix2<Scalar> R = T.template block<2, 2>(0, 0);
        auto rot_result = so2<Scalar, Policy>::from_matrix(R);
        if (!rot_result.has_value())
        {
            return std::unexpected("Rotation block invalid: " + rot_result.error());
        }

        vector2<Scalar> trans = T.template block<2, 1>(0, 2);
        return se2(rot_result.value(), trans);
    }

    /// Access the rotation component.
    [[nodiscard]] const so2<Scalar, Policy>& rotation() const { return m_rotation; }

    /// Access the translation component.
    [[nodiscard]] const vector2<Scalar>& translation() const { return m_translation; }

    /// Transform a 2D point: R * p + t.
    /// Reference: SE(2) action on R^2.
    [[nodiscard]] vector2<Scalar> act(const vector2<Scalar>& p) const
    {
        return m_rotation.act(p) + m_translation;
    }

private:
    so2<Scalar, Policy> m_rotation;
    vector2<Scalar> m_translation;
};

}

#endif
