#ifndef HPP_GUARD_CARTAN_LIE_SO2_H
#define HPP_GUARD_CARTAN_LIE_SO2_H

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"

#include "cartan/lie/policy.h"
#include "cartan/lie/lie_failure.h"

#include <cmath>
#include <cassert>
#include "cartan/expected.h"

namespace cartan
{

/// 2D rotation group SO(2), parameterized by scalar type and policy.
/// Internal representation: (cos(theta), sin(theta)) pair.
/// Reference: Lynch & Park, Modern Robotics, Ch. 3.2.1, p. 68-69.
template <typename Scalar, typename Policy = strict_policy>
class so2
{
public:
    /// Default constructor: the identity rotation (cos = 1, sin = 0). Already
    /// unit, so no normalization runs regardless of policy.
    so2()
        : m_cos(Scalar(1))
        , m_sin(Scalar(0))
    {
    }

    /// Construct from cos/sin pair. Strict policy normalizes to unit circle.
    so2(Scalar cos_val, Scalar sin_val)
        : m_cos(cos_val)
        , m_sin(sin_val)
    {
        if constexpr (Policy::normalize_on_construct)
        {
            Scalar n = std::hypot(m_cos, m_sin);
            if constexpr (Policy::assert_valid)
            {
                assert(n > detail::epsilon_v<Scalar>);
            }
            m_cos /= n;
            m_sin /= n;
        }
    }

    /// Exponential map: angle -> SO(2) rotation.
    /// Reference: Lynch & Park, Modern Robotics, Def. 3.2, p. 68.
    static so2 exp(Scalar theta)
    {
        return so2(std::cos(theta), std::sin(theta));
    }

    /// Logarithmic map: SO(2) rotation -> angle in (-pi, pi].
    /// Reference: Lynch & Park, Modern Robotics, p. 69.
    Scalar log() const
    {
        return std::atan2(m_sin, m_cos);
    }

    /// Group inverse: R^{-1} = R^T, which negates the sine component.
    /// Reference: Rotation inverse is transpose for orthogonal matrices.
    so2 inverse() const
    {
        return so2(m_cos, -m_sin);
    }

    /// Group composition: R1 * R2 via angle-addition formulas.
    /// Result uses the stricter of the two policies.
    /// Reference: Lynch & Park, Modern Robotics, rotation composition, p. 68.
    template <typename P2>
    auto operator*(const so2<Scalar, P2>& rhs) const
        -> so2<Scalar, stricter_policy<Policy, P2>>
    {
        return so2<Scalar, stricter_policy<Policy, P2>>(
            m_cos * rhs.cos_angle() - m_sin * rhs.sin_angle(),
            m_sin * rhs.cos_angle() + m_cos * rhs.sin_angle());
    }

    /// Convert to 2x2 rotation matrix: [[c, -s], [s, c]].
    /// Reference: Lynch & Park, Modern Robotics, Eq. 3.10, p. 68.
    matrix2<Scalar> matrix() const
    {
        matrix2<Scalar> R;
        R << m_cos, -m_sin,
             m_sin,  m_cos;
        return R;
    }

    /// Angle accessor (same as log).
    Scalar angle() const
    {
        return log();
    }

    /// Manifold-aware approximate equality: true iff the wrapped angular
    /// distance between the two rotations is at most tol (radians). Uses
    /// std::abs((this->inverse() * other).log()), so angles straddling +/-pi
    /// that denote the same rotation compare equal. No exact equality operator
    /// is provided: bit-exact floating compare on cos/sin is a footgun (angle
    /// wrap, drift).
    bool isApprox(const so2& other, Scalar tol) const
    {
        return std::abs((inverse() * other).log()) <= tol;
    }

    /// Identity element: zero rotation.
    static so2 identity()
    {
        return so2(Scalar(1), Scalar(0));
    }

    /// Construct from 2x2 rotation matrix with validation.
    /// Checks orthogonality (R^T * R ~= I) and det(R) ~= 1.
    /// Returns cartan::unexpected with a lie_failure code on failure.
    /// Reference: Rotation matrix properties, Lynch & Park, p. 23-24.
    static cartan::expected<so2, lie_failure>
    from_matrix(const matrix2<Scalar>& R)
    {
        Scalar tol = detail::sqrt_epsilon_v<Scalar>;

        // Check orthogonality: R^T * R ~= I
        matrix2<Scalar> RtR = R.transpose() * R;
        if ((RtR - matrix2<Scalar>::Identity()).norm() > tol)
        {
            return cartan::unexpected(lie_failure::non_orthogonal);
        }

        // Check determinant ~= +1 (not a reflection)
        Scalar det = R.determinant();
        if (std::abs(det - Scalar(1)) > tol)
        {
            return cartan::unexpected(lie_failure::improper_rotation);
        }

        return so2(R(0, 0), R(1, 0));
    }

    /// Direct access to cosine component.
    Scalar cos_angle() const { return m_cos; }

    /// Direct access to sine component.
    Scalar sin_angle() const { return m_sin; }

    /// Rotate a 2D vector: R * v.
    /// Reference: 2D rotation action on R^2.
    vector2<Scalar> act(const vector2<Scalar>& v) const
    {
        return matrix() * v;
    }

private:
    Scalar m_cos;
    Scalar m_sin;
};

}

#endif
