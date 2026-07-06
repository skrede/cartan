#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_SCREW_AXIS_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_SCREW_AXIS_H

/// Screw axis representation for kinematic chain joints.
///
/// A screw axis S = (omega, v) where ||omega|| = 1 for revolute joints
/// or omega = 0 and ||v|| = 1 for prismatic joints. Used in the Product
/// of Exponentials (PoE) formula for forward kinematics.
///
/// Reference: Lynch & Park, Modern Robotics, Def. 3.24, p. 102.

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"

#include "cartan/lie/lie_failure.h"

#include <cmath>
#include "cartan/expected.h"

namespace cartan
{

/// Screw axis for a kinematic joint in PoE form.
/// Revolute: omega is unit rotation axis, v = -omega x point_on_axis.
/// Prismatic: omega = 0, v is unit translation direction.
/// Reference: Lynch & Park, Modern Robotics, Def. 3.24, p. 102.
template <typename Scalar = double>
class screw_axis
{
public:
    /// Construct a revolute joint screw axis.
    static screw_axis revolute(
        const vector3<Scalar>& axis,
        const vector3<Scalar>& point)
    {
        vector3<Scalar> w = axis.normalized();
        vector3<Scalar> v = -w.cross(point);
        return screw_axis(w, v);
    }

    /// Construct a prismatic joint screw axis.
    static screw_axis prismatic(const vector3<Scalar>& direction)
    {
        return screw_axis(vector3<Scalar>::Zero(), direction.normalized());
    }

    /// Construct from a 6-vector (omega, v) with unit constraint validation.
    /// Revolute (||omega|| > 0): requires ||omega|| = 1.
    /// Prismatic (omega = 0): requires ||v|| = 1.
    static cartan::expected<screw_axis, lie_failure> from_vector(
        const vector6<Scalar>& vec)
    {
        vector3<Scalar> omega = vec.template head<3>();
        vector3<Scalar> v = vec.template tail<3>();

        Scalar omega_norm = omega.norm();

        if (omega_norm > detail::sqrt_epsilon_v<Scalar>)
        {
            // Revolute: ||omega|| must be 1
            if (std::abs(omega_norm - Scalar(1)) > detail::sqrt_epsilon_v<Scalar>)
            {
                return cartan::unexpected(lie_failure::non_unit_screw_axis);
            }
            return screw_axis(omega, v);
        }

        // Prismatic: ||v|| must be 1
        Scalar v_norm = v.norm();
        if (std::abs(v_norm - Scalar(1)) > detail::sqrt_epsilon_v<Scalar>)
        {
            return cartan::unexpected(lie_failure::non_unit_screw_axis);
        }
        return screw_axis(omega, v);
    }

    /// Angular velocity component (rotation axis for revolute, zero for prismatic).
    const vector3<Scalar>& omega() const { return m_omega; }

    /// Linear velocity component.
    const vector3<Scalar>& v() const { return m_v; }

    /// Export as 6-vector (omega, v) in omega-first convention.
    vector6<Scalar> to_vector() const
    {
        vector6<Scalar> vec;
        vec.template head<3>() = m_omega;
        vec.template tail<3>() = m_v;
        return vec;
    }

    /// True if this is a revolute (rotational) joint axis.
    bool is_revolute() const
    {
        return m_omega.squaredNorm() > detail::epsilon_v<Scalar>;
    }

    /// True if this is a prismatic (translational) joint axis.
    bool is_prismatic() const { return !is_revolute(); }

private:
    screw_axis(const vector3<Scalar>& omega, const vector3<Scalar>& v)
        : m_omega(omega)
        , m_v(v)
    {
    }

    vector3<Scalar> m_omega;  ///< Angular component
    vector3<Scalar> m_v;      ///< Linear component
};

}

#endif
