#ifndef HPP_GUARD_CARTAN_LIE_HAT_VEE_H
#define HPP_GUARD_CARTAN_LIE_HAT_VEE_H

#include "cartan/types.h"

namespace cartan
{

/// Constructs a 3x3 skew-symmetric matrix from a 3-vector.
/// hat([v1, v2, v3]) = [[0, -v3, v2], [v3, 0, -v1], [-v2, v1, 0]]
/// Property: hat(v) * w = v x w (cross product).
/// Reference: Lynch & Park, Modern Robotics, Def. 3.7 / Eq. 3.30, p. 75.
template <typename Scalar>
matrix3<Scalar> hat(const vector3<Scalar>& v)
{
    matrix3<Scalar> S;
    S <<      Scalar(0), -v(2),        v(1),
              v(2),       Scalar(0),  -v(0),
             -v(1),       v(0),        Scalar(0);
    return S;
}

/// Extracts a 3-vector from a 3x3 skew-symmetric matrix.
/// Inverse of hat for so(3): vee(hat(v)) == v.
/// Reference: Lynch & Park, Modern Robotics, inverse of Def. 3.7, p. 75.
template <typename Scalar>
vector3<Scalar> vee(const matrix3<Scalar>& S)
{
    vector3<Scalar> v;
    v << S(2, 1), S(0, 2), S(1, 0);
    return v;
}

/// Constructs a 4x4 se(3) twist matrix from a 6-vector.
/// Twist V = (omega, v) uses omega-first convention per Lynch & Park.
/// Layout: top-left 3x3 = hat(omega), top-right 3x1 = v, bottom row = zeros.
/// Reference: Lynch & Park, Modern Robotics, Eq. 3.85, p. 103.
template <typename Scalar>
matrix4<Scalar> hat(const vector6<Scalar>& V)
{
    matrix4<Scalar> M;
    M.setZero();
    vector3<Scalar> omega = V.template head<3>();
    M.template block<3, 3>(0, 0) = hat(omega);
    M.template block<3, 1>(0, 3) = V.template tail<3>();
    return M;
}

/// Extracts a 6-vector from a 4x4 se(3) twist matrix.
/// Returns V = (omega, v) in omega-first convention.
/// Inverse of hat for se(3): vee(hat(V)) == V.
/// Reference: Lynch & Park, Modern Robotics, inverse of Eq. 3.85, p. 103.
template <typename Scalar>
vector6<Scalar> vee(const matrix4<Scalar>& M)
{
    vector6<Scalar> V;
    matrix3<Scalar> omega_hat = M.template block<3, 3>(0, 0);
    V.template head<3>() = vee(omega_hat);
    V.template tail<3>() = M.template block<3, 1>(0, 3);
    return V;
}

}

#endif
