/// @file lie_basics.cpp
/// @brief Demonstrates SO(3) and SE(3) Lie group operations.
///
/// Shows: exp/log maps, group composition, adjoint, inverse, rotation of vectors.

#include "liepp/lie.h"

#include <iostream>

int main()
{
    using SO3 = liepp::so3<double>;
    using SE3 = liepp::se3<double>;
    using vec3 = liepp::vector3<double>;
    using vec6 = liepp::vector6<double>;

    // --- SO(3): 3D rotations ---
    vec3 phi{0.1, 0.2, 0.3};
    auto R = SO3::exp(phi);
    std::cout << "SO(3) exp/log round-trip error: " << (phi - R.log()).norm() << "\n";

    auto R2 = SO3::exp(vec3{0.5, -0.1, 0.0});
    auto R3 = R * R2;                        // group composition
    auto R_inv = R.inverse();                // group inverse
    std::cout << "R * R_inv ~ I error: " << (R * R_inv).log().norm() << "\n";

    auto Ad_R = R.adjoint();                 // 3x3 adjoint (rotation matrix)
    vec3 v{1.0, 0.0, 0.0};
    std::cout << "Rotated vector: " << R.act(v).transpose() << "\n";

    // --- SE(3): 3D rigid body transformations ---
    vec6 twist;
    twist << 0.0, 0.0, 0.3, 0.1, 0.2, 0.0;  // (omega, rho)
    auto T = SE3::exp(twist);
    std::cout << "SE(3) exp/log round-trip error: " << (twist - T.log()).norm() << "\n";

    auto T2 = SE3::exp(vec6{{0.1, -0.1, 0.0, 0.0, 0.0, 0.5}});
    auto T3 = T * T2;                        // rigid body composition
    std::cout << "Translation: " << T3.translation().transpose() << "\n";
    std::cout << "Rotation matrix:\n" << T3.rotation().matrix() << "\n";
}
