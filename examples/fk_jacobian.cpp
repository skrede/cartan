/// @file fk_jacobian.cpp
/// @brief Forward kinematics and Jacobians for a UR3e 6-DOF robot.
///
/// Shows: kinematic_chain construction, forward_kinematics, space_jacobian,
/// body_jacobian. Uses UR3e PoE screw parameters (hardcoded).

#include "cartan/serial_chain.h"

#include <iostream>
#include <numbers>

int main()
{
    using vec3 = cartan::vector3<double>;

    // UR3e 6-DOF: PoE screw axes in space frame
    auto s1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0, 0, 0.15185));
    auto s3 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(-0.24355, 0, 0.15185));
    auto s4 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(-0.45675, 0, 0.15185));
    auto s5 = cartan::screw_axis<double>::revolute(vec3(0, 0, -1), vec3(-0.45675, 0.13105, 0));
    auto s6 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(-0.45675, 0, -0.08535));

    vec3 home_trans(-0.45675, 0.22315, 0.0665);
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    cartan::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    cartan::kinematic_chain<double, 6> chain(
        home, {s1, s2, s3, s4, s5, s6}, {lim, lim, lim, lim, lim, lim});

    // FK at a sample configuration
    Eigen::Vector<double, 6> q{0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
    auto fk = cartan::forward_kinematics(chain, q);

    std::cout << "End-effector pose:\n" << fk.end_effector.matrix() << "\n\n";

    // Space and body Jacobians
    auto Js = cartan::space_jacobian(chain, fk);
    auto Jb = cartan::body_jacobian(chain, fk);

    std::cout << "Space Jacobian:\n" << Js << "\n\n";
    std::cout << "Body Jacobian:\n" << Jb << "\n";
}
