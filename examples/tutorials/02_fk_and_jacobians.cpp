/// @file 02_fk_and_jacobians.cpp
/// @brief Forward kinematics with space and body Jacobians, walked through
///        twice: first on a planar 3R built from screw_axis primitives, then
///        on a spatial 6R modeled on KUKA KR 6 R900 kinematics.
///
/// Shows: cartan::screw_axis::revolute construction, kinematic_chain<Scalar, N>
///        assembly from primitives, cartan::forward_kinematics, and the two
///        Jacobian flavors -- space-frame and body-frame -- that the
///        product-of-exponentials formulation exposes. Each step is annotated
///        with Lynch & Park (Modern Robotics) chapter references.

#include "cartan/serial_chain.h"

#include <iostream>
#include <numbers>

int main()
{
    using vec3 = cartan::vector3<double>;

    // Joint limits are shared across every joint in both sections. The chains
    // exercised here have full revolute range; bounded ranges are demonstrated
    // in the URDF walkthrough tutorial where they are parsed from the robot
    // description.
    cartan::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

    // --- Section 1: planar 3R -----------------------------------------------
    //
    // Three revolute joints rotating about the world z-axis. Joint i sits at
    // the world-frame point q_i = (sum of preceding link lengths, 0, 0). For
    // a pure-rotation screw axis the space-frame screw is
    //
    //     S_i = (omega_i, -omega_i x q_i)
    //
    // and the rotation axis omega_i alone determines the joint's effect on
    // end-effector velocity once the link geometry is fixed. The planar 3R
    // is the textbook entry point for the product-of-exponentials formula --
    // every joint commutes with itself, the home pose is trivial, and the
    // screw axes are interpretable by inspection. See Lynch & Park,
    // Modern Robotics, Ch. 3.3.2 for the screw-axis definition and
    // Ch. 4 for the chained product T(q) = exp([S_1] q_1) ... exp([S_n] q_n) M.
    std::cout << "=== Section 1: planar 3R =====================================\n\n";

    // Link lengths picked for textbook readability: 0.30 + 0.25 + 0.20 = 0.75 m
    auto s1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0,    0, 0));
    auto s2 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0.30, 0, 0));
    auto s3 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0.55, 0, 0));

    vec3 home_trans_3r(0.75, 0, 0);
    auto home_3r = cartan::se3<double>(cartan::so3<double>::identity(), home_trans_3r);
    cartan::kinematic_chain<double, 3> chain_3r(
        home_3r, {s1, s2, s3}, {lim, lim, lim});

    // Evaluate at a small but non-degenerate joint configuration.
    Eigen::Vector3d q_3r{0.4, -0.3, 0.5};
    auto fk_3r = cartan::forward_kinematics(chain_3r, q_3r);

    std::cout << "Joint configuration q (rad): " << q_3r.transpose() << "\n\n";
    std::cout << "End-effector pose T(q):\n"
              << fk_3r.end_effector.matrix() << "\n\n";

    // The space Jacobian J_s relates joint rates to the end-effector twist
    // expressed in the space frame: V_s = J_s(q) * q_dot. Its i-th column is
    // the screw axis S_i adjoint-transported through the partial product of
    // exponentials from joint 1..i-1. See Lynch & Park Ch. 5.1.
    std::cout << "Space Jacobian J_s (6x3):\n"
              << cartan::space_jacobian(chain_3r, fk_3r) << "\n\n";

    // The body Jacobian J_b relates joint rates to the end-effector twist
    // expressed in the body (end-effector) frame: V_b = J_b(q) * q_dot. It is
    // the natural quantity for IK -- the error twist log(T_curr^-1 * T_target)
    // already lives in the body frame, so J_b is what the LM iteration
    // factorizes. See Lynch & Park Ch. 5.1.
    std::cout << "Body  Jacobian J_b (6x3):\n"
              << cartan::body_jacobian(chain_3r, fk_3r) << "\n\n";

    // --- Section 2: spatial 6R (KUKA KR 6 R900 SIXX kinematics) -------------
    //
    // The KR 6 R900 SIXX is a compact six-axis industrial arm whose joint
    // axes alternate between vertical, horizontal-parallel, and wrist-roll
    // orientations. The dimensions below come from the working-envelope
    // drawing in the manufacturer's datasheet -- the kinematic structure of
    // a serial chain is not copyrightable, so reproducing the (axis, point)
    // pairs as screw-axis primitives is straightforward.
    //
    // The product-of-exponentials form scales identically to the planar
    // case: each additional joint contributes one exp([S_i] q_i) factor to
    // the chained product, and the space / body Jacobian formulas pick up
    // one additional column. See Lynch & Park Ch. 4.1 for the general
    // serial-chain PoE form.
    std::cout << "=== Section 2: spatial 6R (KUKA KR 6 R900 SIXX) ==============\n\n";

    auto k1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0,     0, 0));
    auto k2 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0,     0, 0.400));
    auto k3 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0.455, 0, 0.400));
    auto k4 = cartan::screw_axis<double>::revolute(vec3(1, 0, 0), vec3(0.875, 0, 0.400));
    auto k5 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0.875, 0, 0.400));
    auto k6 = cartan::screw_axis<double>::revolute(vec3(1, 0, 0), vec3(0.935, 0, 0.400));

    vec3 home_trans_6r(0.935, 0, 0.400);
    auto home_6r = cartan::se3<double>(cartan::so3<double>::identity(), home_trans_6r);
    cartan::kinematic_chain<double, 6> chain_6r(
        home_6r, {k1, k2, k3, k4, k5, k6}, {lim, lim, lim, lim, lim, lim});

    Eigen::Vector<double, 6> q_6r{0.2, -0.4, 0.3, -0.5, 0.6, -0.2};
    auto fk_6r = cartan::forward_kinematics(chain_6r, q_6r);

    std::cout << "Joint configuration q (rad): " << q_6r.transpose() << "\n\n";
    std::cout << "End-effector pose T(q):\n"
              << fk_6r.end_effector.matrix() << "\n\n";

    // Both Jacobians are 6x6 for a six-DOF chain. The rank of either matrix
    // at a given q reports the local mobility of the end-effector; a rank
    // deficiency signals a kinematic singularity. The condition number of
    // J_b is the natural conditioning measure for the local IK problem.
    std::cout << "Space Jacobian J_s (6x6):\n"
              << cartan::space_jacobian(chain_6r, fk_6r) << "\n\n";
    std::cout << "Body  Jacobian J_b (6x6):\n"
              << cartan::body_jacobian(chain_6r, fk_6r) << "\n";

    return 0;
}
