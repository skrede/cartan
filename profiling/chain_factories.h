#ifndef HPP_GUARD_CARTAN_PROFILING_CHAIN_FACTORIES_H
#define HPP_GUARD_CARTAN_PROFILING_CHAIN_FACTORIES_H

/// @file chain_factories.h
/// @brief Shared cartan PoE chain factories, random target generation, and error
///        decomposition utilities for benchmarking and profiling.
///
/// Extracted from benchmarks/benchmark_utils.h so that both the benchmark suite
/// and the standalone profiling tool can share the same robot definitions
/// without pulling in external robot library dependencies.

#include <cartan/types.h>
#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/chain/storage_trait.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <cmath>
#include <random>
#include <numbers>
#include <utility>

namespace cartan::benchmarks
{

// ===========================================================================
// cartan PoE chain factories
// ===========================================================================

/// Planar 3R chain (3-DOF).
/// Three revolute joints about z, unit link lengths.
/// Classic textbook example.
template <typename Scalar>
auto make_3r_planar_chain() -> cartan::kinematic_chain<Scalar, 3>
{
    using vec3 = cartan::vector3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(1), Scalar(0), Scalar(0)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(2), Scalar(0), Scalar(0)));

    vec3 home_trans(Scalar(3), Scalar(0), Scalar(0));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return cartan::kinematic_chain<Scalar, 3>(
        home,
        {s1, s2, s3},
        {lim, lim, lim});
}

/// Universal Robots UR3e (6-DOF).
/// DH parameters from UR official documentation, converted to PoE.
/// Reference: UR official DH parameters page.
template <typename Scalar>
auto make_ur3e_chain() -> cartan::kinematic_chain<Scalar, 6>
{
    using vec3 = cartan::vector3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.15185)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(-0.24355), Scalar(0), Scalar(0.15185)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(-0.45675), Scalar(0), Scalar(0.15185)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(-1)),
        vec3(Scalar(-0.45675), Scalar(0.13105), Scalar(0)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(-0.45675), Scalar(0), Scalar(-0.08535)));

    vec3 home_trans(Scalar(-0.45675), Scalar(0.22315), Scalar(0.0665));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return cartan::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});
}

/// KUKA LBR Med 14 R820 (7-DOF).
/// Same geometry as LBR iiwa in test_utils.h.
/// Link lengths from LBR Med 14 R820 technical data.
/// Joint axis pattern: alternating z/y for 7-DOF LBR.
/// Reference: Lynch & Park, Modern Robotics, Ch. 4 (PoE form).
template <typename Scalar>
auto make_lbr_med14_chain() -> cartan::kinematic_chain<Scalar, 7>
{
    using vec3 = cartan::vector3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.360)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.360)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.780)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.780)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(1.180)));
    auto s7 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(1.180)));

    vec3 home_trans(Scalar(0), Scalar(0), Scalar(1.306));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return cartan::kinematic_chain<Scalar, 7>(
        home,
        {s1, s2, s3, s4, s5, s6, s7},
        {lim, lim, lim, lim, lim, lim, lim});
}

/// KUKA KR 6 R900 SIXX (6-DOF).
/// Dimensions estimated from working envelope drawing.
/// Zero configuration is forward-pointing (arm extended horizontally).
/// Reference: KUKA KR 6 SIXX working envelope drawing.
template <typename Scalar>
auto make_kr6_sixx_chain() -> cartan::kinematic_chain<Scalar, 6>
{
    using vec3 = cartan::vector3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.400)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.455), Scalar(0), Scalar(0.400)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.875), Scalar(0), Scalar(0.400)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.875), Scalar(0), Scalar(0.400)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.935), Scalar(0), Scalar(0.400)));

    vec3 home_trans(Scalar(0.935), Scalar(0), Scalar(0.400));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return cartan::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});
}

/// Franka Emika Panda (7-DOF).
/// DH parameters from Franka Emika official documentation.
/// d1=0.333, d3=0.316, d5=0.384, df=0.107, a4=0.0825, a5=-0.0825, a7=0.088.
/// At zero config the arm is vertical with alternating z/y axes.
/// Reference: Franka Emika Panda datasheet, "FCI documentation DH parameters".
template <typename Scalar>
auto make_panda_chain() -> cartan::kinematic_chain<Scalar, 7>
{
    using vec3 = cartan::vector3<Scalar>;

    // Joint axes and points at zero configuration:
    // J1: z-axis at origin
    // J2: y-axis at (0, 0, 0.333)
    // J3: z-axis at (0, 0, 0.333)
    // J4: -y-axis at (0.0825, 0, 0.649) [d1+d3=0.649, a4=0.0825]
    // J5: z-axis at (0, 0, 0.649)
    // J6: y-axis at (0, 0, 1.033) [d1+d3+d5=1.033]
    // J7: z-axis at (0.088, 0, 1.033) [a7=0.088]
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.333)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.333)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.0825), Scalar(0), Scalar(0.649)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.649)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(1.033)));
    auto s7 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0.088), Scalar(0), Scalar(1.033)));

    // Home: end-effector at (0.088, 0, 1.033 + 0.107) = (0.088, 0, 1.140)
    vec3 home_trans(Scalar(0.088), Scalar(0), Scalar(1.140));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return cartan::kinematic_chain<Scalar, 7>(
        home,
        {s1, s2, s3, s4, s5, s6, s7},
        {lim, lim, lim, lim, lim, lim, lim});
}

/// ABB IRB 120 (6-DOF).
/// DH parameters from ABB IRB 120 product specification.
/// d1=0.290, a1=0, a2=0.270, d4=0.302, d6=0.072.
/// At zero config the arm is vertical.
/// Reference: ABB IRB 120 product specification datasheet.
template <typename Scalar>
auto make_abb_irb120_chain() -> cartan::kinematic_chain<Scalar, 6>
{
    using vec3 = cartan::vector3<Scalar>;

    // J1: z at origin
    // J2: y at (0, 0, 0.290)
    // J3: y at (0, 0, 0.560)  [0.290 + 0.270]
    // J4: x at (0, 0, 0.560)
    // J5: y at (0, 0, 0.862)  [0.560 + 0.302]
    // J6: x at (0, 0, 0.862)
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.290)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.560)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.560)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.862)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.862)));

    // Home: end-effector at (0, 0, 0.862 + 0.072) = (0, 0, 0.934)
    vec3 home_trans(Scalar(0), Scalar(0), Scalar(0.934));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return cartan::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});
}

/// Kinova Jaco2 (6-DOF).
/// DH parameters from Kinova Jaco2 technical documentation.
/// d1=0.2755, d2=0.4100, d3=0.2073, d4=0.0743, d5=0.0743, d6=0.1687.
/// Spherical wrist with offset geometry.
/// Reference: Kinova Jaco2 User Guide, DH parameter table.
template <typename Scalar>
auto make_jaco2_chain() -> cartan::kinematic_chain<Scalar, 6>
{
    using vec3 = cartan::vector3<Scalar>;

    // Simplified vertical zero-config PoE representation:
    // J1: z at origin
    // J2: y at (0, 0, 0.2755)
    // J3: y at (0.410, 0, 0.2755) [a2=0.410 along x after shoulder]
    // J4: x at (0.410, 0.2073, 0.2755) [d3 along y for wrist offset]
    // J5: y at (0.410, 0.2816, 0.2755) [d3+d4]
    // J6: x at (0.410, 0.3559, 0.2755) [d3+d4+d5]
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.2755)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.410), Scalar(0), Scalar(0.2755)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.410), Scalar(0.2073), Scalar(0.2755)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.410), Scalar(0.2816), Scalar(0.2755)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.410), Scalar(0.3559), Scalar(0.2755)));

    // Home: end-effector at (0.410, 0.3559+0.1687, 0.2755) = (0.410, 0.5246, 0.2755)
    vec3 home_trans(Scalar(0.410), Scalar(0.5246), Scalar(0.2755));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return cartan::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});
}

/// Fetch Robotics arm (7-DOF).
/// DH parameters from Fetch Robotics URDF / ROS wiki.
/// Vertical zero config. Shoulder height 0.40, upper arm 0.321, forearm 0.321,
/// wrist offset 0.1385.
/// Reference: Fetch Robotics arm URDF, ROS wiki documentation.
template <typename Scalar>
auto make_fetch_chain() -> cartan::kinematic_chain<Scalar, 7>
{
    using vec3 = cartan::vector3<Scalar>;

    // J1: z at origin (shoulder pan)
    // J2: y at (0, 0, 0.400) (shoulder lift)
    // J3: x at (0, 0, 0.400) (upperarm roll)
    // J4: y at (0.321, 0, 0.400) (elbow flex)
    // J5: x at (0.321, 0, 0.400) (forearm roll)
    // J6: y at (0.642, 0, 0.400) (wrist flex)
    // J7: x at (0.642, 0, 0.400) (wrist roll)
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.400)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.400)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.321), Scalar(0), Scalar(0.400)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.321), Scalar(0), Scalar(0.400)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.642), Scalar(0), Scalar(0.400)));
    auto s7 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.642), Scalar(0), Scalar(0.400)));

    // Home: end-effector at (0.642 + 0.1385, 0, 0.400) = (0.7805, 0, 0.400)
    vec3 home_trans(Scalar(0.7805), Scalar(0), Scalar(0.400));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return cartan::kinematic_chain<Scalar, 7>(
        home,
        {s1, s2, s3, s4, s5, s6, s7},
        {lim, lim, lim, lim, lim, lim, lim});
}

/// Rethink Baxter single arm (7-DOF).
/// DH parameters from Rethink Robotics Baxter URDF.
/// Shoulder height 0.2703, upper arm 0.3644, forearm 0.3743,
/// wrist link 0.2295.
/// Reference: Rethink Robotics Baxter URDF / technical specification.
template <typename Scalar>
auto make_baxter_chain() -> cartan::kinematic_chain<Scalar, 7>
{
    using vec3 = cartan::vector3<Scalar>;

    // Baxter has an offset shoulder. Simplified vertical zero-config:
    // J1: z at origin (shoulder pan -- S0)
    // J2: y at (0.069, 0, 0.2703) (shoulder pitch -- S1)
    // J3: x at (0.069, 0, 0.2703) (shoulder roll -- E0)
    // J4: y at (0.069+0.3644, 0, 0.2703) = (0.4334, 0, 0.2703) (elbow -- E1)
    // J5: x at (0.4334, 0, 0.2703) (forearm roll -- W0)
    // J6: y at (0.4334+0.3743, 0, 0.2703) = (0.8077, 0, 0.2703) (wrist pitch -- W1)
    // J7: x at (0.8077, 0, 0.2703) (wrist roll -- W2)
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.069), Scalar(0), Scalar(0.2703)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.069), Scalar(0), Scalar(0.2703)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.4334), Scalar(0), Scalar(0.2703)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.4334), Scalar(0), Scalar(0.2703)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.8077), Scalar(0), Scalar(0.2703)));
    auto s7 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.8077), Scalar(0), Scalar(0.2703)));

    // Home: end-effector at (0.8077 + 0.2295, 0, 0.2703) = (1.0372, 0, 0.2703)
    vec3 home_trans(Scalar(1.0372), Scalar(0), Scalar(0.2703));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return cartan::kinematic_chain<Scalar, 7>(
        home,
        {s1, s2, s3, s4, s5, s6, s7},
        {lim, lim, lim, lim, lim, lim, lim});
}

/// KUKA LWR 4+ (7-DOF).
/// DH parameters from KUKA LWR 4+ technical documentation.
/// d1=0.3105, d3=0.4, d5=0.39, d7=0.078. Alternating z/y axis pattern.
/// Reference: KUKA LWR 4+ product specification / DLR lightweight robot documentation.
template <typename Scalar>
auto make_kuka_lwr4_chain() -> cartan::kinematic_chain<Scalar, 7>
{
    using vec3 = cartan::vector3<Scalar>;

    // J1: z at origin
    // J2: y at (0, 0, 0.3105)
    // J3: z at (0, 0, 0.3105)
    // J4: -y at (0, 0, 0.7105)  [0.3105 + 0.4]
    // J5: z at (0, 0, 0.7105)
    // J6: y at (0, 0, 1.1005)   [0.7105 + 0.39]
    // J7: z at (0, 0, 1.1005)
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.3105)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.3105)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.7105)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.7105)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(1.1005)));
    auto s7 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(1.1005)));

    // Home: end-effector at (0, 0, 1.1005 + 0.078) = (0, 0, 1.1785)
    vec3 home_trans(Scalar(0), Scalar(0), Scalar(1.1785));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return cartan::kinematic_chain<Scalar, 7>(
        home,
        {s1, s2, s3, s4, s5, s6, s7},
        {lim, lim, lim, lim, lim, lim, lim});
}

// ===========================================================================
// Dynamic chain variants
// ===========================================================================

/// UR3e dynamic variant (runtime DOF).
template <typename Scalar>
auto make_ur3e_chain_dynamic() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    return make_ur3e_chain<Scalar>().to_dynamic();
}

/// LBR Med 14 dynamic variant (runtime DOF).
template <typename Scalar>
auto make_lbr_med14_chain_dynamic() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    return make_lbr_med14_chain<Scalar>().to_dynamic();
}

// ===========================================================================
// Random target generation
// ===========================================================================

/// Generate random joint configuration within chain limits.
template <int N, typename Scalar>
auto random_joint_config(
    const cartan::kinematic_chain<Scalar, N>& chain,
    std::mt19937& rng)
    -> typename cartan::joint_state<Scalar, N>::position_type
{
    using position_type = typename cartan::joint_state<Scalar, N>::position_type;
    int n = chain.num_joints();

    position_type q;
    if constexpr (N == cartan::dynamic)
    {
        q.resize(n);
    }

    std::uniform_real_distribution<Scalar> dist;
    const auto& limits = chain.limits();
    for (int i = 0; i < n; ++i)
    {
        auto idx = static_cast<std::size_t>(i);
        dist = std::uniform_real_distribution<Scalar>(
            limits[idx].position_min, limits[idx].position_max);
        q(i) = dist(rng);
    }

    return q;
}

/// Generate a guaranteed-reachable target by running FK on a random config.
template <int N, typename Scalar>
auto random_reachable_target(
    const cartan::kinematic_chain<Scalar, N>& chain,
    std::mt19937& rng)
    -> cartan::se3<Scalar>
{
    auto q = random_joint_config(chain, rng);
    auto fk = cartan::forward_kinematics(chain, q);
    return fk.end_effector;
}

// ===========================================================================
// Error decomposition utility
// ===========================================================================

/// Compute position and orientation error between FK(q_solution) and target.
/// Returns {position_error, orientation_error} in meters and radians.
/// Uses body-frame twist error: log(T_fk^{-1} * T_target).
template <int N, typename Scalar>
auto compute_pose_errors(
    const cartan::kinematic_chain<Scalar, N>& chain,
    const typename cartan::joint_state<Scalar, N>::position_type& q_solution,
    const cartan::se3<Scalar>& target)
    -> std::pair<Scalar, Scalar>
{
    auto fk = cartan::forward_kinematics(chain, q_solution);
    auto error_twist = (fk.end_effector.inverse() * target).log();

    // omega-first convention: head<3> = angular, tail<3> = linear
    Scalar orientation_error = error_twist.template head<3>().norm();
    Scalar position_error = error_twist.template tail<3>().norm();

    return {position_error, orientation_error};
}

}

#endif
