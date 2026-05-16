#ifndef HPP_GUARD_CARTAN_TESTS_FIXTURES_CHAIN_FACTORIES_H
#define HPP_GUARD_CARTAN_TESTS_FIXTURES_CHAIN_FACTORIES_H

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
#include <cartan/serial/chain/joint_tags.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/chain/storage_trait.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <cmath>
#include <random>
#include <limits>
#include <numbers>
#include <utility>

namespace cartan::fixtures
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
// Additive static_chain / paired kinematic_chain factories
//
// Each fixture exposes a `_geometry` helper in `detail::` that returns the
// screw axes, home pose, and joint limits for a single chain; the public
// `_chain` factory wraps them into a `kinematic_chain` and the `_static`
// factory wraps them into a `static_chain<Scalar, Joints...>` whose joint-tag
// pack matches the omega direction of each screw axis. The pair keeps
// downstream consumers (FK, random_reachable_target, analytical solvers) from
// having to synthesize geometry inline.
// ===========================================================================

namespace detail
{

// --- Synthetic planar 2R fixture (analytical_solver_2r geometry) ---
template <typename Scalar>
struct planar_2r_geometry
{
    static constexpr Scalar l1 = Scalar(0.5);
    static constexpr Scalar l2 = Scalar(0.4);

    [[nodiscard]] static std::array<cartan::screw_axis<Scalar>, 2> axes()
    {
        using vec3 = cartan::vector3<Scalar>;
        return {
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(1), Scalar(0)),
                vec3(Scalar(0), Scalar(0), Scalar(0))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(1), Scalar(0)),
                vec3(l1, Scalar(0), Scalar(0)))};
    }

    [[nodiscard]] static cartan::se3<Scalar> home()
    {
        return cartan::se3<Scalar>(
            cartan::so3<Scalar>::identity(),
            cartan::vector3<Scalar>(l1 + l2, Scalar(0), Scalar(0)));
    }

    [[nodiscard]] static std::array<cartan::joint_limits<Scalar>, 2> limits()
    {
        cartan::joint_limits<Scalar> lim{
            -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
        return {lim, lim};
    }
};

// --- Synthetic spatial 3R fixture (analytical_solver_3r ZYZ geometry) ---
template <typename Scalar>
struct spatial_3r_geometry
{
    static constexpr Scalar link_offset = Scalar(0.5);
    static constexpr Scalar ee_offset = Scalar(0.3);

    [[nodiscard]] static std::array<cartan::screw_axis<Scalar>, 3> axes()
    {
        using vec3 = cartan::vector3<Scalar>;
        return {
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(0), Scalar(1)),
                vec3(Scalar(0), Scalar(0), Scalar(0))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(1), Scalar(0)),
                vec3(Scalar(0), Scalar(0), Scalar(0))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(0), Scalar(1)),
                vec3(link_offset, Scalar(0), Scalar(0)))};
    }

    [[nodiscard]] static cartan::se3<Scalar> home()
    {
        return cartan::se3<Scalar>(
            cartan::so3<Scalar>::identity(),
            cartan::vector3<Scalar>(link_offset + ee_offset, Scalar(0), Scalar(0)));
    }

    [[nodiscard]] static std::array<cartan::joint_limits<Scalar>, 3> limits()
    {
        cartan::joint_limits<Scalar> lim{
            -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
        return {lim, lim, lim};
    }
};

// --- ABB IRB 120 (Pieper-anchored variant of make_abb_irb120_chain) ---
//
// Wrist-decoupling note: this static_chain geometry deliberately diverges from
// the paired kinematic_chain `make_abb_irb120_chain` factory. The kinematic
// variant keeps the original IRB120 link offsets (wrist axes 4-5 anchored at
// z = 0.560, axis 6 at z = 0.862). The static variant below anchors all three
// wrist axes (4, 5, 6) at the axis-6 wrist-center point (0, 0, 0.862) so that
// pieper_6r_solver::find_wrist_intersection succeeds. The home pose is
// preserved (end-effector at (0, 0, 0.934)) and joint axis directions match
// the kinematic chain, so the two factories agree on FK at the home
// configuration; they diverge for non-zero joint angles. This asymmetry is
// what permits closed-form Pieper-decoupled solves on the static side while
// keeping FK-walked target generation and matched iterative cells running on
// the unmodified source geometry.
template <typename Scalar>
struct abb_irb120_geometry
{
    [[nodiscard]] static std::array<cartan::screw_axis<Scalar>, 6> axes()
    {
        using vec3 = cartan::vector3<Scalar>;
        return {
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(0), Scalar(1)),
                vec3(Scalar(0), Scalar(0), Scalar(0))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(1), Scalar(0)),
                vec3(Scalar(0), Scalar(0), Scalar(0.290))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(1), Scalar(0)),
                vec3(Scalar(0), Scalar(0), Scalar(0.560))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(1), Scalar(0), Scalar(0)),
                vec3(Scalar(0), Scalar(0), Scalar(0.862))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(1), Scalar(0)),
                vec3(Scalar(0), Scalar(0), Scalar(0.862))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(1), Scalar(0), Scalar(0)),
                vec3(Scalar(0), Scalar(0), Scalar(0.862)))};
    }

    [[nodiscard]] static cartan::se3<Scalar> home()
    {
        return cartan::se3<Scalar>(
            cartan::so3<Scalar>::identity(),
            cartan::vector3<Scalar>(Scalar(0), Scalar(0), Scalar(0.934)));
    }

    [[nodiscard]] static std::array<cartan::joint_limits<Scalar>, 6> limits()
    {
        cartan::joint_limits<Scalar> lim{
            -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
        return {lim, lim, lim, lim, lim, lim};
    }
};

// --- KUKA KR 6 R900 SIXX (matches make_kr6_sixx_chain screw axes) ---
template <typename Scalar>
struct kr6_sixx_geometry
{
    [[nodiscard]] static std::array<cartan::screw_axis<Scalar>, 6> axes()
    {
        using vec3 = cartan::vector3<Scalar>;
        return {
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(0), Scalar(1)),
                vec3(Scalar(0), Scalar(0), Scalar(0))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(1), Scalar(0)),
                vec3(Scalar(0), Scalar(0), Scalar(0.400))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(1), Scalar(0)),
                vec3(Scalar(0.455), Scalar(0), Scalar(0.400))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(1), Scalar(0), Scalar(0)),
                vec3(Scalar(0.875), Scalar(0), Scalar(0.400))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(0), Scalar(1), Scalar(0)),
                vec3(Scalar(0.875), Scalar(0), Scalar(0.400))),
            cartan::screw_axis<Scalar>::revolute(
                vec3(Scalar(1), Scalar(0), Scalar(0)),
                vec3(Scalar(0.935), Scalar(0), Scalar(0.400)))};
    }

    [[nodiscard]] static cartan::se3<Scalar> home()
    {
        return cartan::se3<Scalar>(
            cartan::so3<Scalar>::identity(),
            cartan::vector3<Scalar>(Scalar(0.935), Scalar(0), Scalar(0.400)));
    }

    [[nodiscard]] static std::array<cartan::joint_limits<Scalar>, 6> limits()
    {
        cartan::joint_limits<Scalar> lim{
            -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
        return {lim, lim, lim, lim, lim, lim};
    }
};

}

// --- Synthetic planar 2R factory pair ---
template <typename Scalar>
auto make_planar_2r_chain() -> cartan::kinematic_chain<Scalar, 2>
{
    auto axes = detail::planar_2r_geometry<Scalar>::axes();
    auto limits = detail::planar_2r_geometry<Scalar>::limits();
    return cartan::kinematic_chain<Scalar, 2>(
        detail::planar_2r_geometry<Scalar>::home(),
        {axes[0], axes[1]},
        {limits[0], limits[1]});
}

template <typename Scalar>
auto make_planar_2r_static()
    -> cartan::static_chain<Scalar, cartan::revolute_y, cartan::revolute_y>
{
    return cartan::static_chain<Scalar, cartan::revolute_y, cartan::revolute_y>(
        detail::planar_2r_geometry<Scalar>::home(),
        detail::planar_2r_geometry<Scalar>::axes(),
        detail::planar_2r_geometry<Scalar>::limits());
}

// --- Synthetic spatial 3R factory pair ---
template <typename Scalar>
auto make_spatial_3r_chain() -> cartan::kinematic_chain<Scalar, 3>
{
    auto axes = detail::spatial_3r_geometry<Scalar>::axes();
    auto limits = detail::spatial_3r_geometry<Scalar>::limits();
    return cartan::kinematic_chain<Scalar, 3>(
        detail::spatial_3r_geometry<Scalar>::home(),
        {axes[0], axes[1], axes[2]},
        {limits[0], limits[1], limits[2]});
}

template <typename Scalar>
auto make_spatial_3r_static()
    -> cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z>
{
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z>(
        detail::spatial_3r_geometry<Scalar>::home(),
        detail::spatial_3r_geometry<Scalar>::axes(),
        detail::spatial_3r_geometry<Scalar>::limits());
}

// --- ABB IRB 120 static_chain factory (paired with make_abb_irb120_chain) ---
template <typename Scalar>
auto make_abb_irb120_static()
    -> cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>
{
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        detail::abb_irb120_geometry<Scalar>::home(),
        detail::abb_irb120_geometry<Scalar>::axes(),
        detail::abb_irb120_geometry<Scalar>::limits());
}

// --- KUKA KR 6 R900 SIXX static_chain factory (paired with make_kr6_sixx_chain) ---
template <typename Scalar>
auto make_kr6_sixx_static()
    -> cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>
{
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        detail::kr6_sixx_geometry<Scalar>::home(),
        detail::kr6_sixx_geometry<Scalar>::axes(),
        detail::kr6_sixx_geometry<Scalar>::limits());
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

// ===========================================================================
// Synthetic Cartanbot fixture (paired with tests/fixtures/urdf/cartanbot.urdf)
// ===========================================================================

/// Pedagogical 6-DOF synthetic chain matching the always-on cartanbot.urdf
/// fixture; revolute, prismatic, and continuous joints exercise every URDF
/// code path the parser supports. Two fixed joints in the URDF (a sensor
/// mount between joint5 and joint6, and a tool offset after joint6) fold
/// into joint6's screw-axis origin and the home pose translation
/// respectively, leaving exactly six mobile joints in the chain.
template <typename Scalar = double>
auto make_cartanbot_chain() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    using vec3 = cartan::vector3<Scalar>;

    // Mobile joint axes (in the base frame at zero configuration).
    // Cumulative joint positions, threading through the parent-child URDF tree
    // (xyz origins relative to the parent link), compose to:
    //   joint1: (0, 0, 0.10)
    //   joint2: (0, 0, 0.30)        = 0.10 + 0.20
    //   joint3: (0, 0, 0.50)        = 0.30 + 0.20 (prismatic, direction only)
    //   joint4: (0, 0, 0.85)        = 0.50 + 0.35
    //   joint5: (0, 0, 1.20)        = 0.85 + 0.35 (continuous wrist roll)
    //   sensor_offset fixed (0, 0.05, 0) shifts the frame to (0, 0.05, 1.20)
    //   joint6: (0, 0.05, 1.35)     = sensor frame + (0, 0, 0.15)
    //   tool_offset fixed (0, 0, 0.05) folds into the home translation
    //   tool0:  (0, 0.05, 1.40)
    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.10)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.30)));
    auto s3 = cartan::screw_axis<Scalar>::prismatic(
        vec3(Scalar(0), Scalar(0), Scalar(1)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.85)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(1.20)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0.05), Scalar(1.35)));

    vec3 home_trans(Scalar(0), Scalar(0.05), Scalar(1.40));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::identity(), home_trans);

    cartan::joint_limits<Scalar> rev_lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};
    cartan::joint_limits<Scalar> prismatic_lim{Scalar(0), Scalar(0.20)};
    cartan::joint_limits<Scalar> continuous_lim{
        -std::numeric_limits<Scalar>::infinity(),
        +std::numeric_limits<Scalar>::infinity()};

    auto chain_static = cartan::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {rev_lim, rev_lim, prismatic_lim, rev_lim, continuous_lim, rev_lim});
    return chain_static.to_dynamic();
}

// ===========================================================================
// Extended real-world fixtures (CARTAN_URDF_EXTENDED_TESTS opt-in)
// ===========================================================================

#ifdef CARTAN_URDF_EXTENDED_TESTS

// The five extended factories below mirror the vendored real-world URDFs
// under tests/fixtures/urdf/extended/. Their screw axes were derived by
// walking each URDF's joint tree from base_link to the unique tool leaf,
// composing the per-joint <origin rpy/> rotations into the cumulative
// base-frame and folding fixed joints into the surrounding accumulator
// (the same procedure cartan::urdf::build_chain executes). They therefore
// reproduce numerical noise inherited from the upstream xacro arithmetic
// (e.g. the 2e-10 axis components on the UR variants that come from
// xacro's exact half-pi handling). Keeping the noise in the factories
// matches what the parser produces exactly so the 1e-12 parity gate holds.

/// Hand-coded ground-truth chain matching the vendored
/// tests/fixtures/urdf/extended/ur3e.urdf.
template <typename Scalar = double>
auto make_ur3e_chain_extended() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    using vec3 = cartan::vector3<Scalar>;
    using mat3 = cartan::matrix3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1.0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.15185)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0), Scalar(0), Scalar(0.15185)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0.24355), Scalar(0), Scalar(0.15185)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0.45675), Scalar(0.13105), Scalar(0.151849999973121)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-4.10207e-10), Scalar(-1.0)),
        vec3(Scalar(0.45675), Scalar(0.131049999964989), Scalar(0.066499999973121)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0.45675), Scalar(0.223149999964989), Scalar(0.066499999954231)));

    mat3 R_home;
    R_home << Scalar(-1.0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(2.05103e-10), Scalar(1.0),
              Scalar(0), Scalar(1.0), Scalar(-2.05103e-10);
    vec3 p_home(Scalar(0.45675), Scalar(0.223149999964989), Scalar(0.066499999954231));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::from_matrix(R_home).value(), p_home);

    cartan::joint_limits<Scalar> lim{
        -Scalar(2) * std::numbers::pi_v<Scalar>,
        +Scalar(2) * std::numbers::pi_v<Scalar>};

    auto chain_static = cartan::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});
    return chain_static.to_dynamic();
}

/// Hand-coded ground-truth chain matching the vendored
/// tests/fixtures/urdf/extended/ur5e.urdf.
template <typename Scalar = double>
auto make_ur5e_chain_extended() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    using vec3 = cartan::vector3<Scalar>;
    using mat3 = cartan::matrix3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1.0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.1625)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0), Scalar(0), Scalar(0.1625)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0.425), Scalar(0), Scalar(0.1625)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0.8172), Scalar(0.1333), Scalar(0.16249999997266)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-4.10207e-10), Scalar(-1.0)),
        vec3(Scalar(0.8172), Scalar(0.133299999959102), Scalar(0.06279999997266)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0.8172), Scalar(0.232899999959102), Scalar(0.062799999952231)));

    mat3 R_home;
    R_home << Scalar(-1.0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(2.05103e-10), Scalar(1.0),
              Scalar(0), Scalar(1.0), Scalar(-2.05103e-10);
    vec3 p_home(Scalar(0.8172), Scalar(0.232899999959102), Scalar(0.062799999952231));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::from_matrix(R_home).value(), p_home);

    cartan::joint_limits<Scalar> lim{
        -Scalar(2) * std::numbers::pi_v<Scalar>,
        +Scalar(2) * std::numbers::pi_v<Scalar>};

    auto chain_static = cartan::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});
    return chain_static.to_dynamic();
}

/// Hand-coded ground-truth chain matching the vendored
/// tests/fixtures/urdf/extended/ur10.urdf.
template <typename Scalar = double>
auto make_ur10_chain_extended() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    using vec3 = cartan::vector3<Scalar>;
    using mat3 = cartan::matrix3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1.0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.1273)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0), Scalar(0), Scalar(0.1273)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0.612), Scalar(0), Scalar(0.1273)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(1.1843), Scalar(0.163941), Scalar(0.127299999966375)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-4.10207e-10), Scalar(-1.0)),
        vec3(Scalar(1.1843), Scalar(0.163940999952539), Scalar(0.011599999966375)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(1.1843), Scalar(0.256140999952539), Scalar(0.011599999947465)));

    mat3 R_home;
    R_home << Scalar(-1.0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(2.05103e-10), Scalar(1.0),
              Scalar(0), Scalar(1.0), Scalar(-2.05103e-10);
    vec3 p_home(Scalar(1.1843), Scalar(0.256140999952539), Scalar(0.011599999947465));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::from_matrix(R_home).value(), p_home);

    cartan::joint_limits<Scalar> lim{
        -Scalar(2) * std::numbers::pi_v<Scalar>,
        +Scalar(2) * std::numbers::pi_v<Scalar>};

    auto chain_static = cartan::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});
    return chain_static.to_dynamic();
}

/// Hand-coded ground-truth chain matching the vendored
/// tests/fixtures/urdf/extended/ur16.urdf.
template <typename Scalar = double>
auto make_ur16_chain_extended() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    using vec3 = cartan::vector3<Scalar>;
    using mat3 = cartan::matrix3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1.0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.1807)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0), Scalar(0), Scalar(0.1807)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0.4784), Scalar(0), Scalar(0.1807)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0.8384), Scalar(0.17415), Scalar(0.180699999964281)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-4.10207e-10), Scalar(-1.0)),
        vec3(Scalar(0.8384), Scalar(0.174149999950837), Scalar(0.060849999964281)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(-2.05103e-10)),
        vec3(Scalar(0.8384), Scalar(0.290699999950837), Scalar(0.060849999940376)));

    mat3 R_home;
    R_home << Scalar(-1.0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(2.05103e-10), Scalar(1.0),
              Scalar(0), Scalar(1.0), Scalar(-2.05103e-10);
    vec3 p_home(Scalar(0.8384), Scalar(0.290699999950837), Scalar(0.060849999940376));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::from_matrix(R_home).value(), p_home);

    cartan::joint_limits<Scalar> lim{
        -Scalar(2) * std::numbers::pi_v<Scalar>,
        +Scalar(2) * std::numbers::pi_v<Scalar>};

    auto chain_static = cartan::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});
    return chain_static.to_dynamic();
}

/// Hand-coded ground-truth chain matching the vendored
/// tests/fixtures/urdf/extended/iiwa14.urdf.
template <typename Scalar = double>
auto make_iiwa14_chain_extended() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    using vec3 = cartan::vector3<Scalar>;
    using mat3 = cartan::matrix3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1.0)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(0)),
        vec3(Scalar(-0.00043624), Scalar(0), Scalar(0.36)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1.0)),
        vec3(Scalar(-0.00043624), Scalar(0), Scalar(0.36)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-1.0), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.78)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1.0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.78)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1.0), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(1.18)));
    auto s7 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1.0)),
        vec3(Scalar(0), Scalar(0), Scalar(1.18)));

    mat3 R_home;
    R_home << Scalar(1.0), Scalar(0), Scalar(0),
              Scalar(0), Scalar(1.0), Scalar(0),
              Scalar(0), Scalar(0), Scalar(1.0);
    vec3 p_home(Scalar(0), Scalar(0), Scalar(1.306));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::from_matrix(R_home).value(), p_home);

    cartan::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>,
        +std::numbers::pi_v<Scalar>};

    auto chain_static = cartan::kinematic_chain<Scalar, 7>(
        home,
        {s1, s2, s3, s4, s5, s6, s7},
        {lim, lim, lim, lim, lim, lim, lim});
    return chain_static.to_dynamic();
}

/// Hand-coded ground-truth chain matching the vendored
/// tests/fixtures/urdf/extended/panda.urdf. The screw axes were derived by
/// walking the vendored URDF's joint tree from panda_link0 to the
/// panda_link8 flange, composing the per-joint rpy rotations into the
/// cumulative base frame and folding the panda_joint8 trailing fixed offset
/// into the home pose (the same procedure cartan::urdf::build_chain executes
/// after the multi-fixed-leaf disambiguation that skips the seven
/// `panda_link{N}_sc` self-collision wrapper sublinks). The chain therefore
/// reproduces the float64 noise inherited from xacro's exact half-pi
/// arithmetic verbatim so the 1e-12 parity gate holds. The per-joint limits
/// preserve the asymmetric ranges declared in the vendored URDF (joint_4 and
/// joint_6 each have one-sided bounds on the Panda hardware).
template <typename Scalar = double>
auto make_panda_chain_extended() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    using vec3 = cartan::vector3<Scalar>;
    using mat3 = cartan::matrix3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.33300000000000002)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(6.123233995736766e-17)),
        vec3(Scalar(0), Scalar(0), Scalar(0.33300000000000002)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(-1.9349419426528181e-17), Scalar(0.64900000000000002)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-1), Scalar(6.123233995736766e-17)),
        vec3(Scalar(0.082500000000000004), Scalar(-1.9349419426528181e-17), Scalar(0.64900000000000002)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(4.1637991171010009e-18), Scalar(1.0329999999999999)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-1), Scalar(6.123233995736766e-17)),
        vec3(Scalar(0), Scalar(4.1637991171010009e-18), Scalar(1.0329999999999999)));
    auto s7 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-1.2246467991473532e-16), Scalar(-1)),
        vec3(Scalar(0.087999999999999995), Scalar(4.1637991171010009e-18), Scalar(1.0329999999999999)));

    mat3 R_home;
    R_home << Scalar(1), Scalar(0), Scalar(0),
              Scalar(0), Scalar(-1), Scalar(-1.2246467991473532e-16),
              Scalar(0), Scalar(1.2246467991473532e-16), Scalar(-1);
    vec3 p_home(Scalar(0.087999999999999995), Scalar(-8.9399216337756786e-18), Scalar(0.92599999999999993));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::from_matrix(R_home).value(), p_home);

    // Per-joint limits as declared in the vendored URDF.
    cartan::joint_limits<Scalar> lim1{Scalar(-2.8973), Scalar(2.8973)};
    cartan::joint_limits<Scalar> lim2{Scalar(-1.7628), Scalar(1.7628)};
    cartan::joint_limits<Scalar> lim3{Scalar(-2.8973), Scalar(2.8973)};
    cartan::joint_limits<Scalar> lim4{Scalar(-3.0718), Scalar(-0.0698)};
    cartan::joint_limits<Scalar> lim5{Scalar(-2.8973), Scalar(2.8973)};
    cartan::joint_limits<Scalar> lim6{Scalar(-0.0175), Scalar(3.7525)};
    cartan::joint_limits<Scalar> lim7{Scalar(-2.8973), Scalar(2.8973)};

    auto chain_static = cartan::kinematic_chain<Scalar, 7>(
        home,
        {s1, s2, s3, s4, s5, s6, s7},
        {lim1, lim2, lim3, lim4, lim5, lim6, lim7});
    return chain_static.to_dynamic();
}

/// Hand-coded ground-truth chain matching the vendored
/// tests/fixtures/urdf/extended/iiwa7.urdf. The screw axes were derived by
/// walking the URDF's joint tree from the world frame through the
/// world_iiwa_joint fixed pass-through, the seven iiwa_joint_{1..7} mobile
/// joints, and the trailing iiwa_joint_ee fixed offset (45 mm flange) to
/// iiwa_link_ee, composing the per-joint rpy rotations into the cumulative
/// base frame. The chain reproduces the float64 noise inherited from xacro's
/// exact half-pi arithmetic verbatim so the 1e-12 parity gate holds. All
/// seven mobile joints are about z in the local link frame; the cumulative
/// rpy stack rotates them in alternating y/z patterns when projected back to
/// the base frame.
template <typename Scalar = double>
auto make_lbr_iiwa7_chain_extended() -> cartan::kinematic_chain<Scalar, cartan::dynamic>
{
    using vec3 = cartan::vector3<Scalar>;
    using mat3 = cartan::matrix3<Scalar>;

    auto s1 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.14999999999999999)));
    auto s2 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(1.2246467991473532e-16), Scalar(1), Scalar(6.123233995736766e-17)),
        vec3(Scalar(0), Scalar(0), Scalar(0.33999999999999997)));
    auto s3 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(-1.2246467991473532e-16), Scalar(1.2325951644078309e-32), Scalar(1)),
        vec3(Scalar(-1.5747477717949504e-33), Scalar(-1.2858791391047208e-17), Scalar(0.54999999999999993)));
    auto s4 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(-1.2246467991473532e-16), Scalar(-1), Scalar(6.123233995736766e-17)),
        vec3(Scalar(-2.3268289183799715e-17), Scalar(-1.2858791391047205e-17), Scalar(0.73999999999999999)));
    auto s5 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(-1.2246467991473532e-16), Scalar(1.2246467991473535e-16), Scalar(1)),
        vec3(Scalar(-4.8985871965894131e-17), Scalar(6.1629758220391547e-33), Scalar(0.94999999999999996)));
    auto s6 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(2.4492935982947064e-16), Scalar(1), Scalar(-6.1232339957367636e-17)),
        vec3(Scalar(-8.7121373291342709e-17), Scalar(-0.060699999999999976), Scalar(1.1399999999999999)));
    auto s7 = cartan::screw_axis<Scalar>::revolute(
        vec3(Scalar(-1.2246467991473537e-16), Scalar(2.4651903288156619e-32), Scalar(1)),
        vec3(Scalar(-8.2173800222787403e-17), Scalar(2.7755575615628914e-17), Scalar(1.2209999999999999)));

    mat3 R_home;
    R_home << Scalar(1), Scalar(3.6739403974420594e-16), Scalar(-1.2246467991473537e-16),
              Scalar(-3.6739403974420594e-16), Scalar(1), Scalar(2.4651903288156619e-32),
              Scalar(1.2246467991473532e-16), Scalar(3.6977854932234928e-32), Scalar(1);
    vec3 p_home(Scalar(-8.7684710818950493e-17), Scalar(2.7755575615628914e-17), Scalar(1.2659999999999998));
    auto home = cartan::se3<Scalar>(cartan::so3<Scalar>::from_matrix(R_home).value(), p_home);

    // Per-joint limits as declared in the vendored URDF (symmetric ranges,
    // joints 1/3/5 at +/-170 deg, joints 2/4/6 at +/-120 deg, joint 7 at
    // +/-175 deg, expressed in radians).
    cartan::joint_limits<Scalar> lim_170{
        Scalar(-2.9670597283903604), Scalar(+2.9670597283903604)};
    cartan::joint_limits<Scalar> lim_120{
        Scalar(-2.0943951023931953), Scalar(+2.0943951023931953)};
    cartan::joint_limits<Scalar> lim_175{
        Scalar(-3.0543261909900763), Scalar(+3.0543261909900763)};

    auto chain_static = cartan::kinematic_chain<Scalar, 7>(
        home,
        {s1, s2, s3, s4, s5, s6, s7},
        {lim_170, lim_120, lim_170, lim_120, lim_170, lim_120, lim_175});
    return chain_static.to_dynamic();
}

#endif

}

#endif
