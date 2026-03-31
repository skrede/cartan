#pragma once

/// @file test_utils.h
/// @brief Test utilities: scalar-typed epsilon and robot geometry factories.
///
/// Provides test_eps<Scalar> for tolerance calculations and factory functions
/// returning kinematic_chain<N, Scalar> for DOF 1-7, covering real robot
/// geometries (LBR iiwa, UR3e, KR 6 SIXX, PUMA 560) and synthetic chains.
///
/// All numeric literals use Scalar(...) casts to avoid implicit narrowing.
/// All geometries expressed as Product of Exponentials screw parameters.

#include <liepp/types.h>

#include <liepp/lie/se3.h>
#include <liepp/lie/so3.h>
#include <liepp/chain/screw_axis.h>
#include <liepp/chain/joint_limits.h>
#include <liepp/chain/kinematic_chain.h>

#include <limits>
#include <numbers>

namespace liepp::test
{

// ---------------------------------------------------------------------------
// test_eps: base epsilon for tolerance calculations
// ---------------------------------------------------------------------------

/// Base epsilon for tolerance calculations.
/// Each test decides its own multiplier based on operation sensitivity.
/// Example: Scalar(100) * test_eps<Scalar> for FK, Scalar(10000) for IK.
template <typename Scalar>
inline constexpr Scalar test_eps = std::numeric_limits<Scalar>::epsilon();

// ---------------------------------------------------------------------------
// Robot geometry factory functions
// ---------------------------------------------------------------------------

/// KUKA LBR iiwa 14 R820 (7-DOF).
/// Link lengths from LBR Med 14 R820 technical data.
/// Joint axis pattern: alternating z/y for 7-DOF LBR.
/// Reference: Lynch & Park, Modern Robotics, Ch. 4 (PoE form).
template <typename Scalar>
auto make_lbr_iiwa_chain() -> liepp::kinematic_chain<Scalar, 7>
{
    using vec3 = liepp::vector3<Scalar>;

    auto s1 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.360)));
    auto s3 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.360)));
    auto s4 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(-1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.780)));
    auto s5 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0.780)));
    auto s6 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(1.180)));
    auto s7 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(1.180)));

    vec3 home_trans(Scalar(0), Scalar(0), Scalar(1.306));
    auto home = liepp::se3<Scalar>(liepp::so3<Scalar>::identity(), home_trans);

    liepp::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return liepp::kinematic_chain<Scalar, 7>(
        home,
        {s1, s2, s3, s4, s5, s6, s7},
        {lim, lim, lim, lim, lim, lim, lim});
}

/// Universal Robots UR3e (6-DOF).
/// DH parameters from UR official documentation, converted to PoE.
/// At zero config the arm is extended vertically.
/// Reference: UR official DH parameters page.
template <typename Scalar>
auto make_ur3e_chain() -> liepp::kinematic_chain<Scalar, 6>
{
    using vec3 = liepp::vector3<Scalar>;

    auto s1 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.15185)));
    auto s3 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(-0.24355), Scalar(0), Scalar(0.15185)));
    auto s4 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(-0.45675), Scalar(0), Scalar(0.15185)));
    auto s5 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(-1)),
        vec3(Scalar(-0.45675), Scalar(0.13105), Scalar(0)));
    auto s6 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(-0.45675), Scalar(0), Scalar(-0.08535)));

    // Home M: end-effector position at zero config.
    // DH chain at theta=0: translation accumulates through link offsets.
    // Position: x = -(a2 + a3) = -(0.24355 + 0.2132) = -0.45675
    //           y = d4 + d6 = 0.13105 + 0.0921 = 0.22315
    //           z = d1 - d5 = 0.15185 - 0.08535 = 0.0665
    // Rotation: at zero config with UR DH, the end-effector frame has
    // accumulated alpha rotations: R = Rx(pi/2)*Rx(pi/2)*Rx(-pi/2) = Rx(pi/2)
    // For sweep testing purposes, we use identity rotation and the computed position.
    vec3 home_trans(Scalar(-0.45675), Scalar(0.22315), Scalar(0.0665));
    auto home = liepp::se3<Scalar>(liepp::so3<Scalar>::identity(), home_trans);

    liepp::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return liepp::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});
}

/// KUKA KR 6 R900 SIXX (6-DOF).
/// Dimensions estimated from working envelope drawing.
/// Zero configuration is forward-pointing (arm extended horizontally).
/// Reference: KUKA KR 6 SIXX working envelope drawing.
template <typename Scalar>
auto make_kr6_sixx_chain() -> liepp::kinematic_chain<Scalar, 6>
{
    using vec3 = liepp::vector3<Scalar>;

    // At zero config (forward-pointing):
    // J1: base rotation about z
    // J2: shoulder pitch about y at base height
    // J3: elbow pitch about y at shoulder + link2
    // J4: forearm roll about x along forearm
    // J5: wrist pitch about y at wrist
    // J6: flange roll about x at flange
    //
    // Dimensions (meters):
    //   base height: 0.400
    //   link 2 (shoulder to elbow): 0.455
    //   link 3 (elbow to wrist): 0.420
    //   wrist offset d4: 0.080
    //   flange offset d6: 0.060
    //
    // At zero config the arm points along +x:
    //   J2 is at (0, 0, 0.400)
    //   J3 is at (0.455, 0, 0.400)
    //   J4-J5-J6 wrist center at (0.875, 0, 0.400)
    //   End-effector at (0.935, 0, 0.400)

    auto s1 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.400)));
    auto s3 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.455), Scalar(0), Scalar(0.400)));
    auto s4 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.875), Scalar(0), Scalar(0.400)));
    auto s5 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.875), Scalar(0), Scalar(0.400)));
    auto s6 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.935), Scalar(0), Scalar(0.400)));

    vec3 home_trans(Scalar(0.935), Scalar(0), Scalar(0.400));
    auto home = liepp::se3<Scalar>(liepp::so3<Scalar>::identity(), home_trans);

    liepp::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return liepp::kinematic_chain<Scalar, 6>(
        home,
        {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});
}

/// PUMA 560 truncated to 5 DOF (first 5 joints).
/// DH parameters from published robotics literature (Corke).
/// Converted to PoE using Lynch & Park Ch. 4 methodology.
/// Reference: Peter Corke, Robotics Toolbox; Lynch & Park, Modern Robotics.
template <typename Scalar>
auto make_puma560_5dof_chain() -> liepp::kinematic_chain<Scalar, 5>
{
    using vec3 = liepp::vector3<Scalar>;

    // PUMA 560 DH parameters (first 5 joints):
    //   J1: a=0,      d=0.6718, alpha=pi/2
    //   J2: a=0.4318, d=0,      alpha=0
    //   J3: a=0.0203, d=0.15005,alpha=-pi/2
    //   J4: a=0,      d=0.4318, alpha=pi/2
    //   J5: a=0,      d=0,      alpha=-pi/2
    //
    // At zero config (all theta=0), working in space frame:
    // J1: about z at origin
    // J2: about y at (0, 0, 0.6718) -- after d1 offset along z,
    //     alpha=pi/2 rotates next frame but at theta=0 the joint axis is y
    // J3: about y at (0.4318, 0, 0.6718) -- after a2 along x
    // J4: about x at (0.4521, 0.15005, 0.6718) -- a3 along x, d3 along y,
    //     alpha=-pi/2 means next axis rotated
    // J5: about y at (0.4521, 0.15005+0.4318, 0.6718) -- d4 along y
    //     = (0.4521, 0.58185, 0.6718)

    auto s1 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.6718)));
    auto s3 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.4318), Scalar(0), Scalar(0.6718)));
    auto s4 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(1), Scalar(0), Scalar(0)),
        vec3(Scalar(0.4521), Scalar(0.15005), Scalar(0.6718)));
    auto s5 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.4521), Scalar(0.58185), Scalar(0.6718)));

    // Home M at zero config: end-effector at accumulated position
    vec3 home_trans(Scalar(0.4521), Scalar(0.58185), Scalar(0.6718));
    auto home = liepp::se3<Scalar>(liepp::so3<Scalar>::identity(), home_trans);

    liepp::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return liepp::kinematic_chain<Scalar, 5>(
        home,
        {s1, s2, s3, s4, s5},
        {lim, lim, lim, lim, lim});
}

/// Spatial 4R chain (SCARA-like pattern, 4-DOF).
/// Synthetic chain for DOF=4 sweep testing.
template <typename Scalar>
auto make_4r_spatial_chain() -> liepp::kinematic_chain<Scalar, 4>
{
    using vec3 = liepp::vector3<Scalar>;

    auto s1 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0), Scalar(0), Scalar(0.5)));
    auto s3 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(1), Scalar(0)),
        vec3(Scalar(0.3), Scalar(0), Scalar(0.5)));
    auto s4 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0.6), Scalar(0), Scalar(0.5)));

    vec3 home_trans(Scalar(0.8), Scalar(0), Scalar(0.5));
    auto home = liepp::se3<Scalar>(liepp::so3<Scalar>::identity(), home_trans);

    liepp::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return liepp::kinematic_chain<Scalar, 4>(
        home,
        {s1, s2, s3, s4},
        {lim, lim, lim, lim});
}

/// Planar 3R chain (3-DOF).
/// Three revolute joints about z, unit link lengths.
/// Classic textbook example.
template <typename Scalar>
auto make_3r_planar_chain() -> liepp::kinematic_chain<Scalar, 3>
{
    using vec3 = liepp::vector3<Scalar>;

    auto s1 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(1), Scalar(0), Scalar(0)));
    auto s3 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(2), Scalar(0), Scalar(0)));

    vec3 home_trans(Scalar(3), Scalar(0), Scalar(0));
    auto home = liepp::se3<Scalar>(liepp::so3<Scalar>::identity(), home_trans);

    liepp::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return liepp::kinematic_chain<Scalar, 3>(
        home,
        {s1, s2, s3},
        {lim, lim, lim});
}

/// Planar 2R chain (2-DOF).
/// Two revolute joints about z, unit link lengths.
template <typename Scalar>
auto make_2r_planar_chain() -> liepp::kinematic_chain<Scalar, 2>
{
    using vec3 = liepp::vector3<Scalar>;

    auto s1 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));
    auto s2 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(1), Scalar(0), Scalar(0)));

    vec3 home_trans(Scalar(2), Scalar(0), Scalar(0));
    auto home = liepp::se3<Scalar>(liepp::so3<Scalar>::identity(), home_trans);

    liepp::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return liepp::kinematic_chain<Scalar, 2>(
        home,
        {s1, s2},
        {lim, lim});
}

/// Single revolute joint (1-DOF).
/// Revolute about z at origin, link along x.
template <typename Scalar>
auto make_1r_chain() -> liepp::kinematic_chain<Scalar, 1>
{
    using vec3 = liepp::vector3<Scalar>;

    auto s1 = liepp::screw_axis<Scalar>::revolute(
        vec3(Scalar(0), Scalar(0), Scalar(1)),
        vec3(Scalar(0), Scalar(0), Scalar(0)));

    vec3 home_trans(Scalar(1), Scalar(0), Scalar(0));
    auto home = liepp::se3<Scalar>(liepp::so3<Scalar>::identity(), home_trans);

    liepp::joint_limits<Scalar> lim{
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>};

    return liepp::kinematic_chain<Scalar, 1>(
        home,
        {s1},
        {lim});
}

} // namespace liepp::test
