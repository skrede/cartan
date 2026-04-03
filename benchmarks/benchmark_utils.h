#pragma once

/// @file benchmark_utils.h
/// @brief Standalone benchmark utilities: robot chain factories, random target
///        generation, error decomposition, and KDL frame conversion.
///
/// All robot geometries expressed as Product of Exponentials screw parameters.
/// This file is standalone from test infrastructure (D-14).
/// Chain factories cover ~10 robots in both cartan PoE and KDL representations (D-11, D-12).

#include "../profiling/chain_factories.h"

#include <kdl/chain.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/jntarray.hpp>

namespace cartan::benchmarks
{

// ===========================================================================
// KDL chain factories (D-12)
// ===========================================================================

namespace detail
{

/// Helper to create a KDL revolute segment with rotation about a given axis
/// and a frame offset to the next joint.
inline KDL::Segment make_kdl_revolute_z(
    const KDL::Vector& offset = KDL::Vector::Zero(),
    const KDL::Rotation& rot = KDL::Rotation::Identity())
{
    return KDL::Segment(
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(rot, offset));
}

inline KDL::Segment make_kdl_revolute_y(
    const KDL::Vector& offset = KDL::Vector::Zero(),
    const KDL::Rotation& rot = KDL::Rotation::Identity())
{
    return KDL::Segment(
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(rot, offset));
}

inline KDL::Segment make_kdl_revolute_x(
    const KDL::Vector& offset = KDL::Vector::Zero(),
    const KDL::Rotation& rot = KDL::Rotation::Identity())
{
    return KDL::Segment(
        KDL::Joint(KDL::Joint::RotX),
        KDL::Frame(rot, offset));
}

inline KDL::Segment make_kdl_revolute_neg_y(
    const KDL::Vector& offset = KDL::Vector::Zero())
{
    // RotY with frame that negates Y axis
    return KDL::Segment(
        KDL::Joint(KDL::Joint::RotY, -1.0),
        KDL::Frame(KDL::Rotation::Identity(), offset));
}

/// Set all joint limits to [-pi, pi].
inline void set_symmetric_pi_limits(KDL::JntArray& q_min, KDL::JntArray& q_max, unsigned int n)
{
    q_min.resize(n);
    q_max.resize(n);
    for (unsigned int i = 0; i < n; ++i)
    {
        q_min(i) = -M_PI;
        q_max(i) = M_PI;
    }
}

}

/// 3R planar chain as KDL (3-DOF). All joints about Z, unit link lengths.
inline KDL::Chain make_3r_planar_kdl_chain()
{
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(1, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(1, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(1, 0, 0))));
    return chain;
}

inline void make_3r_planar_kdl_limits(KDL::JntArray& q_min, KDL::JntArray& q_max)
{
    detail::set_symmetric_pi_limits(q_min, q_max, 3);
}

/// UR3e as KDL (6-DOF).
inline KDL::Chain make_ur3e_kdl_chain()
{
    KDL::Chain chain;
    // Frame offsets derived from PoE joint positions:
    // F_i = point_{i+1} - point_i
    // J1: RotZ at (0,0,0), F1 -> (0,0,h1)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.15185))));
    // J2: RotY at (0,0,h1), F2 -> (-l1,0,0)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(-0.24355, 0, 0))));
    // J3: RotY at (-l1,0,h1), F3 -> (-l2,0,0)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(-0.2132, 0, 0))));
    // J4: RotY at (-(l1+l2),0,h1), F4 -> (0,w1,-h1)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0.13105, -0.15185))));
    // J5: Rot-Z at (-(l1+l2),w1,0), F5 -> (0,-w1,-h2)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ, -1.0), KDL::Frame(KDL::Vector(0, -0.13105, -0.08535))));
    // J6: RotY at (-(l1+l2),0,-h2), F6 -> (0,w1+w2,h1)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0.22315, 0.15185))));
    return chain;
}

inline void make_ur3e_kdl_limits(KDL::JntArray& q_min, KDL::JntArray& q_max)
{
    detail::set_symmetric_pi_limits(q_min, q_max, 6);
}

/// LBR Med 14 as KDL (7-DOF).
inline KDL::Chain make_lbr_med14_kdl_chain()
{
    KDL::Chain chain;
    // J1: RotZ, link to (0, 0, 0.360)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.360))));
    // J2: RotY, link to (0, 0, 0) [axis change only]
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    // J3: RotZ, link to (0, 0, 0.420)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.420))));
    // J4: RotY (negative), link to (0, 0, 0)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY, -1.0), KDL::Frame(KDL::Vector(0, 0, 0))));
    // J5: RotZ, link to (0, 0, 0.400)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.400))));
    // J6: RotY, link to (0, 0, 0)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    // J7: RotZ, link to (0, 0, 0.126)
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.126))));
    return chain;
}

inline void make_lbr_med14_kdl_limits(KDL::JntArray& q_min, KDL::JntArray& q_max)
{
    detail::set_symmetric_pi_limits(q_min, q_max, 7);
}

/// KR 6 SIXX as KDL (6-DOF).
inline KDL::Chain make_kr6_sixx_kdl_chain()
{
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.400))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.455, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.420, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.060, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0, 0, 0))));
    return chain;
}

inline void make_kr6_sixx_kdl_limits(KDL::JntArray& q_min, KDL::JntArray& q_max)
{
    detail::set_symmetric_pi_limits(q_min, q_max, 6);
}

/// Panda as KDL (7-DOF).
inline KDL::Chain make_panda_kdl_chain()
{
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.333))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY, -1.0), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.0825, 0, 0.316))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(-0.0825, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.384))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.088, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.107))));
    return chain;
}

inline void make_panda_kdl_limits(KDL::JntArray& q_min, KDL::JntArray& q_max)
{
    detail::set_symmetric_pi_limits(q_min, q_max, 7);
}

/// ABB IRB 120 as KDL (6-DOF).
inline KDL::Chain make_abb_irb120_kdl_chain()
{
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.290))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0.270))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0, 0, 0.302))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0, 0, 0.072))));
    return chain;
}

inline void make_abb_irb120_kdl_limits(KDL::JntArray& q_min, KDL::JntArray& q_max)
{
    detail::set_symmetric_pi_limits(q_min, q_max, 6);
}

/// Kinova Jaco2 as KDL (6-DOF).
inline KDL::Chain make_jaco2_kdl_chain()
{
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.2755))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.410, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0.2073, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0, 0.0743, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0.0743, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0, 0.1687, 0))));
    return chain;
}

inline void make_jaco2_kdl_limits(KDL::JntArray& q_min, KDL::JntArray& q_max)
{
    detail::set_symmetric_pi_limits(q_min, q_max, 6);
}

/// Fetch arm as KDL (7-DOF).
inline KDL::Chain make_fetch_kdl_chain()
{
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.400))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.321, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.321, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.1385, 0, 0))));
    return chain;
}

inline void make_fetch_kdl_limits(KDL::JntArray& q_min, KDL::JntArray& q_max)
{
    detail::set_symmetric_pi_limits(q_min, q_max, 7);
}

/// Baxter single arm as KDL (7-DOF).
inline KDL::Chain make_baxter_kdl_chain()
{
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.069, 0, 0.2703))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.3644, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.3743, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.2295, 0, 0))));
    return chain;
}

inline void make_baxter_kdl_limits(KDL::JntArray& q_min, KDL::JntArray& q_max)
{
    detail::set_symmetric_pi_limits(q_min, q_max, 7);
}

/// KUKA LWR 4+ as KDL (7-DOF).
inline KDL::Chain make_kuka_lwr4_kdl_chain()
{
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.3105))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.4))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY, -1.0), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.39))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0, 0, 0.078))));
    return chain;
}

inline void make_kuka_lwr4_kdl_limits(KDL::JntArray& q_min, KDL::JntArray& q_max)
{
    detail::set_symmetric_pi_limits(q_min, q_max, 7);
}

// ===========================================================================
// KDL frame conversion
// ===========================================================================

/// Convert cartan SE(3) pose to KDL::Frame.
inline KDL::Frame se3_to_kdl_frame(const cartan::se3<double>& pose)
{
    auto R = pose.rotation().matrix();
    auto t = pose.translation();
    return KDL::Frame(
        KDL::Rotation(
            R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2)),
        KDL::Vector(t(0), t(1), t(2)));
}

}
