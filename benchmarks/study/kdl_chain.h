#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_KDL_CHAIN_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_KDL_CHAIN_H

/// @file kdl_chain.h
/// @brief The comparator library's model of the feasible set's own chain.
///
/// Built from the screw axes the loaded description produced, so the
/// wall-clock-budgeted comparator solves the robot every other participant
/// solves. A hand-written second description of the same robot is the defect
/// this study exists to remove, and bounds derived from one chain do not fix a
/// geometry taken from another.
///
/// A revolute screw axis is a line in the base frame; KDL applies each joint's
/// rotation in its parent's frame, and a rotation about a fixed line composes
/// into exactly the product-of-exponentials the chain means. The home pose rides
/// on the last segment's tip.

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>

#include <Eigen/Dense>

#include <string>
#include <stdexcept>

namespace cartan::bench
{

inline KDL::Vector to_kdl(const Eigen::Vector3d& value)
{
    return KDL::Vector(value.x(), value.y(), value.z());
}

inline KDL::Frame to_kdl(const cartan::se3<double>& pose)
{
    const Eigen::Matrix3d rotation = pose.rotation().matrix();
    return KDL::Frame(
        KDL::Rotation(rotation(0, 0), rotation(0, 1), rotation(0, 2), rotation(1, 0),
            rotation(1, 1), rotation(1, 2), rotation(2, 0), rotation(2, 1), rotation(2, 2)),
        to_kdl(pose.translation()));
}

template <int N>
KDL::Chain build_kdl_chain(const cartan::kinematic_chain<double, N>& chain)
{
    KDL::Chain built;
    for (int i = 0; i < chain.num_joints(); ++i)
    {
        const auto& screw = chain.axis(i);
        if (!screw.is_revolute())
        {
            throw std::runtime_error("the comparator chain builder carries revolute joints only");
        }
        const Eigen::Vector3d point = screw.omega().cross(screw.v());
        const KDL::Frame tip =
            i + 1 == chain.num_joints() ? to_kdl(chain.home()) : KDL::Frame::Identity();
        built.addSegment(KDL::Segment(
            KDL::Joint("j" + std::to_string(i), to_kdl(point), to_kdl(screw.omega()),
                KDL::Joint::RotAxis),
            tip));
    }
    return built;
}

}

#endif
