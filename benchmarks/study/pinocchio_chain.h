#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_PINOCCHIO_CHAIN_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_PINOCCHIO_CHAIN_H

/// @file pinocchio_chain.h
/// @brief The peer library's model of the feasible set's own chain.
///
/// The model is built from the screw axes the feasible set carries, so the two
/// participants are solving one robot rather than two descriptions of one.

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/joint/joint-revolute-unaligned.hpp>

#include <Eigen/Dense>

#include <string>
#include <utility>
#include <stdexcept>

namespace cartan::bench
{

struct pinocchio_model
{
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex ee_frame;

    explicit pinocchio_model(pinocchio::Model built)
        : model(std::move(built))
        , data(model)
        , ee_frame(0)
    {
    }
};

inline pinocchio::SE3 to_pinocchio(const cartan::se3<double>& pose)
{
    return pinocchio::SE3(pose.rotation().matrix(), pose.translation());
}

namespace detail
{

/// A screw axis carries its line in space; pinocchio wants each joint placed
/// relative to the previous one, so the point on the axis travels along with
/// the parent index as the chain is walked.
struct joint_placement
{
    pinocchio::JointIndex parent;
    Eigen::Vector3d point;
};

inline joint_placement add_revolute_joint(
    pinocchio::Model& model,
    const joint_placement& previous,
    const cartan::screw_axis<double>& screw,
    int index)
{
    if (!screw.is_revolute())
    {
        throw std::runtime_error("the peer model builder carries revolute joints only");
    }
    const Eigen::Vector3d point = screw.omega().cross(screw.v());
    pinocchio::SE3 placement = pinocchio::SE3::Identity();
    placement.translation() = point - previous.point;
    return {model.addJoint(previous.parent,
                pinocchio::JointModelRevoluteUnaligned(screw.omega()), placement,
                "j" + std::to_string(index)),
        point};
}

}

template <int N>
pinocchio_model build_pinocchio_model(
    const cartan::kinematic_chain<double, N>& chain, const std::string& name)
{
    pinocchio::Model model;
    model.name = name;
    detail::joint_placement placed{0, Eigen::Vector3d::Zero()};
    for (int i = 0; i < chain.num_joints(); ++i)
    {
        placed = detail::add_revolute_joint(model, placed, chain.axis(i), i);
    }

    pinocchio::SE3 tool = pinocchio::SE3::Identity();
    tool.rotation() = chain.home().rotation().matrix();
    tool.translation() = chain.home().translation() - placed.point;

    pinocchio_model built(std::move(model));
    built.ee_frame = built.model.addFrame(
        pinocchio::Frame("ee", placed.parent, tool, pinocchio::OP_FRAME));
    built.data = pinocchio::Data(built.model);
    return built;
}

}

#endif
