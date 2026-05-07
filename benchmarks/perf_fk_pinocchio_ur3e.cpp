#include "benchmark_utils.h"

#include <cartan/serial/chain/static_chain.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/joint/joint-revolute-unaligned.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <cstdio>
#include <random>

int main(int argc, char**)
{
    auto kc = cartan::benchmarks::make_ur3e_chain<double>();
    pinocchio::Model model;
    pinocchio::JointIndex parent = 0;
    Eigen::Vector3d prev = Eigen::Vector3d::Zero();
    for (int i = 0; i < kc.num_joints(); ++i)
    {
        const auto& s = kc.axis(i);
        Eigen::Vector3d omega = s.omega();
        Eigen::Vector3d v = s.v();
        Eigen::Vector3d q_perp = omega.cross(v);
        pinocchio::SE3 placement;
        placement.setIdentity();
        placement.translation() = q_perp - prev;
        parent = model.addJoint(parent,
            pinocchio::JointModelRevoluteUnaligned(omega),
            placement, "j" + std::to_string(i));
        prev = q_perp;
    }
    pinocchio::SE3 ee_placement;
    ee_placement.rotation() = kc.home().rotation().matrix();
    ee_placement.translation() = kc.home().translation() - prev;
    auto ee_frame = model.addFrame(pinocchio::Frame(
        "ee", parent, ee_placement, pinocchio::OP_FRAME));
    pinocchio::Data data(model);

    std::mt19937 rng(42);
    auto q_cartan = cartan::benchmarks::random_joint_config(kc, rng);
    Eigen::VectorXd q(kc.num_joints());
    for (int i = 0; i < kc.num_joints(); ++i) q(i) = q_cartan(i);

    const long iters = (argc > 1) ? 1 : 50'000'000;
    double accum = 0.0;
    for (long i = 0; i < iters; ++i)
    {
        pinocchio::framesForwardKinematics(model, data, q);
        accum += data.oMf[ee_frame].translation()(0);
    }
    std::printf("pinocchio UR3e FK done, accum=%g\n", accum);
    return 0;
}
