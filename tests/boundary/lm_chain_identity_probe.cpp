#include "boundary_fixtures.h"

#include <cartan/serial/ik/solver/lm.h>

#include <cstdio>
#include <cstring>

// One case, one executable: the swapped chain stops the process, so a case
// sharing a binary with other assertions takes every later case down with it.
//
// The case argument lets the same binary step the chain setup() latched, which
// is what the assertion mechanism's own control needs.
//
// Both chains carry six joints, so the joint-count binding step() shares with
// every other solver accepts the swap and only the identity check can refuse it.
template <typename Scalar>
static int run_probe(bool mismatched)
{
    auto chain = cartan::fixtures::make_six_joint_dynamic_chain<Scalar>();
    auto twin = cartan::fixtures::make_six_joint_dynamic_chain<Scalar>();

    const Eigen::VectorX<Scalar> seed =
        cartan::fixtures::joint_vector<Scalar>(cartan::fixtures::six_joints, Scalar(0.15));
    const Eigen::VectorX<Scalar> posed =
        cartan::fixtures::joint_vector<Scalar>(cartan::fixtures::six_joints, Scalar(0.35));
    const cartan::se3<Scalar> target =
        cartan::forward_kinematics(chain, posed)->end_effector;

    cartan::builtin_lm<cartan::kinematic_chain<Scalar, cartan::dynamic>> solver;
    solver.setup(chain, target, seed, cartan::convergence_criteria<Scalar>{});

    const cartan::step_result<Scalar> stepped = solver.step(mismatched ? twin : chain, 4);
    std::printf("%d units consumed, reaching an error of %f\n",
        stepped.metrics.units_consumed, static_cast<double>(stepped.metrics.error_norm));
    return 0;
}

int main(int argc, char** argv)
{
    const bool mismatched = (argc > 1) && std::strcmp(argv[1], "mismatched") == 0;
    const bool single_precision = (argc > 2) && std::strcmp(argv[2], "float") == 0;
    return single_precision ? run_probe<float>(mismatched) : run_probe<double>(mismatched);
}
