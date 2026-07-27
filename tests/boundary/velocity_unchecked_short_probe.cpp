#include "six_joint_chain.h"

#include <cstdio>
#include <cstdlib>

// One case, one executable: AddressSanitizer terminates the process, so a case
// sharing a binary with other assertions takes every later case down with it.
//
// The optional joint-count argument exists so the same binary can be invoked
// in bounds, which is what the tripwire's negative control needs.
int main(int argc, char** argv)
{
    auto chain = cartan::testing::make_six_joint_dynamic_chain<double>();
    const int size = (argc > 1) ? std::atoi(argv[1]) : chain.num_joints() - 1;

    const Eigen::VectorXd q = Eigen::VectorXd::Constant(size, 0.1);
    const Eigen::VectorXd dq = Eigen::VectorXd::Constant(chain.num_joints(), 0.2);

    const cartan::vector6<double> twist =
        cartan::end_effector_velocity_unchecked(chain, q, dq);
    std::printf("joint vector of size %d produced a twist of norm %f\n", size, twist.norm());
    return 0;
}
