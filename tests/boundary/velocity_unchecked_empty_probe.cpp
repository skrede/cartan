#include "six_joint_chain.h"

#include <cstdio>

// One case, one executable: AddressSanitizer terminates the process, so a case
// sharing a binary with other assertions takes every later case down with it.
int main()
{
    auto chain = cartan::testing::make_six_joint_dynamic_chain<double>();
    const Eigen::VectorXd q;
    const Eigen::VectorXd dq = Eigen::VectorXd::Constant(chain.num_joints(), 0.2);

    const cartan::vector6<double> twist =
        cartan::end_effector_velocity_unchecked(chain, q, dq);
    std::printf("empty joint vector produced a twist of norm %f\n", twist.norm());
    return 0;
}
