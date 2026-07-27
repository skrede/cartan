#include "six_joint_chain.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

// One case, one executable: AddressSanitizer terminates the process, so a case
// sharing a binary with other assertions takes every later case down with it.
//
// The joint-velocity vector is consumed by the Jacobian product rather than by
// the accumulation loop, so its length cases are a memory-error surface distinct
// from the joint-position ones. The scalar is selectable because the product's
// evaluator differs between the two: measured, an over-long vector faults in
// float and is silently truncated in double.
template <typename Scalar>
static int run_probe(int size)
{
    auto chain = cartan::testing::make_six_joint_dynamic_chain<Scalar>();
    const Eigen::VectorX<Scalar> q =
        Eigen::VectorX<Scalar>::Constant(chain.num_joints(), Scalar(0.1));
    const Eigen::VectorX<Scalar> dq = Eigen::VectorX<Scalar>::Constant(size, Scalar(0.2));

    const cartan::vector6<Scalar> twist =
        cartan::end_effector_velocity_unchecked(chain, q, dq);
    std::printf("joint-velocity vector of size %d produced a twist of norm %f\n",
        size, static_cast<double>(twist.norm()));
    return 0;
}

int main(int argc, char** argv)
{
    const int size = (argc > 1) ? std::atoi(argv[1]) : 5;
    const bool single_precision = (argc > 2) && std::strcmp(argv[2], "float") == 0;
    return single_precision ? run_probe<float>(size) : run_probe<double>(size);
}
