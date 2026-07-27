#include "boundary_fixtures.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

// One case, one executable: AddressSanitizer terminates the process, so a case
// sharing a binary with other assertions takes every later case down with it.
//
// The length argument lets the same binary be invoked in bounds, which is what
// the assertion mechanism's own controls need.
template <typename Scalar>
static int run_probe(int size)
{
    auto chain = cartan::fixtures::make_six_joint_dynamic_chain<Scalar>();
    const Eigen::VectorX<Scalar> q = cartan::fixtures::joint_vector(size, Scalar(0.1));
    const Eigen::VectorX<Scalar> dq =
        cartan::fixtures::joint_vector(chain.num_joints(), Scalar(0.2));

    const cartan::vector6<Scalar> twist =
        cartan::end_effector_velocity_unchecked(chain, q, dq);
    std::printf("joint vector of size %d produced a twist of norm %f\n",
        size, static_cast<double>(twist.norm()));
    return 0;
}

int main(int argc, char** argv)
{
    const int size = (argc > 1) ? std::atoi(argv[1]) : 5;
    const bool single_precision = (argc > 2) && std::strcmp(argv[2], "float") == 0;
    return single_precision ? run_probe<float>(size) : run_probe<double>(size);
}
