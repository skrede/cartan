#include "boundary_fixtures.h"

#include <cstdio>
#include <cstring>

// One case, one executable: AddressSanitizer terminates the process, so a case
// sharing a binary with other assertions takes every later case down with it.
template <typename Scalar>
static int run_probe()
{
    auto chain = cartan::fixtures::make_six_joint_dynamic_chain<Scalar>();
    const Eigen::VectorX<Scalar> q;
    const Eigen::VectorX<Scalar> dq =
        cartan::fixtures::joint_vector(chain.num_joints(), Scalar(0.2));

    const cartan::vector6<Scalar> twist =
        cartan::end_effector_velocity_unchecked(chain, q, dq);
    std::printf("empty joint vector produced a twist of norm %f\n",
        static_cast<double>(twist.norm()));
    return 0;
}

int main(int argc, char** argv)
{
    const bool single_precision = (argc > 1) && std::strcmp(argv[1], "float") == 0;
    return single_precision ? run_probe<float>() : run_probe<double>();
}
