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

    const auto fk = cartan::forward_kinematics_unchecked(chain, q);
    std::printf("empty joint vector produced a pose of norm %f\n",
        static_cast<double>(fk.end_effector.matrix().norm()));
    return 0;
}

int main(int argc, char** argv)
{
    const bool single_precision = (argc > 1) && std::strcmp(argv[1], "float") == 0;
    return single_precision ? run_probe<float>() : run_probe<double>();
}
