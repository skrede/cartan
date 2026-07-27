#include "boundary_fixtures.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

// One case, one executable: AddressSanitizer terminates the process, so a case
// sharing a binary with other assertions takes every later case down with it.
//
// The result's joint count is an argument so the same binary can be invoked
// with a result the chain's own size, which is what the assertion mechanism's
// own controls need.
template <typename Scalar>
static int run_probe(int result_joints)
{
    auto chain = cartan::fixtures::make_six_joint_dynamic_chain<Scalar>();
    auto source = cartan::fixtures::make_dynamic_chain<Scalar>(result_joints);
    const auto fk = cartan::forward_kinematics_unchecked(
        source, cartan::fixtures::joint_vector(result_joints, Scalar(0.2)));

    const auto J = cartan::space_jacobian_unchecked(chain, fk);
    std::printf("a result holding %d intermediates produced a Jacobian of norm %f\n",
        result_joints, static_cast<double>(J.norm()));
    return 0;
}

int main(int argc, char** argv)
{
    const int result_joints = (argc > 1) ? std::atoi(argv[1]) : 3;
    const bool single_precision = (argc > 2) && std::strcmp(argv[2], "float") == 0;
    return single_precision ? run_probe<float>(result_joints) : run_probe<double>(result_joints);
}
