#include "benchmark_utils.h"

#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/fk/forward_kinematics_matrix.h>
#include <cartan/serial/fk/jacobian_matrix.h>
#include <cartan/serial/fk/detail/axis_specializations.h>

#include <cstdio>
#include <random>

namespace
{

using ur3e_chain = cartan::kinematic_chain<double, 6>;
using ur3e_config = cartan::joint_state<double, 6>::position_type;

/// One checked evaluation of each entry point, establishing what the timed loop
/// below then assumes, so that loop measures the kinematics and not the guards.
bool preconditions_hold(const ur3e_chain& kc, const ur3e_config& q)
{
    auto fkm = cartan::forward_kinematics_matrix(kc, q);
    if (!fkm)
    {
        std::fprintf(stderr, "cartan UR3e matrix FK refused the drawn configuration: %s\n",
            cartan::message(fkm.error()));
        return false;
    }
    auto J = cartan::space_jacobian(kc, *fkm);
    if (!J)
    {
        std::fprintf(stderr, "cartan UR3e matrix Jacobian refused that result: %s\n",
            cartan::message(J.error()));
        return false;
    }
    return true;
}

}

int main(int argc, char**)
{
    auto kc = cartan::fixtures::make_ur3e_chain<double>();
    std::mt19937 rng(42);
    auto q = cartan::fixtures::random_joint_config(kc, rng);
    if (!preconditions_hold(kc, q))
        return 1;

    const long iters = (argc > 1) ? 1 : 50'000'000;
    double accum = 0.0;
    for (long i = 0; i < iters; ++i)
    {
        auto fkm = cartan::forward_kinematics_matrix_unchecked(kc, q);
        auto J = cartan::space_jacobian_unchecked(kc, fkm);
        accum += fkm.end_effector.p(0) + J(0, 0);
    }
    std::printf("cartan UR3e FK+Jac (matrix) done, accum=%g\n", accum);
    return 0;
}
