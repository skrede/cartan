#include "benchmark_utils.h"

#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/fk/forward_kinematics_matrix.h>
#include <cartan/serial/fk/jacobian_matrix.h>
#include <cartan/serial/fk/detail/axis_specializations.h>

#include <cstdio>
#include <random>

int main(int argc, char**)
{
    auto kc = cartan::benchmarks::make_ur3e_chain<double>();
    std::mt19937 rng(42);
    auto q = cartan::benchmarks::random_joint_config(kc, rng);

    const long iters = (argc > 1) ? 1 : 50'000'000;
    double accum = 0.0;
    for (long i = 0; i < iters; ++i)
    {
        auto fkm = cartan::forward_kinematics_matrix(kc, q);
        auto J = cartan::space_jacobian(kc, fkm);
        accum += fkm.end_effector.p(0) + J(0, 0);
    }
    std::printf("cartan UR3e FK+Jac (matrix) done, accum=%g\n", accum);
    return 0;
}
