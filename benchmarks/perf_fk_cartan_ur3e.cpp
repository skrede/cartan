#include "benchmark_utils.h"

#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>
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
        auto r = cartan::forward_kinematics(kc, q);
        accum += r.end_effector.translation()(0);
    }
    std::printf("cartan UR3e FK done, accum=%g\n", accum);
    return 0;
}
