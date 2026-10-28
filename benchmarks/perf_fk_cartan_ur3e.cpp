#include "benchmark_utils.h"

#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/fk/detail/axis_specializations.h>

#include <cstdio>
#include <random>

namespace
{

using ur3e_chain = cartan::kinematic_chain<double, 6>;
using ur3e_config = cartan::joint_state<double, 6>::position_type;

/// One checked evaluation establishing what the timed loop below then assumes,
/// so that loop measures forward kinematics and not the guard in front of it.
bool precondition_holds(const ur3e_chain& kc, const ur3e_config& q)
{
    auto fk = cartan::forward_kinematics(kc, q);
    if (!fk)
    {
        std::fprintf(stderr, "cartan UR3e FK refused the drawn configuration: %s\n",
            cartan::message(fk.error()));
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
    if (!precondition_holds(kc, q))
        return 1;

    const long iters = (argc > 1) ? 1 : 50'000'000;
    double accum = 0.0;
    for (long i = 0; i < iters; ++i)
    {
        auto r = cartan::forward_kinematics_unchecked(kc, q);
        accum += r.end_effector.translation()(0);
    }
    std::printf("cartan UR3e FK done, accum=%g\n", accum);
    return 0;
}
