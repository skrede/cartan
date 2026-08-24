#ifdef CARTAN_HAS_ARGMIN

#include "../support/instantiation_drive.h"

#include "../fixtures/chain_factories.h"

#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/ik/solver/nw_sqp.h>
#include <cartan/serial/ik/solver/argmin_lbfgsb.h>
#include <cartan/serial/ik/wrapper/restart_wrapper.h>

#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>

namespace
{

using chain_type = cartan::kinematic_chain<double, cartan::dynamic>;

template <typename Policy>
using bound_runner = cartan::basic_ik_runner<
    cartan::restart_wrapper<chain_type, Policy, cartan::clamp_limits>>;

/// The target is the forward kinematics of the seed itself, so the backend
/// reports no iteration on the very first step and the configuration handed
/// back is whichever one setup() left behind.
template <typename Policy>
void check_reported_iterate_is_sized()
{
    const chain_type chain = cartan::fixtures::make_ur3e_chain_dynamic<double>();
    const cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 100, 200};

    bound_runner<Policy> runner;
    runner.setup(
        chain,
        cartan::testing::reachable_target(chain),
        cartan::testing::seeded_configuration(chain),
        criteria);
    auto outcome = runner.solve();

    if (outcome)
    {
        CHECK(outcome->solution.position.size() == chain.num_joints());
    }
    else
    {
        CHECK(outcome.error().last_q.size() == chain.num_joints());
    }
}

}

TEST_CASE("argmin_lbfgsb reports an iterate sized to the chain", "[ik][argmin][lbfgsb][dynamic]")
{
    check_reported_iterate_is_sized<
        cartan::argmin_lbfgsb<chain_type, cartan::clamp_limits>>();
}

TEST_CASE("nw_sqp reports an iterate sized to the chain", "[ik][argmin][nw_sqp][dynamic]")
{
    check_reported_iterate_is_sized<
        cartan::nw_sqp<chain_type, cartan::clamp_limits>>();
}

#endif
