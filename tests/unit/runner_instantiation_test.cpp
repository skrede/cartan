// The advertised runner and wrapper forms over the runtime-sized chain: both
// scalar types, the fixed and the dynamic form, and three, six and seven joints.
//
// These forms do not share an entry-point shape, which is why the family is
// driven through more than one helper. A wrapper is a solve policy and takes a
// step; a runner takes a solve; the enumeration takes neither and answers in one
// call. A single drive over all of them does not compile.
//
// The enumeration is driven at a pose its seed reaches, because the accumulation
// and dedup that distinguish it run only once a restart has converged.

#include "../support/instantiation_drive.h"

#include "../fixtures/chain_factories.h"

#include <cartan/serial/ik/solvers.h>

#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/solver/exhaustive_ik_runner.h>

#include <cartan/serial/ik/wrapper/restart_wrapper.h>

#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>

namespace spp = cartan;

namespace
{

template <typename Chain>
using restart_lm = spp::restart_wrapper<Chain, spp::lm<Chain>>;

template <typename Chain>
using exhaustive_lm = spp::exhaustive_ik_runner<Chain, spp::lm<Chain>>;

template <typename Chain>
void drive_runners(const Chain& chain)
{
    const auto reached = spp::testing::reachable_target(chain);

    spp::testing::drive_stepped<restart_lm<Chain>>(chain, reached);
    spp::testing::drive_stepped<spp::speed_ik_runner<Chain>>(chain, reached);
    spp::testing::drive_stepped<spp::robust_ik_runner<Chain>>(chain, reached);
    spp::testing::drive_exhaustive<exhaustive_lm<Chain>>(chain, reached);
    spp::testing::drive_solved<spp::dual_ik_runner<Chain>>(
        chain, spp::testing::distant_target(chain));
}

template <typename Scalar>
void drive_every_shape()
{
    drive_runners(spp::fixtures::make_3r_planar_chain<Scalar>());
    drive_runners(spp::fixtures::make_ur3e_chain<Scalar>());
    drive_runners(spp::fixtures::make_lbr_med14_chain<Scalar>());
    drive_runners(spp::fixtures::make_3r_planar_chain<Scalar>().to_dynamic());
    drive_runners(spp::fixtures::make_ur3e_chain<Scalar>().to_dynamic());
    drive_runners(spp::fixtures::make_lbr_med14_chain<Scalar>().to_dynamic());
}

template <typename Chain>
void complete_runners(const Chain& chain)
{
    const auto target = spp::testing::distant_target(chain);

    spp::testing::drive_to_completion<restart_lm<Chain>>(chain, target);
    spp::testing::drive_to_completion<spp::speed_ik_runner<Chain>>(chain, target);
    spp::testing::drive_to_completion<spp::robust_ik_runner<Chain>>(chain, target);
}

template <typename Scalar>
void complete_every_shape()
{
    complete_runners(spp::fixtures::make_3r_planar_chain<Scalar>());
    complete_runners(spp::fixtures::make_ur3e_chain<Scalar>());
    complete_runners(spp::fixtures::make_3r_planar_chain<Scalar>().to_dynamic());
    complete_runners(spp::fixtures::make_ur3e_chain<Scalar>().to_dynamic());
}

}

TEST_CASE("every advertised runner form instantiates over a runtime-sized chain at double",
    "[kinematic_chain][instantiation]")
{
    drive_every_shape<double>();
}

TEST_CASE("every advertised runner form instantiates over a runtime-sized chain at float",
    "[kinematic_chain][instantiation]")
{
    drive_every_shape<float>();
}

TEST_CASE("the runner slice terminates a full run at double",
    "[kinematic_chain][instantiation]")
{
    complete_every_shape<double>();
}

TEST_CASE("the runner slice terminates a full run at float",
    "[kinematic_chain][instantiation]")
{
    complete_every_shape<float>();
}
