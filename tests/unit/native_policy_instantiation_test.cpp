// The native solve policies over the runtime-sized chain: both scalar types,
// the fixed and the dynamic form, and three, six and seven joints.
//
// The joint-count axis is here because some liabilities are properties of the
// instance rather than of the type. A decomposition whose extent is fixed at
// compile time answers for the joint count it was sized from and aborts at
// another, and one driven only on six-joint fixtures never reaches it.
//
// The stepped drives assert that a combination instantiated and stepped. The
// completion drives run the whole loop from a seed away from the target, so the
// run enters the step body instead of answering at its seed, and they assert
// that it terminated with a status: an under-actuated three-joint chain
// legitimately fails to reach a general pose.

#include "../support/instantiation_drive.h"

#include "../fixtures/chain_factories.h"

#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/solver/lbfgsb.h>
#include <cartan/serial/ik/solver/projected_lm.h>
#include <cartan/serial/ik/solver/newton_raphson.h>

#include <cartan/lie/se3.h>

#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>

#include <type_traits>

namespace spp = cartan;

namespace
{

using chain6 = spp::kinematic_chain<double, 6>;

template <typename Chain>
void drive_natives(const Chain& chain, const spp::se3<typename Chain::scalar_type>& target)
{
    spp::testing::drive_stepped<spp::lm<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::lbfgsb<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::projected_lm<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::dls<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::newton_raphson<Chain>>(chain, target);
}

template <typename Chain>
void step_natives(const Chain& chain)
{
    drive_natives(chain, spp::testing::reachable_target(chain));
}

template <typename Scalar>
void step_every_shape()
{
    step_natives(spp::fixtures::make_3r_planar_chain<Scalar>());
    step_natives(spp::fixtures::make_ur3e_chain<Scalar>());
    step_natives(spp::fixtures::make_lbr_med14_chain<Scalar>());
    step_natives(spp::fixtures::make_3r_planar_chain<Scalar>().to_dynamic());
    step_natives(spp::fixtures::make_ur3e_chain<Scalar>().to_dynamic());
    step_natives(spp::fixtures::make_lbr_med14_chain<Scalar>().to_dynamic());
}

template <typename Chain>
void complete_natives(const Chain& chain)
{
    const auto target = spp::testing::distant_target(chain);

    spp::testing::drive_to_completion<spp::lm<Chain>>(chain, target);
    spp::testing::drive_to_completion<spp::lbfgsb<Chain>>(chain, target);
    spp::testing::drive_to_completion<spp::projected_lm<Chain>>(chain, target);
    spp::testing::drive_to_completion<spp::dls<Chain>>(chain, target);
    spp::testing::drive_to_completion<spp::newton_raphson<Chain>>(chain, target);
}

template <typename Scalar>
void complete_every_shape()
{
    complete_natives(spp::fixtures::make_3r_planar_chain<Scalar>());
    complete_natives(spp::fixtures::make_ur3e_chain<Scalar>());
    complete_natives(spp::fixtures::make_3r_planar_chain<Scalar>().to_dynamic());
    complete_natives(spp::fixtures::make_ur3e_chain<Scalar>().to_dynamic());
}

}

/// lm and lbfgsb are the published spellings and each is a plain synonym at its
/// default limits policy, so asserting the identity resolves the alias without
/// building a second copy of a type the drives already generate.
static_assert(std::is_same_v<spp::lm<chain6>, spp::builtin_lm<chain6>>);
static_assert(std::is_same_v<spp::lbfgsb<chain6>, spp::builtin_lbfgsb<chain6>>);

TEST_CASE("every native solve policy instantiates over a runtime-sized chain at double",
    "[kinematic_chain][instantiation]")
{
    step_every_shape<double>();
}

TEST_CASE("every native solve policy instantiates over a runtime-sized chain at float",
    "[kinematic_chain][instantiation]")
{
    step_every_shape<float>();
}

TEST_CASE("every native solve policy terminates a full run at double",
    "[kinematic_chain][instantiation]")
{
    complete_every_shape<double>();
}

TEST_CASE("every native solve policy terminates a full run at float",
    "[kinematic_chain][instantiation]")
{
    complete_every_shape<float>();
}
