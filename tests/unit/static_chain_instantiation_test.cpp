// The native solve policies, driven over the compile-time-tagged chain at every
// joint count and both scalar types.
//
// A class template that is only included has its dependent expressions parsed
// and not type-checked, and instantiating the class instantiates its member
// declarations rather than its member bodies, so an entry point nothing drives
// is one no compiler has checked.
//
// Each combination is driven through one setup() and one step() rather than to
// convergence: what is asserted here is that the combination instantiated and
// stepped, not that it solved, which the per-solver targets cover.

#include "../support/instantiation_drive.h"
#include "../support/static_chain_factories.h"

#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/solver/lbfgsb.h>
#include <cartan/serial/ik/solver/projected_lm.h>
#include <cartan/serial/ik/solver/newton_raphson.h>

#include <cartan/lie/se3.h>

#include <catch2/catch_test_macros.hpp>

#include <type_traits>

namespace spp = cartan;

namespace
{

using ur3e_static = spp::testing::static_ur3e_chain<double>;

template <typename Chain>
void drive_natives(const Chain& chain, const spp::se3<typename Chain::scalar_type>& target)
{
    spp::testing::drive_stepped<spp::lm<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::lbfgsb<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::projected_lm<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::dls<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::newton_raphson<Chain>>(chain, target);
}

}

/// lm and lbfgsb are the published spellings and each is a plain synonym at its
/// default limits policy, so asserting the identity resolves the alias without
/// building a second copy of a type the drives above already generate.
static_assert(std::is_same_v<spp::lm<ur3e_static>, spp::builtin_lm<ur3e_static>>);
static_assert(std::is_same_v<spp::lbfgsb<ur3e_static>, spp::builtin_lbfgsb<ur3e_static>>);

TEST_CASE("every native solve policy instantiates over a static chain at double",
    "[static_chain][instantiation]")
{
    const auto planar = spp::testing::make_3r_planar_static<double>();
    drive_natives(planar, spp::testing::reachable_target(planar));

    const auto ur3e = spp::testing::make_ur3e_static<double>();
    drive_natives(ur3e, spp::testing::reachable_target(ur3e));

    const auto med14 = spp::testing::make_lbr_med14_static<double>();
    drive_natives(med14, spp::testing::reachable_target(med14));
}

TEST_CASE("every native solve policy instantiates over a static chain at float",
    "[static_chain][instantiation]")
{
    const auto planar = spp::testing::make_3r_planar_static<float>();
    drive_natives(planar, spp::testing::reachable_target(planar));

    const auto ur3e = spp::testing::make_ur3e_static<float>();
    drive_natives(ur3e, spp::testing::reachable_target(ur3e));

    const auto med14 = spp::testing::make_lbr_med14_static<float>();
    drive_natives(med14, spp::testing::reachable_target(med14));
}
