// Every backend-gated solve policy over the runtime-sized chain: both scalar
// types, the fixed and the dynamic form, and three, six and seven joints.
//
// The joint-count axis is here because some liabilities are properties of the
// instance rather than of the type, and the scalar axis because these policies
// convert to double internally: nothing else in the suite asks whether their
// float-chain forms are well-formed.
//
// A published alias earns a drive of its own where it binds template arguments
// no other instantiation supplies, since it can then be ill-formed while every
// default-argument instantiation of the same template compiles. An alias that
// only renames a default binding is resolved by asserting the identity, which
// costs no second copy of a type already driven.
//
// The non-default-convergence cases belong to the pre-existing backend
// instantiation target and are not repeated here; what this widens is the
// scalar and joint-count reach of the default forms.

#include "../support/instantiation_drive.h"

#include "../fixtures/chain_factories.h"

#include <cartan/serial/ik/solver/mma.h>
#include <cartan/serial/ik/solver/nw_sqp.h>
#include <cartan/serial/ik/solver/argmin_lm.h>
#include <cartan/serial/ik/solver/argmin_slsqp.h>
#include <cartan/serial/ik/solver/filter_slsqp.h>
#include <cartan/serial/ik/solver/argmin_bobyqa.h>
#include <cartan/serial/ik/solver/argmin_lbfgsb.h>
#include <cartan/serial/ik/solver/filter_nw_sqp.h>
#include <cartan/serial/ik/solver/argmin_projected_gn.h>
#include <cartan/serial/ik/solver/augmented_lagrangian.h>
#include <cartan/serial/ik/solver/argmin_projected_gradient_gn.h>

#include <cartan/lie/se3.h>

#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>

#include <type_traits>

namespace spp = cartan;

namespace
{

using chain6 = spp::kinematic_chain<double, 6>;

template <typename Chain>
void drive_backend(const Chain& chain, const spp::se3<typename Chain::scalar_type>& target)
{
    spp::testing::drive_stepped<spp::argmin_bobyqa<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::argmin_lbfgsb<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::argmin_lm<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::argmin_projected_gn<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::argmin_projected_gradient_gn<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::argmin_slsqp<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::argmin_slsqp_nlopt_compat<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::augmented_lagrangian<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::filter_nw_sqp<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::filter_slsqp<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::mma<Chain>>(chain, target);
    spp::testing::drive_stepped<spp::nw_sqp<Chain>>(chain, target);
}

template <typename Chain>
void step_backend(const Chain& chain)
{
    drive_backend(chain, spp::testing::reachable_target(chain));
}

template <typename Scalar>
void step_every_shape()
{
    step_backend(spp::fixtures::make_3r_planar_chain<Scalar>());
    step_backend(spp::fixtures::make_ur3e_chain<Scalar>());
    step_backend(spp::fixtures::make_lbr_med14_chain<Scalar>());
    step_backend(spp::fixtures::make_3r_planar_chain<Scalar>().to_dynamic());
    step_backend(spp::fixtures::make_ur3e_chain<Scalar>().to_dynamic());
    step_backend(spp::fixtures::make_lbr_med14_chain<Scalar>().to_dynamic());
}

}

/// argmin_slsqp_fast is documented as a synonym that survives only so existing
/// call sites resolve, so it renames a binding the drives above already
/// generate and the identity is what resolves it.
static_assert(std::is_same_v<spp::argmin_slsqp_fast<chain6>, spp::argmin_slsqp<chain6>>);

TEST_CASE("every backend-gated solve policy instantiates over a runtime-sized chain at double",
    "[argmin][kinematic_chain][instantiation]")
{
    step_every_shape<double>();
}

TEST_CASE("every backend-gated solve policy instantiates over a runtime-sized chain at float",
    "[argmin][kinematic_chain][instantiation]")
{
    step_every_shape<float>();
}
