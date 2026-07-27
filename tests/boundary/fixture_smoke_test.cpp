// Every fixture factory in the tree, instantiated once per scalar. Construction
// reaching the assertions at all is most of what this asserts: each factory now
// unwraps a checked construction inside its own body, so a disturbed axis
// literal or a limits value the factory refuses fails loudly there rather than
// surfacing as a wrong robot in whichever consumer happens to run first.

#include "../fixtures/opw_chains.h"
#include "../fixtures/chain_factories.h"
#include "../fixtures/prismatic_chains.h"
#include "../fixtures/redundant_chains.h"
#include "../fixtures/analytical_chains.h"

#include "../test_utils.h"

#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

namespace
{

namespace fx = cartan::fixtures;

template <typename Chain>
void expect_chain(const Chain& chain, int joints)
{
    using Scalar = typename Chain::scalar_type;
    REQUIRE(chain.num_joints() == joints);

    const Eigen::VectorX<Scalar> zero = Eigen::VectorX<Scalar>::Zero(joints);
    auto fk = cartan::forward_kinematics(chain, zero);
    REQUIRE(fk.has_value());
    REQUIRE(fk->end_effector.matrix().allFinite());
}

template <typename Scalar>
void expect_named_robots()
{
    expect_chain(fx::make_3r_planar_chain<Scalar>(), 3);
    expect_chain(fx::make_ur3e_chain<Scalar>(), 6);
    expect_chain(fx::make_lbr_med14_chain<Scalar>(), 7);
    expect_chain(fx::make_kr6_sixx_chain<Scalar>(), 6);
    expect_chain(fx::make_panda_chain<Scalar>(), 7);
    expect_chain(fx::make_abb_irb120_chain<Scalar>(), 6);
    expect_chain(fx::make_jaco2_chain<Scalar>(), 6);
    expect_chain(fx::make_fetch_chain<Scalar>(), 7);
    expect_chain(fx::make_baxter_chain<Scalar>(), 7);
    expect_chain(fx::make_kuka_lwr4_chain<Scalar>(), 7);
    expect_chain(fx::make_cartanbot_chain<Scalar>(), 6);
}

template <typename Scalar>
void expect_static_chain_pairs()
{
    expect_chain(fx::make_planar_2r_chain<Scalar>(), 2);
    expect_chain(fx::make_planar_2r_static<Scalar>(), 2);
    expect_chain(fx::make_spatial_3r_chain<Scalar>(), 3);
    expect_chain(fx::make_spatial_3r_static<Scalar>(), 3);
    expect_chain(fx::make_abb_irb120_static<Scalar>(), 6);
    expect_chain(fx::make_kr6_sixx_static<Scalar>(), 6);
    expect_chain(fx::make_ur3e_chain_dynamic<Scalar>(), 6);
    expect_chain(fx::make_lbr_med14_chain_dynamic<Scalar>(), 7);
}

template <typename Scalar>
void expect_analytical_and_opw()
{
    expect_chain(fx::make_anti_parallel_wrist_puma<Scalar>(), 6);
    expect_chain(fx::make_offset_shoulder_puma<Scalar>(), 6);
    expect_chain(fx::make_near_spherical_wrist_puma<Scalar>(Scalar(0.01)), 6);
    expect_chain(fx::make_puma_realistic_limits<Scalar>(), 6);
    expect_chain(fx::make_kr6_r900_opw_chain<Scalar>(), 6);

    const auto params = fx::kr6_r900_opw_parameters<Scalar>();
    REQUIRE(params.c1 > Scalar(0));
}

template <typename Scalar>
void expect_prismatic_and_redundant()
{
    expect_chain(fx::make_rppr_signed_chain<Scalar>(), 4);
    expect_chain(fx::make_rppr_signed_static<Scalar>(), 4);
    expect_chain(fx::make_rppr_signed_dynamic<Scalar>(), 4);
    expect_chain(fx::make_redundant_7r_chain_dynamic<Scalar>(), 7);
    expect_chain(fx::make_highdof_12r_chain_dynamic<Scalar>(), 12);
}

/// The utility header's factories are not fixtures, but their joint limits
/// were migrated by the same change and no other target in this lane builds
/// them.
template <typename Scalar>
void expect_utility_robots()
{
    namespace ut = cartan::test;
    expect_chain(ut::make_lbr_iiwa_chain<Scalar>(), 7);
    expect_chain(ut::make_ur3e_chain<Scalar>(), 6);
    expect_chain(ut::make_kr6_sixx_chain<Scalar>(), 6);
    expect_chain(ut::make_puma560_5dof_chain<Scalar>(), 5);
    expect_chain(ut::make_4r_spatial_chain<Scalar>(), 4);
    expect_chain(ut::make_3r_planar_chain<Scalar>(), 3);
    expect_chain(ut::make_2r_planar_chain<Scalar>(), 2);
    expect_chain(ut::make_1r_chain<Scalar>(), 1);
}

template <typename Scalar>
void expect_every_factory()
{
    expect_named_robots<Scalar>();
    expect_static_chain_pairs<Scalar>();
    expect_analytical_and_opw<Scalar>();
    expect_prismatic_and_redundant<Scalar>();
    expect_utility_robots<Scalar>();
}

}

TEST_CASE("every fixture factory constructs and evaluates at zero", "[fixtures]")
{
    SECTION("double")
    {
        expect_every_factory<double>();
    }

    SECTION("float")
    {
        expect_every_factory<float>();
    }
}
