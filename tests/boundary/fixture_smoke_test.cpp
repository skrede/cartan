// Every fixture factory in the tree, instantiated once per scalar and pinned.
//
// Construction alone asserts very little: it catches a tag/axis contradiction,
// a nonfinite input and a refused limit, and nothing else. A joint count and a
// finite end-effector are not functions of the geometry, so a link length or an
// axis point could be moved to a different robot's and still pass. What makes
// the geometry observable is the pair of assertions below -- the end-effector
// translation at a fixed non-zero configuration, pinned per factory, and exact
// axis-and-home equality across each static/dynamic twin whose two halves carry
// independent literals.
//
// The pinned values were generated from this tree, so they are a regression
// fingerprint: they do not independently validate any robot model against its
// datasheet, which is what the parity gates against the vendored URDFs do.

#include "fixture_pins.h"

#include "../fixtures/opw_chains.h"
#include "../fixtures/chain_factories.h"
#include "../fixtures/prismatic_chains.h"
#include "../fixtures/redundant_chains.h"
#include "../fixtures/analytical_chains.h"

#include "../test_utils.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>

namespace
{

namespace fx = cartan::fixtures;
using cartan::testing::expect_chain;
using cartan::testing::expect_same_geometry;

template <typename Scalar>
void expect_named_robots()
{
    expect_chain(fx::make_3r_planar_chain<Scalar>(), 3, 2.40228207, 1.64348958, 0.0);
    expect_chain(fx::make_ur3e_chain<Scalar>(), 6, -0.47851822, 0.06737238, 0.23203372);
    expect_chain(fx::make_lbr_med14_chain<Scalar>(), 7, 0.16182950, 0.02400607, 1.28099351);
    expect_chain(fx::make_kr6_sixx_chain<Scalar>(), 6, 0.78093209, 0.24705551, -0.01795753);
    expect_chain(fx::make_panda_chain<Scalar>(), 7, -0.03743870, 0.09847993, 1.11325881);
    expect_chain(fx::make_abb_irb120_chain<Scalar>(), 6, 0.29737752, 0.05038506, 0.83233722);
    expect_chain(fx::make_jaco2_chain<Scalar>(), 6, 0.31517893, 0.60952020, 0.25912373);
    expect_chain(fx::make_fetch_chain<Scalar>(), 7, 0.61409307, 0.25505179, 0.02763051);
    expect_chain(fx::make_baxter_chain<Scalar>(), 7, 0.80982335, 0.34395743, -0.21022271);
    expect_chain(fx::make_kuka_lwr4_chain<Scalar>(), 7, 0.14509493, 0.01573889, 1.15621153);
    expect_chain(fx::make_cartanbot_chain<Scalar>(), 6, 0.51952454, 0.21527853, 1.56450011);
}

template <typename Scalar>
void expect_static_chain_pairs()
{
    expect_chain(fx::make_planar_2r_chain<Scalar>(), 2, 0.80780249, 0.0, -0.37361709);
    expect_chain(fx::make_planar_2r_static<Scalar>(), 2, 0.80780249, 0.0, -0.37361709);
    expect_chain(fx::make_spatial_3r_chain<Scalar>(), 3, 0.69170570, 0.30677052, -0.23245647);
    expect_chain(fx::make_spatial_3r_static<Scalar>(), 3, 0.69170570, 0.30677052, -0.23245647);
    expect_chain(fx::make_abb_irb120_static<Scalar>(), 6, 0.29737752, 0.05038506, 0.83233722);
    expect_chain(fx::make_kr6_sixx_static<Scalar>(), 6, 0.78093209, 0.24705551, -0.01795753);
    expect_chain(fx::make_ur3e_chain_dynamic<Scalar>(), 6, -0.47851822, 0.06737238, 0.23203372);
    expect_chain(fx::make_lbr_med14_chain_dynamic<Scalar>(), 7,
        0.16182950, 0.02400607, 1.28099351);
}

/// The two pairs whose halves are built from independent literal blocks rather
/// than from one shared geometry helper. A length changed on either side alone
/// desynchronises the pair, which no single-chain assertion would notice.
template <typename Scalar>
void expect_twins_agree()
{
    expect_same_geometry(
        fx::make_abb_irb120_static<Scalar>(), fx::make_abb_irb120_chain<Scalar>());
    expect_same_geometry(
        fx::make_kr6_sixx_static<Scalar>(), fx::make_kr6_sixx_chain<Scalar>());
}

template <typename Scalar>
void expect_analytical_and_opw()
{
    expect_chain(fx::make_anti_parallel_wrist_puma<Scalar>(), 6,
        0.66238592, 0.20358008, 0.13493560);
    expect_chain(fx::make_offset_shoulder_puma<Scalar>(), 6,
        0.77522825, 0.29759034, 0.14479789);
    expect_chain(fx::make_puma_realistic_limits<Scalar>(), 6,
        0.63192778, 0.25326231, 0.14479789);
    expect_chain(fx::make_kr6_r900_opw_chain<Scalar>(), 6,
        0.83519849, -0.26567037, -0.00451945);

    auto near_spherical = fx::make_near_spherical_wrist_puma<Scalar>(Scalar(0.01));
    REQUIRE(near_spherical.has_value());
    expect_chain(*near_spherical, 6, 0.63381931, 0.25398559, 0.14699600);

    const auto params = fx::kr6_r900_opw_parameters<Scalar>();
    REQUIRE(params.c1 > Scalar(0));
}

template <typename Scalar>
void expect_prismatic_and_redundant()
{
    expect_chain(fx::make_rppr_signed_chain<Scalar>(), 4, 0.66873554, 0.20686414, -0.3);
    expect_chain(fx::make_rppr_signed_static<Scalar>(), 4, 0.66873554, 0.20686414, -0.3);
    expect_chain(fx::make_rppr_signed_dynamic<Scalar>(), 4, 0.66873554, 0.20686414, -0.3);
    expect_chain(fx::make_redundant_7r_chain_dynamic<Scalar>(), 7,
        0.16182950, 0.02400607, 1.28099351);
    expect_chain(fx::make_highdof_12r_chain_dynamic<Scalar>(), 12,
        0.52689995, 0.55342998, 0.72620315);
}

/// The utility header's factories are not fixtures, but their joint limits were
/// migrated by the same change and no other target in this lane builds them.
template <typename Scalar>
void expect_utility_robots()
{
    namespace ut = cartan::test;
    expect_chain(ut::make_lbr_iiwa_chain<Scalar>(), 7, 0.16182950, 0.02400607, 1.28099351);
    expect_chain(ut::make_ur3e_chain<Scalar>(), 6, -0.47851822, 0.06737238, 0.23203372);
    expect_chain(ut::make_kr6_sixx_chain<Scalar>(), 6, 0.78093209, 0.24705551, -0.01795753);
    expect_chain(ut::make_puma560_5dof_chain<Scalar>(), 5, 0.31268032, 0.68558843, 0.63804959);
    expect_chain(ut::make_4r_spatial_chain<Scalar>(), 4, 0.64352732, 0.26093357, 0.13406648);
    expect_chain(ut::make_3r_planar_chain<Scalar>(), 3, 2.40228207, 1.64348958, 0.0);
    expect_chain(ut::make_2r_planar_chain<Scalar>(), 2, 1.78067210, 0.86016268, 0.0);
    expect_chain(ut::make_1r_chain<Scalar>(), 1, 0.95533648, 0.29552020, 0.0);
}

/// The two forward-kinematics sites in the fixture header live in function
/// templates nothing else here instantiates, so without this they would have
/// been migrated and never compiled. The continuous-joint draw belongs with
/// them because that chain carries (-inf, +inf) position bounds, which the
/// standard forbids handing to a uniform distribution.
template <typename Scalar>
void expect_generated_targets()
{
    std::mt19937 rng(20260727U);
    const auto chain = fx::make_ur3e_chain<Scalar>();

    const auto target = fx::random_reachable_target(chain, rng);
    REQUIRE(target.matrix().allFinite());

    const auto q = fx::random_joint_config(chain, rng);
    const auto errors = fx::compute_pose_errors(chain, q, target);
    REQUIRE(std::isfinite(errors.first));
    REQUIRE(std::isfinite(errors.second));

    REQUIRE(fx::random_joint_config(fx::make_cartanbot_chain<Scalar>(), rng).allFinite());
}

template <typename Scalar>
void expect_every_factory()
{
    expect_named_robots<Scalar>();
    expect_static_chain_pairs<Scalar>();
    expect_twins_agree<Scalar>();
    expect_analytical_and_opw<Scalar>();
    expect_prismatic_and_redundant<Scalar>();
    expect_utility_robots<Scalar>();
    expect_generated_targets<Scalar>();
}

}

TEST_CASE("every fixture factory constructs and reproduces its pinned pose", "[fixtures]")
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
