// The advertised closed-form solvers, each at the joint count its own geometry
// admits: two for the planar form, three for the spatial form, six for the two
// six-joint forms. The joint-count axis the rest of the matrix sweeps does not
// reach this family -- a closed form does not compile at a count its derivation
// is not written for, and repeating it at a count the geometry does admit would
// drive the same instantiation twice.
//
// The returned result is inspected rather than discarded. A solver that answered
// with an empty solution set, or with a branch carrying a NaN, would be
// indistinguishable from a working one to a drive that only asked whether the
// call compiled.

#include "../support/kinematics_helpers.h"

#include "../fixtures/opw_chains.h"
#include "../fixtures/chain_factories.h"

#include <cartan/analytical.h>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <utility>
#include <source_location>

namespace spp = cartan;

namespace
{

/// The unwrapping form, not the aborting one: a refused factory here should fail
/// its own case rather than take the other fifteen down with it.
template <typename T, typename E>
T made(spp::expected<T, E> factory,
       const std::source_location& where = std::source_location::current())
{
    return spp::testing::checked(std::move(factory), "analytical factory", where);
}

template <typename Chain>
Eigen::Vector<typename Chain::scalar_type, Chain::joints> posed(const Chain& chain)
{
    using scalar_type = typename Chain::scalar_type;
    using position_type = Eigen::Vector<scalar_type, Chain::joints>;

    position_type q = position_type::Zero(chain.num_joints());
    for (int i = 0; i < chain.num_joints(); ++i)
        q(i) = scalar_type(0.3) * scalar_type(i + 1);
    return q;
}

template <typename Chain>
spp::se3<typename Chain::scalar_type> reachable(const Chain& chain)
{
    return spp::testing::fk_at(chain, posed(chain)).end_effector;
}

/// Three separable ways for a solver that answers with nothing usable to be told
/// apart from one that works: the whole solve reported a failure, it reported
/// success with no branch in it, or a reported branch carries a NaN.
template <typename Result>
void inspect(const Result& solved)
{
    REQUIRE(solved.has_value());
    CHECK(solved->count > 0);
    CHECK(static_cast<std::size_t>(solved->count) <= solved->solutions.size());
    CHECK(solved->solutions.front().allFinite());
}

template <typename Scalar>
void drive_subproblem_forms()
{
    const auto planar = spp::fixtures::make_planar_2r_static<Scalar>();
    inspect(made(spp::planar_2r_solver<decltype(planar)>::make(planar))
        .solve(reachable(planar)));

    const auto spatial = spp::fixtures::make_spatial_3r_static<Scalar>();
    inspect(made(spp::spatial_3r_solver<decltype(spatial)>::make(spatial))
        .solve(reachable(spatial)));

    const auto pieper = spp::fixtures::make_abb_irb120_static<Scalar>();
    inspect(made(spp::pieper_6r_solver<decltype(pieper)>::make(pieper))
        .solve(reachable(pieper)));
}

/// The two verification variants are two advertised forms, not one: the raw one
/// compiles a different body. The wrapping solver is driven through both of its
/// solve overloads, which differ in the reference the unwrap moves toward.
template <typename Scalar>
void drive_ortho_parallel_forms()
{
    using chain_type = decltype(spp::fixtures::make_kr6_r900_opw_chain<Scalar>());

    const chain_type chain = spp::fixtures::make_kr6_r900_opw_chain<Scalar>();
    const spp::opw_parameters<Scalar> params =
        spp::fixtures::kr6_r900_opw_parameters<Scalar>();
    const spp::se3<Scalar> target = reachable(chain);

    const auto verified = made(spp::opw_6r_solver<chain_type>::make(chain, params));
    inspect(verified.solve(target));
    inspect(made(spp::opw_6r_solver<chain_type, spp::opw_raw>::make(chain, params))
        .solve(target));

    const spp::unwrapped_solver wrapping(verified);
    inspect(wrapping.solve(target));
    inspect(wrapping.solve(target, posed(chain)));
}

}

TEST_CASE("every advertised analytical form solves and reports at double",
    "[analytical][instantiation]")
{
    drive_subproblem_forms<double>();
    drive_ortho_parallel_forms<double>();
}

TEST_CASE("every advertised analytical form solves and reports at float",
    "[analytical][instantiation]")
{
    drive_subproblem_forms<float>();
    drive_ortho_parallel_forms<float>();
}
