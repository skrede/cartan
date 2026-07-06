/// @file basic_ik_runner_test.cpp
/// @brief Lifetime-safety contract for basic_ik_runner: borrowed non-owning
///        chain reference, rejection of temporary chains at the call boundary,
///        and typed pre-setup errors instead of null dereferences.

#include "../fixtures/chain_factories.h"

#include <cartan/serial/ik/solvers.h>
#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/basic_ik_runner.h>

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <utility>

namespace spp = cartan;

namespace
{

using chain3 = spp::kinematic_chain<double, 3>;
using runner3 = spp::basic_ik_runner<spp::dls<chain3>>;

/// Compile-time probe: is setup() callable with a temporary (rvalue) chain?
/// A lifetime-safe runner must reject this so a borrowed reference cannot
/// bind-then-dangle. On the fixed runner the rvalue overload is deleted, so
/// the requires-expression evaluates to false.
template <typename Runner, typename Chain>
concept setup_accepts_rvalue_chain = requires(
    Runner r,
    Chain c,
    spp::se3<double> target,
    Eigen::Vector<double, 3> q0,
    spp::convergence_criteria<double> criteria)
{
    r.setup(std::move(c), target, q0, criteria);
};

}

// ===========================================================================
// Pre-setup use is a typed error, never a null dereference.
// ===========================================================================

TEST_CASE("basic_ik_runner solve() before setup() reports not_initialized",
    "[basic_ik_runner][lifetime]")
{
    runner3 runner;

    auto result = runner.solve();

    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error().reason == spp::ik_failure::not_initialized);
}

TEST_CASE("basic_ik_runner step() before setup() returns a terminal status",
    "[basic_ik_runner][lifetime]")
{
    runner3 runner;

    auto status = runner.step();

    REQUIRE(status == spp::ik_status::iteration_limit);
}

// ===========================================================================
// A temporary chain cannot bind to setup() (deleted rvalue overload).
// ===========================================================================

TEST_CASE("basic_ik_runner rejects a temporary chain at setup()",
    "[basic_ik_runner][lifetime]")
{
    STATIC_REQUIRE_FALSE(setup_accepts_rvalue_chain<runner3, chain3>);

    // An lvalue chain remains a valid setup argument.
    STATIC_REQUIRE(requires(
        runner3 r,
        chain3& c,
        spp::se3<double> target,
        Eigen::Vector<double, 3> q0,
        spp::convergence_criteria<double> criteria)
    {
        r.setup(c, target, q0, criteria);
    });
}

// ===========================================================================
// The borrowed reference drives a compile-time static_chain with no copy.
// ===========================================================================

TEST_CASE("basic_ik_runner borrows a static_chain and solves it",
    "[basic_ik_runner][lifetime][static_chain]")
{
    auto kc = spp::fixtures::make_3r_planar_chain<double>();
    auto sc = spp::static_chain<double,
        spp::revolute_z, spp::revolute_z, spp::revolute_z>(
        kc.home(), kc.axes(), kc.limits());

    Eigen::Vector<double, 3> q_known;
    q_known << 0.3, -0.5, 0.7;
    auto target = spp::forward_kinematics(sc, q_known).end_effector;

    Eigen::Vector<double, 3> q0 = Eigen::Vector<double, 3>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    spp::basic_ik_runner sc_solver{spp::speed_ik_runner<decltype(sc)>{}};
    sc_solver.setup(sc, target, q0, criteria);
    auto result = sc_solver.solve();

    REQUIRE(result.has_value());

    auto fk = spp::forward_kinematics(sc, result->solution.position);
    auto err = (fk.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}
