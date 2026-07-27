#include <cartan/serial/ik/basic_ik_runner.h>

#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/policy/limits_policy.h>

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include "../support/joint_limits_helpers.h"
#include "../fixtures/chain_factories.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <type_traits>

namespace spp = cartan;

TEST_CASE("clamp_limits clamps each q(i) to bounds", "[ik][limits]")
{
    auto lim = spp::testing::limits(-1.0, 1.0);
    using chain_type = spp::kinematic_chain<double, 3>;
    typename chain_type::limits_storage limits = {lim, lim, lim};

    Eigen::Vector3d q;
    q << -2.0, 0.5, 3.0;

    spp::clamp_limits::enforce<spp::kinematic_chain<double, 3>>(q, limits);

    REQUIRE(q(0) == Catch::Approx(-1.0));
    REQUIRE(q(1) == Catch::Approx(0.5));
    REQUIRE(q(2) == Catch::Approx(1.0));
}

TEST_CASE("no_limits returns q unchanged", "[ik][limits]")
{
    auto lim = spp::testing::limits(-1.0, 1.0);
    using chain_type = spp::kinematic_chain<double, 3>;
    typename chain_type::limits_storage limits = {lim, lim, lim};

    Eigen::Vector3d q;
    q << -2.0, 0.5, 3.0;
    Eigen::Vector3d q_orig = q;

    spp::no_limits::enforce<spp::kinematic_chain<double, 3>>(q, limits);

    REQUIRE(q(0) == Catch::Approx(q_orig(0)));
    REQUIRE(q(1) == Catch::Approx(q_orig(1)));
    REQUIRE(q(2) == Catch::Approx(q_orig(2)));
}

TEST_CASE("basic_ik_solver with dls_solve_policy and clamp_limits compiles", "[ik][solver]")
{
    using solver_type = spp::basic_ik_runner<spp::dls<spp::kinematic_chain<double, 6>>>;
    solver_type solver;
    static_assert(std::is_default_constructible_v<solver_type>);
}

// A refused setup has to reach the caller as its own typed reason, so both
// cases below drive a runner rather than asserting that the reason is
// representable: an assertion that assigns a reason and reads it back holds
// just as well with the validation deleted.
//
// The length case needs a dynamic chain -- a fixed chain's position_type makes
// the wrong length unrepresentable at the call, which is the stronger guard and
// the reason only the dynamic form can reach the runtime check.
TEST_CASE("a refused setup surfaces through solve() as its typed reason", "[ik][solver]")
{
    STATIC_REQUIRE(spp::ik_status::dimension_mismatch != spp::ik_status::non_finite_input);
    STATIC_REQUIRE(spp::ik_failure::dimension_mismatch != spp::ik_failure::non_finite_input);

    auto fixed = spp::fixtures::make_ur3e_chain<double>();
    auto target = spp::se3<double>::identity();
    spp::convergence_criteria<double> criteria;

    SECTION("a joint vector of the wrong length")
    {
        auto chain = fixed.to_dynamic();
        spp::basic_ik_runner<spp::dls<spp::kinematic_chain<double, spp::dynamic>>> runner;
        Eigen::VectorXd q0 = Eigen::VectorXd::Zero(chain.num_joints() - 1);

        runner.setup(chain, target, q0, criteria);
        auto result = runner.solve();

        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().reason == spp::ik_failure::dimension_mismatch);
    }

    SECTION("a nonfinite joint value")
    {
        spp::basic_ik_runner<spp::dls<spp::kinematic_chain<double, 6>>> runner;
        Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
        q0(3) = std::numeric_limits<double>::quiet_NaN();

        runner.setup(fixed, target, q0, criteria);
        auto result = runner.solve();

        REQUIRE_FALSE(result.has_value());
        CHECK(result.error().reason == spp::ik_failure::non_finite_input);
    }
}
