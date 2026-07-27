#include <cartan/serial/ik/basic_ik_runner.h>

#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/policy/limits_policy.h>

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include "../support/joint_limits_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <utility>
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

// A setup refused for shape or finiteness has to be expressible on both sides
// of the runner: as a status a policy can latch and as a reason the error the
// caller receives can carry. Naming one without the other would let a refusal
// latch with nothing to report, or be reportable with nothing to latch it.
TEST_CASE("a refused setup is expressible as a status and as a failure reason", "[ik][solver]")
{
    using runner_type = spp::basic_ik_runner<spp::dls<spp::kinematic_chain<double, 6>>>;
    using error_type = typename decltype(std::declval<runner_type&>().solve())::error_type;

    static_assert(std::is_same_v<decltype(error_type{}.reason), spp::ik_failure>);

    STATIC_REQUIRE(spp::ik_status::dimension_mismatch != spp::ik_status::non_finite_input);
    STATIC_REQUIRE(spp::ik_failure::dimension_mismatch != spp::ik_failure::non_finite_input);

    error_type shape_error;
    shape_error.reason = spp::ik_failure::dimension_mismatch;
    REQUIRE(shape_error.reason == spp::ik_failure::dimension_mismatch);

    error_type finite_error;
    finite_error.reason = spp::ik_failure::non_finite_input;
    REQUIRE(finite_error.reason == spp::ik_failure::non_finite_input);
}
