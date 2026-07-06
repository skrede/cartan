#include <cartan/serial/ik/basic_ik_runner.h>

#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/policy/limits_policy.h>

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

namespace spp = cartan;

TEST_CASE("clamp_limits clamps each q(i) to bounds", "[ik][limits]")
{
    spp::joint_limits<double> lim{-1.0, 1.0};
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
    spp::joint_limits<double> lim{-1.0, 1.0};
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
