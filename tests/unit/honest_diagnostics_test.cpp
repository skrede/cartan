#include "../support/joint_limits_helpers.h"

#include <cartan/serial/ik/ik_result.h>
#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/ik/detail/feasible_set.h>
#include <cartan/serial/ik/detail/setup_validation.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>

#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <cartan/serial/fk/jacobian.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/fk/singularity_analysis.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <numbers>

namespace spp = cartan;

using Catch::Approx;
using chain6 = spp::kinematic_chain<double, 6>;
using chain_dyn = spp::kinematic_chain<double, spp::dynamic>;
using vec6 = Eigen::Vector<double, 6>;

static spp::joint_limits<double> wide()
{
    return spp::testing::limits(-2 * std::numbers::pi, 2 * std::numbers::pi);
}

static chain6 make_ur5_like_chain()
{
    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.089});
    auto s3 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.425, 0, 0.089});
    auto s4 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, 0.089});
    auto s5 = spp::screw_axis<double>::revolute({0, 0, -1}, {0.817, 0.109, 0});
    auto s6 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, -0.006});

    spp::vector3<double> home_trans;
    home_trans << 0.817, 0.191, -0.006;
    auto home = spp::se3<double>(spp::so3<double>::identity(), home_trans);
    auto lim = wide();
    return chain6(home, {s1, s2, s3, s4, s5, s6}, {lim, lim, lim, lim, lim, lim});
}

/// Two revolute joints sharing one axis: the second can do nothing the first
/// cannot, so the Jacobian is rank deficient and exactly singular.
static chain_dyn make_duplicated_axis_chain()
{
    auto s = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    return chain_dyn(spp::se3<double>::identity(), {s, s}, {wide(), wide()});
}

// ============================================================================
// The measures agree with the decomposition they are read from
// ============================================================================

TEST_CASE("the singularity measures are read off one spectrum", "[ik][diagnostics]")
{
    auto chain = make_ur5_like_chain();
    vec6 q;
    q << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk = spp::forward_kinematics(chain, q);
    REQUIRE(fk.has_value());
    auto J = spp::body_jacobian_unchecked(chain, *fk);
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(
        J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& raw = svd.singularValues();

    auto sigma = spp::singular_values(chain, q);
    REQUIRE(sigma.size() == raw.size());
    CHECK((sigma - raw).norm() < 1e-15);

    REQUIRE(spp::manipulability(sigma));
    REQUIRE(spp::isotropy(sigma));
    REQUIRE(spp::condition_number(sigma));
    CHECK(*spp::manipulability(sigma) == Approx(raw.prod()));
    CHECK(*spp::isotropy(sigma) == Approx(raw(5) / raw(0)));
    CHECK(*spp::condition_number(sigma) == Approx(raw(0) / raw(5)));
    CHECK(*spp::isotropy(sigma) == Approx(1.0 / *spp::condition_number(sigma)));
}

// The characteristic length divides the three linear rows, so on a square
// Jacobian it multiplies the product of the singular values by L^-3 exactly.
TEST_CASE("the characteristic length still scales the linear rows", "[ik][diagnostics]")
{
    auto chain = make_ur5_like_chain();
    vec6 q;
    q << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto unit = spp::manipulability(spp::singular_values(chain, q, 1.0));
    REQUIRE(unit);

    for (double length : {0.5, 0.1, 0.01})
    {
        auto scaled = spp::manipulability(spp::singular_values(chain, q, length));
        REQUIRE(scaled);
        CHECK(*scaled == Approx(*unit / (length * length * length)).epsilon(1e-12));
    }
}

TEST_CASE("an exactly singular Jacobian reports an infinite condition number", "[ik][diagnostics]")
{
    auto chain = make_duplicated_axis_chain();
    Eigen::VectorXd q(2);
    q << 0.4, -0.7;

    auto sigma = spp::singular_values(chain, q);
    REQUIRE(sigma.size() == 2);

    REQUIRE(spp::condition_number(sigma));
    CHECK(std::isinf(*spp::condition_number(sigma)));
    REQUIRE(spp::isotropy(sigma));
    CHECK(*spp::isotropy(sigma) == Approx(0.0).margin(1e-12));
    REQUIRE(spp::manipulability(sigma));
    CHECK(*spp::manipulability(sigma) == Approx(0.0).margin(1e-12));

    REQUIRE(spp::is_near_singular(sigma));
    CHECK(*spp::is_near_singular(sigma));
    CHECK(*spp::is_near_singular(sigma, std::numeric_limits<double>::max()));
}

// The threshold is an argument, so the same configuration answers differently
// under two callers' definitions of "too close" -- which is why the answer
// cannot be a stored flag.
TEST_CASE("the near-singularity threshold is the caller's", "[ik][diagnostics]")
{
    auto chain = make_ur5_like_chain();
    vec6 q;
    q << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto sigma = spp::singular_values(chain, q);
    auto kappa = spp::condition_number(sigma);
    REQUIRE(kappa);
    REQUIRE(std::isfinite(*kappa));

    REQUIRE(spp::is_near_singular(sigma, *kappa * 0.5));
    CHECK(*spp::is_near_singular(sigma, *kappa * 0.5));
    REQUIRE(spp::is_near_singular(sigma, *kappa * 2.0));
    CHECK_FALSE(*spp::is_near_singular(sigma, *kappa * 2.0));

    CHECK(spp::is_near_singular(chain, q, *kappa * 0.5) == spp::is_near_singular(sigma, *kappa * 0.5));
    CHECK(spp::default_singularity_threshold_v<double> == 1e3);
}

TEST_CASE("an unmeasurable spectrum answers nothing rather than false", "[ik][diagnostics]")
{
    chain_dyn chain(spp::se3<double>::identity(), {}, {});
    auto sigma = spp::singular_values(chain, Eigen::VectorXd::Zero(0));

    CHECK(sigma.size() == 0);
    CHECK_FALSE(spp::is_near_singular(sigma).has_value());
    CHECK_FALSE(spp::is_near_singular(chain, Eigen::VectorXd::Zero(0)).has_value());
}

// ============================================================================
// A refused setup measured nothing and says so
// ============================================================================

TEST_CASE("a refused setup leaves the residual poison in place", "[ik][diagnostics]")
{
    auto chain = make_ur5_like_chain();
    spp::basic_ik_runner<spp::lm<chain6>> solver;
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 100, 50};

    vec6 seed = vec6::Zero();
    seed(2) = std::numeric_limits<double>::quiet_NaN();
    solver.setup(chain, spp::se3<double>::identity(), seed, criteria);

    auto result = solver.solve();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == spp::ik_failure::non_finite_input);
    CHECK(std::isnan(result.error().last_error_norm));
}

// ============================================================================
// The status-to-reason mapping is total
// ============================================================================

// Its fall-through arm used to claim an iteration limit for any status it was
// not written for, which is a specific and false statement about why a solve
// failed. Every status now maps to its own reason, or to the one reason that
// asserts nothing about the search.
TEST_CASE("every status maps to a reason that does not misstate it", "[ik][diagnostics]")
{
    using spp::detail::failure_reason_for;
    using spp::ik_failure;
    using spp::ik_status;

    CHECK(failure_reason_for(ik_status::diverged) == ik_failure::diverged);
    CHECK(failure_reason_for(ik_status::stalled) == ik_failure::stalled);
    CHECK(failure_reason_for(ik_status::iteration_limit) == ik_failure::iteration_limit);
    CHECK(failure_reason_for(ik_status::joint_limit_hit) == ik_failure::joint_limit_violation);
    CHECK(failure_reason_for(ik_status::aborted) == ik_failure::aborted);
    CHECK(failure_reason_for(ik_status::unreachable) == ik_failure::unreachable);
    CHECK(failure_reason_for(ik_status::dimension_mismatch) == ik_failure::dimension_mismatch);
    CHECK(failure_reason_for(ik_status::non_finite_input) == ik_failure::non_finite_input);
    CHECK(failure_reason_for(ik_status::unsupported_configuration)
        == ik_failure::unsupported_configuration);

    for (auto status : {ik_status::running, ik_status::converged, ik_status::not_initialized})
    {
        CHECK(failure_reason_for(status) == ik_failure::not_initialized);
        CHECK_FALSE(failure_reason_for(status) == ik_failure::iteration_limit);
    }
}

// ============================================================================
// Which feasible set the policy solved over
// ============================================================================

namespace
{

struct substituting_policy
{
    static constexpr bool substitutes_unbounded_bounds = true;
};

struct projecting_policy
{
};

chain_dyn make_unbounded_chain()
{
    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.3});
    auto unbounded = spp::testing::limits(
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    return chain_dyn(spp::se3<double>::identity(), {s1, s2}, {unbounded, wide()});
}

}

TEST_CASE("the substituted feasible set needs both a substituting policy and an unbounded joint",
    "[ik][diagnostics]")
{
    using spp::detail::feasible_set_solved;
    using spp::feasible_set;

    auto unbounded = make_unbounded_chain();
    auto bounded = make_ur5_like_chain();

    CHECK(feasible_set_solved<substituting_policy>(unbounded) == feasible_set::substituted);
    CHECK(feasible_set_solved<substituting_policy>(bounded) == feasible_set::declared);
    CHECK(feasible_set_solved<projecting_policy>(unbounded) == feasible_set::declared);
    CHECK(feasible_set_solved<projecting_policy>(bounded) == feasible_set::declared);
}

TEST_CASE("the result records the feasible set its policy solved over", "[ik][diagnostics]")
{
    auto chain = make_ur5_like_chain();
    vec6 q;
    q << 0.2, -0.6, 0.9, 0.2, -0.3, 0.5;
    auto fk = spp::forward_kinematics(chain, q);
    REQUIRE(fk.has_value());

    spp::basic_ik_runner<spp::lm<chain6>> solver;
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200, 400};
    solver.setup(chain, fk->end_effector, vec6::Zero(), criteria);

    auto result = solver.solve();
    REQUIRE(result.has_value());
    CHECK(result->solved_feasible_set == spp::feasible_set::declared);
}
