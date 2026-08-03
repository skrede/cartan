#include "../support/joint_limits_helpers.h"

#include <cartan/serial/ik/solvers.h>
#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/ik/detail/selection_metrics.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>

#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <cartan/serial/fk/forward_kinematics.h>

#include <Eigen/SVD>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = cartan;

using chain6 = spp::kinematic_chain<double, 6>;
using chain3 = spp::kinematic_chain<double, 3>;
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

/// A tall body Jacobian, where the characteristic length can reorder the
/// manipulability measure. On the square Jacobian of the 6R chain it cannot.
static chain3 make_planar_3r_chain()
{
    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.3});
    auto s3 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.4, 0, 0.3});

    spp::vector3<double> home_trans;
    home_trans << 0.8, 0.0, 0.3;
    auto home = spp::se3<double>(spp::so3<double>::identity(), home_trans);
    auto lim = wide();
    return chain3(home, {s1, s2, s3}, {lim, lim, lim});
}

static vec6 joints(double a, double b, double c, double d, double e, double f)
{
    vec6 q;
    q << a, b, c, d, e, f;
    return q;
}

static spp::se3<double> reachable_target(const chain6& chain, const vec6& q)
{
    return spp::forward_kinematics_unchecked(chain, q).end_effector;
}

/// The winning configuration of a two-policy race under one objective. Returned
/// as the configuration rather than the solver index, because the index names a
/// position in the pack and two runs with the policies in opposite orders put
/// the same candidate at different indices.
template <typename First, typename Second>
static vec6 raced_winner(
    const chain6& chain,
    const spp::se3<double>& target,
    spp::ik_objective objective,
    First first,
    Second second)
{
    spp::basic_ik_runner solver{std::move(first), std::move(second)};
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200, 400};
    spp::solver_options<double> opts{.objective = objective};
    solver.setup(chain, target, vec6::Zero(), criteria, opts);

    auto result = solver.solve();
    REQUIRE(result.has_value());
    return result->solution.position;
}

static vec6 raced_winner(
    const chain6& chain,
    const spp::se3<double>& target,
    spp::ik_objective objective)
{
    return raced_winner(chain, target, objective,
        spp::speed_ik_runner<chain6>{}, spp::robust_ik_runner<chain6>{});
}

// ============================================================================
// The residual objective and the joint-displacement objective disagree
// ============================================================================

// Pre-change the joint-displacement objective did not exist, so this case did
// not compile. The two rankings genuinely disagree here: the lower-residual
// candidate sits 9.07 rad from the seed and the nearer one 3.10 rad.
TEST_CASE("residual and joint displacement select different candidates", "[ik][selection]")
{
    auto chain = make_ur5_like_chain();
    auto target = reachable_target(chain, joints(0.5, -0.9, 1.3, -0.6, 0.4, 1.7));

    auto by_residual = raced_winner(chain, target, spp::ik_objective::min_error_norm);
    auto by_distance = raced_winner(chain, target, spp::ik_objective::min_joint_distance);

    REQUIRE_FALSE(by_residual.isApprox(by_distance));
    REQUIRE(by_distance.norm() < by_residual.norm());
}

// ============================================================================
// The two Jacobian measures disagree with the residual through a racing runner
// ============================================================================

// The strongest fails-before in this suite: the pre-change racing path had no
// implementation of either measure and ranked every objective on the stored
// error norm, so all three objectives returned the same candidate. Measured on
// the pre-change tree, all three reported the solver at index 0.
TEST_CASE("racing manipulability does not select the lowest-residual candidate", "[ik][selection]")
{
    auto chain = make_ur5_like_chain();
    auto target = reachable_target(chain, joints(0.2, -1.5, 2.0, 0.7, -0.2, 1.1));

    auto by_residual = raced_winner(chain, target, spp::ik_objective::min_error_norm);
    auto by_manipulability = raced_winner(chain, target, spp::ik_objective::max_manipulability);

    REQUIRE_FALSE(by_residual.isApprox(by_manipulability));

    auto chosen = spp::detail::jacobian_metric(
        spp::ik_objective::max_manipulability, chain, by_manipulability, 1.0);
    auto rejected = spp::detail::jacobian_metric(
        spp::ik_objective::max_manipulability, chain, by_residual, 1.0);
    REQUIRE(chosen);
    REQUIRE(rejected);
    REQUIRE(*chosen > *rejected);
}

TEST_CASE("racing isotropy does not select the lowest-residual candidate", "[ik][selection]")
{
    auto chain = make_ur5_like_chain();
    auto target = reachable_target(chain, joints(0.2, -1.5, 2.0, 0.7, -0.2, 1.1));

    auto by_residual = raced_winner(chain, target, spp::ik_objective::min_error_norm);
    auto by_isotropy = raced_winner(chain, target, spp::ik_objective::max_isotropy);

    REQUIRE_FALSE(by_residual.isApprox(by_isotropy));

    auto chosen = spp::detail::jacobian_metric(
        spp::ik_objective::max_isotropy, chain, by_isotropy, 1.0);
    auto rejected = spp::detail::jacobian_metric(
        spp::ik_objective::max_isotropy, chain, by_residual, 1.0);
    REQUIRE(chosen);
    REQUIRE(rejected);
    REQUIRE(*chosen > *rejected);
}

// ============================================================================
// The comparison is independent of the order the candidates arrive in
// ============================================================================

// Stated over the comparison rather than over a runner: reversing a runner's
// policy order changes which policy receives the caller's seed and which
// receives a generated one, so the two runs race different candidates and are
// expected to differ. What must not depend on order is the ranking itself.
TEST_CASE("selection is independent of candidate order", "[ik][selection]")
{
    auto objectives = {
        spp::ik_objective::min_error_norm,
        spp::ik_objective::min_joint_distance,
        spp::ik_objective::max_manipulability,
        spp::ik_objective::max_isotropy};

    for (auto objective : objectives)
    {
        std::optional<double> low{0.25};
        std::optional<double> high{0.75};

        const bool high_wins_forward =
            spp::detail::improves_on(objective, high, low);
        const bool low_wins_reversed =
            spp::detail::improves_on(objective, low, high);

        REQUIRE(high_wins_forward != low_wins_reversed);
        REQUIRE(high_wins_forward == spp::detail::maximizes(objective));
    }
}

// ============================================================================
// The characteristic length, and the ranking it can and cannot change
// ============================================================================

// A length of one reproduces the unnormalized arithmetic exactly, which is what
// makes the parameter purely additive.
TEST_CASE("a characteristic length of one reproduces the unnormalized measures", "[ik][selection]")
{
    auto chain = make_ur5_like_chain();
    auto q = joints(0.3, -0.5, 0.8, 0.1, -0.4, 0.7);

    auto J = spp::body_jacobian_unchecked(chain, spp::forward_kinematics_unchecked(chain, q));
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(
        J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& sigma = svd.singularValues();

    double product = 1.0;
    for (int i = 0; i < sigma.size(); ++i)
    {
        product *= sigma(i);
    }

    auto manipulability = spp::detail::jacobian_metric(
        spp::ik_objective::max_manipulability, chain, q, 1.0);
    auto isotropy = spp::detail::jacobian_metric(
        spp::ik_objective::max_isotropy, chain, q, 1.0);

    REQUIRE(manipulability);
    REQUIRE(isotropy);
    REQUIRE(std::abs(*manipulability - product) < 1e-15);
    REQUIRE(std::abs(*isotropy - sigma(sigma.size() - 1) / sigma(0)) < 1e-15);
}

// On a square body Jacobian the product of the singular values is the absolute
// determinant, so dividing three of the six rows by the length rescales every
// candidate by the same L^-3 and can never reorder two of them. The measure
// still needs the length to carry a coherent unit; it just does not select
// differently because of it.
TEST_CASE("the length rescales manipulability without reordering a square Jacobian", "[ik][selection]")
{
    auto chain = make_ur5_like_chain();
    auto qa = joints(0.3, -0.5, 0.8, 0.1, -0.4, 0.7);
    auto qb = joints(1.0, 0.4, -1.4, 0.9, -1.3, 0.1);

    const auto manip = [&](const vec6& q, double length)
    {
        auto m = spp::detail::jacobian_metric(
            spp::ik_objective::max_manipulability, chain, q, length);
        REQUIRE(m);
        return *m;
    };

    REQUIRE(std::abs(manip(qa, 0.1) / manip(qa, 1.0) - 1000.0) < 1e-6);
    REQUIRE(std::abs(manip(qb, 0.1) / manip(qb, 1.0) - 1000.0) < 1e-6);
    REQUIRE((manip(qa, 1.0) > manip(qb, 1.0)) == (manip(qa, 0.1) > manip(qb, 0.1)));
}

// A tall Jacobian has no such uniform factor, and there the length does reorder.
TEST_CASE("the length reorders manipulability on a tall Jacobian", "[ik][selection]")
{
    auto chain = make_planar_3r_chain();
    Eigen::Vector<double, 3> qa;
    Eigen::Vector<double, 3> qb;
    qa << -1.6095, 0.0744, 0.0184;
    qb << 1.7977, -1.8863, 0.3176;

    const auto manip = [&](const Eigen::Vector<double, 3>& q, double length)
    {
        auto m = spp::detail::jacobian_metric(
            spp::ik_objective::max_manipulability, chain, q, length);
        REQUIRE(m);
        return *m;
    };

    REQUIRE(manip(qa, 1.0) > manip(qb, 1.0));
    REQUIRE(manip(qa, 0.01) < manip(qb, 0.01));
}

// The isotropy ratio is not a uniform rescale either, so the length reorders it
// on the same square Jacobian the manipulability measure is insensitive on.
TEST_CASE("the length reorders isotropy on a square Jacobian", "[ik][selection]")
{
    auto chain = make_ur5_like_chain();
    auto qa = joints(-1.8401, 0.2747, 0.8150, 1.9387, -1.1781, 1.7383);
    auto qb = joints(0.5814, 0.8197, -1.0965, -1.5276, 1.8125, -0.6448);

    const auto isotropy = [&](const vec6& q, double length)
    {
        auto m = spp::detail::jacobian_metric(
            spp::ik_objective::max_isotropy, chain, q, length);
        REQUIRE(m);
        return *m;
    };

    REQUIRE(isotropy(qa, 1.0) < isotropy(qb, 1.0));
    REQUIRE(isotropy(qa, 0.1) > isotropy(qb, 0.1));
}

// ============================================================================
// A chain with no joints leaves both Jacobian measures undefined
// ============================================================================

TEST_CASE("both Jacobian measures are undefined on a jointless chain", "[ik][selection]")
{
    chain_dyn chain(spp::se3<double>::identity(), {}, {});
    Eigen::VectorXd q = Eigen::VectorXd::Zero(0);

    REQUIRE_FALSE(
        spp::detail::jacobian_metric(spp::ik_objective::max_manipulability, chain, q, 1.0));
    REQUIRE_FALSE(
        spp::detail::jacobian_metric(spp::ik_objective::max_isotropy, chain, q, 1.0));

    REQUIRE(spp::detail::selection_admissibility(
        spp::ik_objective::max_manipulability, chain, 1.0)
        == spp::ik_status::unsupported_configuration);
    REQUIRE_FALSE(spp::detail::selection_admissibility(
        spp::ik_objective::min_error_norm, chain, 1.0));
}

// ============================================================================
// Setup refuses what it cannot rank, rather than ranking on a fabrication
// ============================================================================

TEST_CASE("setup refuses an unrankable objective and reports it", "[ik][selection]")
{
    chain_dyn chain(spp::se3<double>::identity(), {}, {});

    spp::basic_ik_runner<spp::lm<chain_dyn>> solver;
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200, 400};
    spp::solver_options<double> opts{.objective = spp::ik_objective::max_isotropy};
    solver.setup(chain, spp::se3<double>::identity(), Eigen::VectorXd::Zero(0), criteria, opts);

    REQUIRE(solver.status() == spp::ik_status::unsupported_configuration);

    auto result = solver.solve();
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error().reason == spp::ik_failure::unsupported_configuration);
}

TEST_CASE("setup refuses a characteristic length it cannot divide by", "[ik][selection]")
{
    auto chain = make_ur5_like_chain();

    for (double length : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()})
    {
        REQUIRE(spp::detail::selection_admissibility(
            spp::ik_objective::max_manipulability, chain, length)
            == spp::ik_status::unsupported_configuration);
    }

    REQUIRE_FALSE(spp::detail::selection_admissibility(
        spp::ik_objective::max_manipulability, chain, 1.0));
}

// A joint-space displacement over a chain mixing revolute and prismatic joints
// would add radians to metres. The refusal is on that mixture and not on the
// coarser joint_kind classification, which calls a non-principal revolute axis
// `general` and would refuse a chain that is entirely in radians.
TEST_CASE("joint displacement is refused only on a revolute-prismatic mixture", "[ik][selection]")
{
    auto revolute = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto skew = spp::screw_axis<double>::revolute(
        spp::vector3<double>(1, 1, 1).normalized(), {0, 0, 0.2});
    auto prismatic = spp::screw_axis<double>::prismatic({0, 0, 1});
    auto lim = wide();
    auto home = spp::se3<double>::identity();

    chain_dyn mixed(home, {revolute, prismatic}, {lim, lim});
    chain_dyn skewed(home, {revolute, skew}, {lim, lim});

    REQUIRE(spp::detail::mixes_revolute_and_prismatic(mixed));
    REQUIRE_FALSE(spp::detail::mixes_revolute_and_prismatic(skewed));

    REQUIRE(spp::detail::selection_admissibility(
        spp::ik_objective::min_joint_distance, mixed, 1.0)
        == spp::ik_status::unsupported_configuration);
    REQUIRE_FALSE(spp::detail::selection_admissibility(
        spp::ik_objective::min_joint_distance, skewed, 1.0));
    REQUIRE_FALSE(spp::detail::selection_admissibility(
        spp::ik_objective::min_error_norm, mixed, 1.0));
}

// ============================================================================
// The metric the winner was ranked on is reachable, with its objective
// ============================================================================

TEST_CASE("the result carries the winning metric and its objective", "[ik][selection]")
{
    auto chain = make_ur5_like_chain();
    auto target = reachable_target(chain, joints(0.3, -0.5, 0.8, 0.1, -0.4, 0.7));

    spp::basic_ik_runner solver{
        spp::speed_ik_runner<chain6>{}, spp::robust_ik_runner<chain6>{}};
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200, 400};
    spp::solver_options<double> opts{.objective = spp::ik_objective::max_manipulability};
    solver.setup(chain, target, vec6::Zero(), criteria, opts);

    auto result = solver.solve();
    REQUIRE(result.has_value());
    REQUIRE(result->selection_objective == spp::ik_objective::max_manipulability);
    REQUIRE(result->selection_metric);

    auto recomputed = spp::detail::jacobian_metric(
        spp::ik_objective::max_manipulability, chain, result->solution.position, 1.0);
    REQUIRE(recomputed);
    REQUIRE(std::abs(*result->selection_metric - *recomputed) < 1e-12);
}

// The speed objective ranks nothing, so an unranked win reads as absent rather
// than as a zero a caller could mistake for a measurement.
TEST_CASE("the speed objective leaves the winning metric absent", "[ik][selection]")
{
    auto chain = make_ur5_like_chain();
    auto target = reachable_target(chain, joints(0.3, -0.5, 0.8, 0.1, -0.4, 0.7));

    spp::basic_ik_runner solver{
        spp::speed_ik_runner<chain6>{}, spp::robust_ik_runner<chain6>{}};
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200, 400};
    solver.setup(chain, target, vec6::Zero(), criteria);

    auto result = solver.solve();
    REQUIRE(result.has_value());
    REQUIRE(result->selection_objective == spp::ik_objective::speed);
    REQUIRE_FALSE(result->selection_metric);
}

// ============================================================================
// The continuation under a non-speed objective is a genuine multistart
// ============================================================================

// Pre-change the continuation re-seeded the policy at the configuration it had
// just converged to; it converged again for zero work and the budget guard
// ended the solve. Measured on the pre-change tree, this solve spent 5 of its
// 3000 units and reported a manipulability of 3.306525e-02 -- the single start
// it began from.
TEST_CASE("a single-policy continuation restarts somewhere new", "[ik][selection]")
{
    auto chain = make_ur5_like_chain();
    auto seed = joints(0.3, -0.5, 0.8, 0.1, -0.4, 0.7);
    auto target = reachable_target(chain, seed);

    spp::basic_ik_runner<spp::lm<chain6>> solver;
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200, 3000};
    spp::solver_options<double> opts{.objective = spp::ik_objective::max_manipulability};
    solver.setup(chain, target, vec6::Zero(), criteria, opts);

    auto result = solver.solve();
    REQUIRE(result.has_value());
    REQUIRE(result->iterations > 5);
    REQUIRE(result->selection_metric);

    auto at_seed = spp::detail::jacobian_metric(
        spp::ik_objective::max_manipulability, chain, seed, 1.0);
    REQUIRE(at_seed);
    REQUIRE(*result->selection_metric > *at_seed);
}
