// Cross-solver semantics: converged-implies-feasible, unweighted convergence
// gate, and feasibility-first best-iterate retention.

#include <cartan/serial/ik/detail/limit_enforcement.h>
#include <cartan/serial/ik/detail/convergence.h>
#include <cartan/serial/ik/policy/error_weight.h>
#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/solver/lbfgsb.h>
#include <cartan/serial/ik/solver/projected_lm.h>
#include <cartan/serial/ik/solver/newton_raphson.h>
#include <cartan/serial/ik/wrapper/restart_wrapper.h>
#ifdef CARTAN_BUILD_ARGMIN
#include <cartan/serial/ik/solver/argmin_lm.h>
#include <cartan/serial/ik/solver/argmin_slsqp.h>
#endif

#include <limits>
#include <algorithm>

#include <cartan/types.h>
#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = cartan;

namespace
{

// A 2R planar chain with tight symmetric limits. The tight bounds let a target
// be constructed whose only pose-solution the unconstrained solvers reach sits
// outside the box.
spp::kinematic_chain<double, 2> make_bounded_2r_chain(double limit)
{
    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<double>::revolute({0, 0, 1}, {1, 0, 0});

    spp::vector3<double> home_trans;
    home_trans << 2, 0, 0;
    auto home = spp::se3<double>(spp::so3<double>::identity(), home_trans);

    spp::joint_limits<double> lim{-limit, limit};
    return spp::kinematic_chain<double, 2>(home, {s1, s2}, {lim, lim});
}

template <typename Stepper, int N>
spp::ik_status run_stepper(
    Stepper& stepper, const spp::kinematic_chain<double, N>& chain, int max_steps)
{
    spp::ik_status status = spp::ik_status::running;
    for (int i = 0; i < max_steps && status == spp::ik_status::running; ++i)
    {
        status = stepper.step(chain, 1).status;
    }
    return status;
}

}

// ============================================================================
// converged implies feasible (no_limits trust-region family)
// ============================================================================

TEST_CASE("within_limits skips unbounded joints and flags out-of-range joints",
          "[ik][semantics][feasibility]")
{
    auto chain = make_bounded_2r_chain(1.0);
    const double tol = spp::detail::default_feasibility_tol<double>();

    Eigen::Vector<double, 2> q_in;
    q_in << 0.5, -0.5;
    REQUIRE(spp::detail::within_limits(q_in, chain, tol));

    Eigen::Vector<double, 2> q_out;
    q_out << 2.0, 0.0;  // joint 0 well outside [-1, 1]
    REQUIRE_FALSE(spp::detail::within_limits(q_out, chain, tol));

    // A joint sitting exactly at the bound is still feasible.
    Eigen::Vector<double, 2> q_edge;
    q_edge << 1.0, -1.0;
    REQUIRE(spp::detail::within_limits(q_edge, chain, tol));
}

// The no_limits trust-region solvers must not report converged at an
// out-of-limits configuration. Seeding the solver at the exact out-of-range
// pose solution makes the pose gate fire on entry; the feasibility check must
// then downgrade the result rather than declare success.
template <typename Solver>
static void assert_no_converged_out_of_limits()
{
    auto chain = make_bounded_2r_chain(1.0);

    Eigen::Vector<double, 2> q_out;
    q_out << 2.5, 1.8;  // both joints outside [-1, 1]
    auto target = spp::forward_kinematics(chain, q_out).end_effector;

    spp::convergence_criteria<double> criteria{};
    criteria.max_iterations_per_attempt = 200;

    Solver solver;
    solver.setup(chain, target, q_out, criteria);
    auto status = run_stepper(solver, chain, 400);

    REQUIRE_FALSE(solver.converged());
    REQUIRE(status != spp::ik_status::converged);
}

TEST_CASE("dls does not report converged out of limits", "[ik][semantics][feasibility]")
{
    assert_no_converged_out_of_limits<spp::dls<spp::kinematic_chain<double, 2>>>();
}

TEST_CASE("builtin_lm does not report converged out of limits", "[ik][semantics][feasibility]")
{
    assert_no_converged_out_of_limits<spp::builtin_lm<spp::kinematic_chain<double, 2>>>();
}

#ifdef CARTAN_BUILD_ARGMIN
TEST_CASE("argmin_lm does not report converged out of limits", "[ik][semantics][feasibility]")
{
    assert_no_converged_out_of_limits<spp::argmin_lm<spp::kinematic_chain<double, 2>>>();
}
#endif

TEST_CASE("projected_lm converged solutions are feasible", "[ik][semantics][feasibility]")
{
    // projected_lm box-projects internally, so a converged result is feasible by
    // construction; assert the invariant on a reachable in-limit target.
    auto chain = make_bounded_2r_chain(std::numbers::pi);

    Eigen::Vector<double, 2> q_known;
    q_known << 0.6, -0.7;
    auto target = spp::forward_kinematics(chain, q_known).end_effector;

    spp::convergence_criteria<double> criteria{};
    criteria.max_iterations_per_attempt = 200;

    spp::projected_lm<spp::kinematic_chain<double, 2>> solver;
    solver.setup(chain, target, q_known, criteria);
    run_stepper(solver, chain, 400);

    if (solver.converged())
    {
        REQUIRE(spp::detail::within_limits(
            solver.solution(), chain, spp::detail::default_feasibility_tol<double>()));
    }
}

// ============================================================================
// Pose-equivalence-class feasibility: a converged-at-2*pi-equivalent config is
// recovered to its in-limits representative; genuinely box-infeasible poses and
// non-pose-periodic (prismatic / helical) DOFs are handled correctly.
// ============================================================================

// A single-DOF chain carrying one arbitrary screw axis, used to probe the
// canonicalization on prismatic / helical joints in isolation.
static spp::kinematic_chain<double, 1> make_single_axis_chain(
    const spp::screw_axis<double>& axis, double lo, double hi)
{
    auto home = spp::se3<double>::identity();
    spp::joint_limits<double> lim{lo, hi};
    return spp::kinematic_chain<double, 1>(home, {axis}, {lim});
}

TEST_CASE("is_zero_pitch_revolute classifies revolute, prismatic, and helical axes",
          "[ik][semantics][feasibility]")
{
    // Pure revolute: v = -omega x point, so omega . v = 0 identically.
    auto revolute = spp::screw_axis<double>::revolute({0, 0, 1}, {1, 0, 0});
    REQUIRE(spp::detail::is_zero_pitch_revolute(revolute));

    // Prismatic: omega = 0, aperiodic -- never wrapped.
    auto prismatic = spp::screw_axis<double>::prismatic({0, 0, 1});
    REQUIRE_FALSE(spp::detail::is_zero_pitch_revolute(prismatic));

    // Helical: omega unit but omega . v = pitch != 0 -- exp(2*pi*S) translates.
    spp::vector6<double> screw;
    screw << 0, 0, 1, 0, 0, 0.5;  // omega = e_z, v_z = 0.5 => pitch 0.5
    auto helical = *spp::screw_axis<double>::from_vector(screw);
    REQUIRE_FALSE(spp::detail::is_zero_pitch_revolute(helical));
}

TEST_CASE("canonical_angle_in_limits performs the general arc test",
          "[ik][semantics][feasibility]")
{
    const double tol = spp::detail::default_feasibility_tol<double>();
    const double pi = std::numbers::pi;
    const double two_pi = 2.0 * pi;

    // A 2*pi-equivalent angle above the range folds back into [-pi, pi].
    double r = spp::detail::canonical_angle_in_limits(0.5 + two_pi, -pi, pi, tol);
    REQUIRE(std::abs(r - 0.5) < 1e-9);

    // The arc test anchors to the actual bounds, not a naive wrap to [-pi, pi]:
    // for the offset range [3, 5], the class of (3.5 - 2*pi) folds up to 3.5,
    // which sits in the arc -- whereas a naive [-pi, pi] wrap would return the
    // out-of-arc representative near -2.78.
    double off = spp::detail::canonical_angle_in_limits(3.5 - two_pi, 3.0, 5.0, tol);
    REQUIRE(std::abs(off - 3.5) < 1e-9);
    REQUIRE(off >= 3.0 - tol);
    REQUIRE(off <= 5.0 + tol);

    // Span >= 2*pi is always satisfiable; the returned representative sits inside.
    double wide = spp::detail::canonical_angle_in_limits(10.0, -4.0, 4.0, tol);
    REQUIRE(wide >= -4.0 - tol);
    REQUIRE(wide <= 4.0 + tol);

    // A gap < 2*pi that the class misses yields a value that stays out of the box
    // (>= lo but > hi), so the downstream box check rejects it.
    double gap = spp::detail::canonical_angle_in_limits(2.5, -1.0, 1.0, tol);
    REQUIRE(gap > 1.0 + tol);
}

// (a) A no_limits solver seeded at a 2*pi-equivalent of an in-limits solution
// must converge AND return the in-limits representative, not the unwrapped angle.
template <typename Solver>
static void assert_accepts_2pi_equivalent()
{
    const double two_pi = 2.0 * std::numbers::pi;
    auto chain = make_bounded_2r_chain(std::numbers::pi);  // bounds [-pi, pi]

    Eigen::Vector<double, 2> q_in_limits;
    q_in_limits << 0.5, -0.5;

    // Seed one joint a full turn away: same pose, but outside the box as returned.
    Eigen::Vector<double, 2> q_seed;
    q_seed << 0.5 + two_pi, -0.5;
    auto target = spp::forward_kinematics(chain, q_seed).end_effector;

    spp::convergence_criteria<double> criteria{};
    criteria.max_iterations_per_attempt = 200;

    Solver solver;
    solver.setup(chain, target, q_seed, criteria);
    run_stepper(solver, chain, 400);

    REQUIRE(solver.converged());
    // The returned configuration is the in-limits 2*pi-equivalent, not the seed.
    REQUIRE(spp::detail::within_limits(
        solver.solution(), chain, spp::detail::default_feasibility_tol<double>()));
    REQUIRE(std::abs(solver.solution()(0) - 0.5) < 1e-6);
    REQUIRE(std::abs(solver.solution()(1) - (-0.5)) < 1e-6);
}

TEST_CASE("dls recovers the in-limits 2pi-equivalent solution", "[ik][semantics][feasibility]")
{
    assert_accepts_2pi_equivalent<spp::dls<spp::kinematic_chain<double, 2>>>();
}

TEST_CASE("builtin_lm recovers the in-limits 2pi-equivalent solution", "[ik][semantics][feasibility]")
{
    assert_accepts_2pi_equivalent<spp::builtin_lm<spp::kinematic_chain<double, 2>>>();
}

#ifdef CARTAN_BUILD_ARGMIN
TEST_CASE("argmin_lm recovers the in-limits 2pi-equivalent solution", "[ik][semantics][feasibility]")
{
    assert_accepts_2pi_equivalent<spp::argmin_lm<spp::kinematic_chain<double, 2>>>();
}
#endif

// (b) Genuinely box-infeasible poses are still rejected, not force-accepted.
TEST_CASE("genuinely infeasible configurations are rejected after canonicalization",
          "[ik][semantics][feasibility]")
{
    const double tol = spp::detail::default_feasibility_tol<double>();

    SECTION("revolute with a limit arc < 2*pi that the class misses")
    {
        auto chain = make_bounded_2r_chain(1.0);  // bounds [-1, 1], span 2 < 2*pi
        // 2.5 has no 2*pi-equivalent inside [-1, 1] (nearest are 2.5 and -3.78).
        Eigen::Vector<double, 2> q;
        q << 2.5, 0.0;
        REQUIRE_FALSE(spp::detail::feasible_after_canonicalization(q, chain, tol));
    }

    SECTION("prismatic out of range is aperiodic and not wrapped")
    {
        auto chain = make_single_axis_chain(
            spp::screw_axis<double>::prismatic({0, 0, 1}), 0.0, 1.0);
        Eigen::Vector<double, 1> q;
        q << 5.0;
        REQUIRE_FALSE(spp::detail::feasible_after_canonicalization(q, chain, tol));
        // Aperiodic: the value is left untouched by the canonicalization.
        REQUIRE(q(0) == 5.0);
    }
}

// (c) A helical / nonzero-pitch DOF is never wrapped: exp(2*pi*S) translates, so
// the angle is not pose-periodic and canonicalization must leave it untouched.
TEST_CASE("helical DOF is not wrapped by canonicalization", "[ik][semantics][feasibility]")
{
    const double tol = spp::detail::default_feasibility_tol<double>();

    spp::vector6<double> screw;
    screw << 0, 0, 1, 0, 0, 0.5;  // pitch 0.5
    auto helical = *spp::screw_axis<double>::from_vector(screw);
    auto chain = make_single_axis_chain(helical, -1.0, 1.0);

    Eigen::Vector<double, 1> q;
    q << 10.0;  // a full-turn shift would change the pose -- must not be applied
    spp::detail::canonicalize_into_limits(q, chain, tol);
    REQUIRE(q(0) == 10.0);
    REQUIRE_FALSE(spp::detail::feasible_after_canonicalization(q, chain, tol));
}

// ============================================================================
// the convergence gate tests raw (unweighted) component norms
// ============================================================================

TEST_CASE("convergence gate ignores error_weight", "[ik][semantics][gate]")
{
    // A body error whose raw component norms sit just below the tolerance, but
    // whose weighted norms are pushed above it by a non-unit weight. The gate
    // the solvers now use must key off the raw norms.
    cartan::convergence_criteria<double> criteria{};
    criteria.orientation_tol = 1e-3;
    criteria.position_tol = 1e-3;

    Eigen::Vector<double, 6> body_error;
    body_error << 8e-4, 0.0, 0.0,   // angular (head<3>): raw norm 8e-4 < 1e-3
                  8e-4, 0.0, 0.0;   // linear  (tail<3>): raw norm 8e-4 < 1e-3

    cartan::error_weight<double> heavy;
    heavy.weights << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;  // weighted norm 4e-3 > 1e-3

    REQUIRE(cartan::detail::is_converged_unweighted(body_error, criteria));
    REQUIRE_FALSE(cartan::detail::is_converged(body_error, heavy, criteria));
}

// A solver carrying a heavy error_weight still declares convergence on a
// reachable in-limit target, because the stopping test no longer weights the
// body error. Seeding at the exact solution makes the gate fire on entry.
template <typename Solver>
static void assert_weight_does_not_block_convergence()
{
    auto chain = make_bounded_2r_chain(std::numbers::pi);

    Eigen::Vector<double, 2> q_known;
    q_known << 0.4, -0.6;
    auto target = spp::forward_kinematics(chain, q_known).end_effector;

    spp::convergence_criteria<double> criteria{};
    criteria.max_iterations_per_attempt = 200;

    cartan::error_weight<double> heavy;
    heavy.weights << 8.0, 8.0, 8.0, 8.0, 8.0, 8.0;

    Solver solver;
    solver.setup(chain, target, q_known, criteria, heavy);
    run_stepper(solver, chain, 400);

    REQUIRE(solver.converged());
}

TEST_CASE("newton_raphson convergence is unaffected by error_weight", "[ik][semantics][gate]")
{
    assert_weight_does_not_block_convergence<spp::newton_raphson<spp::kinematic_chain<double, 2>>>();
}

TEST_CASE("lbfgsb convergence is unaffected by error_weight", "[ik][semantics][gate]")
{
    assert_weight_does_not_block_convergence<spp::builtin_lbfgsb<spp::kinematic_chain<double, 2>>>();
}

TEST_CASE("projected_lm convergence is unaffected by error_weight", "[ik][semantics][gate]")
{
    assert_weight_does_not_block_convergence<spp::projected_lm<spp::kinematic_chain<double, 2>>>();
}

// ============================================================================
// restarting solvers retain the feasibility-first best-so-far iterate
// ============================================================================

// On a target the solver cannot reach it exhausts its restarts and terminates
// non-converged. The reported result must be the lowest-error iterate seen
// across all attempts (feasibility-first), never the last perturbed attempt. We
// drive the solver step-by-step, tracking the minimum error it ever reports;
// the final reported error must equal that minimum, and the returned q must be
// feasible.
template <typename Solver>
static void assert_retains_best_iterate()
{
    auto chain = make_bounded_2r_chain(std::numbers::pi);

    // Unreachable: the 2R arm reaches ~2 m, this target sits at 5 m.
    spp::vector3<double> far_trans;
    far_trans << 5.0, 0.0, 0.0;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    Eigen::Vector<double, 2> q0;
    q0 << 0.1, 0.1;

    spp::convergence_criteria<double> criteria{};
    criteria.max_iterations_per_attempt = 30;
    criteria.max_total_work_units = 600;

    Solver solver;
    solver.setup(chain, target, q0, criteria);

    double running_min = std::numeric_limits<double>::max();
    spp::ik_status status = spp::ik_status::running;
    for (int i = 0; i < 4000 && status == spp::ik_status::running; ++i)
    {
        status = solver.step(chain, 1).status;
        running_min = std::min(running_min, solver.error_norm());
    }

    REQUIRE_FALSE(solver.converged());
    // The final reported error is the best (minimum) seen, not the last attempt.
    REQUIRE(solver.error_norm() <= running_min + 1e-9);
    // Feasibility-first retention returns a configuration inside the joint box.
    REQUIRE(spp::detail::within_limits(
        solver.solution(), chain, spp::detail::default_feasibility_tol<double>()));
}

TEST_CASE("projected_lm returns the best-so-far iterate on a terminal solve",
          "[ik][semantics][retention]")
{
    assert_retains_best_iterate<spp::projected_lm<spp::kinematic_chain<double, 2>>>();
}

TEST_CASE("restart_wrapper returns the best-so-far iterate on a terminal solve",
          "[ik][semantics][retention]")
{
    assert_retains_best_iterate<spp::restart_wrapper<spp::kinematic_chain<double, 2>>>();
}

#ifdef CARTAN_BUILD_ARGMIN
TEST_CASE("argmin_slsqp returns the best-so-far iterate on a terminal solve",
          "[ik][semantics][retention]")
{
    assert_retains_best_iterate<spp::argmin_slsqp<spp::kinematic_chain<double, 2>>>();
}
#endif
