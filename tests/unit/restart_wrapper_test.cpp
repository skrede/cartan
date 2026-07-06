#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/ik/wrapper/restart_wrapper.h>
#include <cartan/serial/ik/solver/lm.h>

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

// ============================================================================
// Helper: UR5-like 6R chain
// ============================================================================

static spp::kinematic_chain<double, 6> make_ur5_like_chain()
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

    spp::joint_limits<double> lim{-2 * std::numbers::pi, 2 * std::numbers::pi};
    return spp::kinematic_chain<double, 6>(home, {s1, s2, s3, s4, s5, s6},
                                  {lim, lim, lim, lim, lim, lim});
}

// ============================================================================
// Helper: run stepper to completion
// ============================================================================

template <int N, typename Scalar, typename Stepper>
spp::ik_status run_stepper(
    Stepper& stepper,
    const spp::kinematic_chain<Scalar, N>& chain,
    int max_steps)
{
    spp::ik_status status = spp::ik_status::running;
    for (int i = 0; i < max_steps && status == spp::ik_status::running; ++i)
    {
        status = stepper.step(chain, 1).status;
    }
    return status;
}

// ============================================================================
// Concept satisfaction
// ============================================================================

TEST_CASE("restart_wrapper concept satisfaction", "[ik][restart]")
{
    static_assert(spp::solve_policy<
        spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>>>);
}

// ============================================================================
// Trivial convergence (FK roundtrip, no restart needed)
// ============================================================================

TEST_CASE("restart_wrapper trivial convergence", "[ik][restart]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 2000;  // matches run_stepper outer bound; allow multiple restart attempts

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 2000);

    REQUIRE(status == spp::ik_status::converged);

    auto fk_sol = spp::forward_kinematics(chain, stepper.solution());
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.head<3>().norm() < 1e-6);
    REQUIRE(err.tail<3>().norm() < 1e-6);
}

// ============================================================================
// Re-seeds after stall
// ============================================================================

TEST_CASE("restart_wrapper re-seeds after stall", "[ik][restart]")
{
    auto chain = make_ur5_like_chain();

    // Target reachable but far from a zero seed -- inner stepper may stall
    // on the first attempt with very few iterations allowed
    Eigen::Vector<double, 6> q_known;
    q_known << 2.5, -1.8, 1.2, -2.0, 1.5, -1.0;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    // Use inner stepper options that make first attempt likely to stall
    spp::lm<spp::kinematic_chain<double, 6>>::options inner_opts;
    inner_opts.stall_window = 3;
    inner_opts.stall_threshold = 1e-4;

    spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>>::options opts;
    opts.max_restarts = 20;

    spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>> stepper(
        opts, spp::lm<spp::kinematic_chain<double, 6>>(inner_opts));

    // Seed far from solution with tight iteration limit to force stalls
    Eigen::Vector<double, 6> q0;
    q0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 15;          // per-attempt cap forces stall + restart
    criteria.max_total_work_units = 5000;              // matches run_stepper max_steps; permits many restart attempts

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 5000);

    // Post-refactor invariant: stepper.iterations() returns cumulative inner units
    // across all restart attempts. Exceeding max_iterations_per_attempt indicates
    // that the wrapper triggered at least one restart and the second attempt
    // billed additional inner units.
    REQUIRE(stepper.iterations() > criteria.max_iterations_per_attempt);

    // If it converged, great. If not, it at least tried multiple restarts.
    if (status == spp::ik_status::converged)
    {
        auto fk_sol = spp::forward_kinematics(chain, stepper.solution());
        auto err = (fk_sol.end_effector.inverse() * target).log();
        REQUIRE(err.head<3>().norm() < 1e-5);
        REQUIRE(err.tail<3>().norm() < 1e-5);
    }
}

// ============================================================================
// Warm-start lambda
// ============================================================================

TEST_CASE("restart_wrapper warm-start lambda", "[ik][restart]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 2.5, -1.8, 1.2, -2.0, 1.5, -1.0;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    // Run with warm-start (the default)
    spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>>::options opts;
    opts.max_restarts = 30;

    spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>> stepper(opts);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 20;
    criteria.max_total_work_units = 10000;  // matches run_stepper outer bound; allow many warm-restart attempts

    stepper.setup(chain, target, q0, criteria);
    run_stepper(stepper, chain, 10000);

    // Warm-start lambda should help convergence -- just verify it does not crash
    // and that iterations are tracked correctly
    REQUIRE(stepper.iterations() > 0);
}

// ============================================================================
// max_restarts exhaustion
// ============================================================================

TEST_CASE("restart_wrapper max_restarts exhaustion", "[ik][restart]")
{
    auto chain = make_ur5_like_chain();

    // Use unreachable target
    spp::vector3<double> far_trans;
    far_trans << 100.0, 100.0, 100.0;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>>::options opts;
    opts.max_restarts = 0;  // no restarts allowed

    spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>> stepper(opts);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 50;
    criteria.max_total_work_units = 100;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 200);

    // With max_restarts=0, behaves like single-start -- should not converge on unreachable
    REQUIRE(status != spp::ik_status::converged);
    REQUIRE((status == spp::ik_status::stalled ||
             status == spp::ik_status::diverged ||
             status == spp::ik_status::iteration_limit));
}

// ============================================================================
// Iterations is cumulative across restarts
// ============================================================================

TEST_CASE("restart_wrapper iterations is cumulative", "[ik][restart]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 2.5, -1.8, 1.2, -2.0, 1.5, -1.0;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::lm<spp::kinematic_chain<double, 6>>::options inner_opts;
    inner_opts.stall_window = 3;
    inner_opts.stall_threshold = 1e-4;

    spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>>::options opts;
    opts.max_restarts = 10;

    spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>> stepper(
        opts, spp::lm<spp::kinematic_chain<double, 6>>(inner_opts));

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 10;  // very few to force restarts
    criteria.max_total_work_units = 5000;       // matches run_stepper outer bound; permits cumulative billing across many restarts

    stepper.setup(chain, target, q0, criteria);
    run_stepper(stepper, chain, 5000);

    // Cumulative billing: iterations() aggregates inner units across all restart
    // attempts (D-13). When the wrapper triggered at least one restart and the
    // second attempt billed additional inner units, the total exceeds the
    // per-attempt cap. criteria.max_iterations_per_attempt is per-attempt, not global.
    REQUIRE(stepper.iterations() > criteria.max_iterations_per_attempt);
}

// ============================================================================
// Abort propagates to inner stepper
// ============================================================================

TEST_CASE("restart_wrapper abort propagates", "[ik][restart]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;

    stepper.setup(chain, target, q0, criteria);

    // Just verify abort() compiles and doesn't crash
    stepper.abort();
}

// ============================================================================
// Composes with racing_scheduler (compile check)
// ============================================================================

TEST_CASE("restart_wrapper composes with racing_scheduler", "[ik][restart]")
{
    // This is a compile-time check: racing_scheduler accepts restart_wrapper
    // via its Solver template parameter. racing_scheduler uses basic_ik_solver<..., Stepper>
    // so we check that basic_ik_solver accepts restart_wrapper.
    using restart_type = spp::restart_wrapper<spp::kinematic_chain<double, 6>, spp::lm<spp::kinematic_chain<double, 6>>>;
    static_assert(spp::solve_policy<restart_type>);

    // Instantiation check: just create the type alias to verify it compiles
    using solver_type = spp::basic_ik_runner<restart_type>;
    solver_type solver;
    (void)solver;
}
