#include "../fixtures/chain_factories.h"
#include "../support/joint_limits_helpers.h"

#include <cartan/types.h>

#include <cartan/lie/se3.h>
#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/ik/solver/newton_raphson.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <algorithm>

namespace
{

using chain_type = cartan::kinematic_chain<double, 6>;
using policy_type = cartan::newton_raphson<chain_type>;

chain_type bounded_ur3e(double half_range)
{
    auto base = cartan::fixtures::make_ur3e_chain<double>();
    auto lim = cartan::testing::limits(-half_range, half_range);
    return chain_type(base.home(), base.axes(), {lim, lim, lim, lim, lim, lim});
}

cartan::se3<double> walked_target(const chain_type& chain)
{
    Eigen::Vector<double, 6> q;
    q << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    return cartan::forward_kinematics_unchecked(chain, q).end_effector;
}

cartan::convergence_criteria<double> budget(int iterations)
{
    cartan::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = iterations;
    criteria.max_total_work_units = 2 * iterations;
    return criteria;
}

struct trajectory
{
    int steps;
    int rises;
    double worst_rise;
    double final_error;
    cartan::ik_status status;
};

// Step one iteration at a time and compare each error norm against its
// immediate predecessor: an endpoint comparison would pass a run that goes
// badly wrong and then recovers, which is not the property being pinned.
trajectory walk(policy_type& stepper, const chain_type& chain, int max_steps)
{
    trajectory walked{0, 0, 0.0, stepper.error_norm(), cartan::ik_status::running};
    while (walked.steps < max_steps && walked.status == cartan::ik_status::running)
    {
        walked.status = stepper.step(chain, 1).status;
        ++walked.steps;
        double rise = stepper.error_norm() - walked.final_error;
        walked.rises += (rise > 0.0) ? 1 : 0;
        walked.worst_rise = std::max(walked.worst_rise, rise);
        walked.final_error = stepper.error_norm();
    }
    return walked;
}

}

// Measured before the line search was corrected: the error rose on 58 of the 60
// steps this run takes, drifting away from the target until the iteration budget
// expired. The margin is the whole run, not a threshold.
TEST_CASE("newton_raphson never worsens the error on a bound chain from a zero seed",
    "[ik][newton_raphson][line_search]")
{
    auto chain = bounded_ur3e(0.30);
    policy_type stepper;
    stepper.setup(chain, walked_target(chain), Eigen::Vector<double, 6>::Zero(), budget(60));

    auto walked = walk(stepper, chain, 60);

    INFO("error rose on " << walked.rises << " of " << walked.steps
        << " steps, worst rise " << walked.worst_rise);
    REQUIRE(walked.rises == 0);
}

// Seeding on the upper bounds makes the projection truncate on the very first
// iteration. Measured before the correction: 58 of 60 steps worsened, the worst
// by 1.3e-06 against a converged error of 1e-06.
TEST_CASE("newton_raphson never worsens the error from a seed on the bounds",
    "[ik][newton_raphson][line_search]")
{
    auto chain = bounded_ur3e(0.30);
    policy_type stepper;
    stepper.setup(chain, walked_target(chain), Eigen::Vector<double, 6>::Constant(0.30), budget(60));

    auto walked = walk(stepper, chain, 60);

    INFO("error rose on " << walked.rises << " of " << walked.steps
        << " steps, worst rise " << walked.worst_rise);
    REQUIRE(walked.rises == 0);
}

// A step the unprojected test accepts and the projected test rejects must leave
// the configuration where it was. Before the correction the solver committed
// that rejected trial and ran to its iteration limit instead of stalling.
TEST_CASE("newton_raphson leaves the configuration untouched when nothing is accepted",
    "[ik][newton_raphson][line_search]")
{
    auto chain = bounded_ur3e(0.30);
    policy_type stepper;
    stepper.setup(chain, walked_target(chain), Eigen::Vector<double, 6>::Constant(0.30), budget(60));

    Eigen::Vector<double, 6> before = stepper.solution();
    cartan::ik_status status = cartan::ik_status::running;
    for (int i = 0; i < 60 && status == cartan::ik_status::running; ++i)
    {
        before = stepper.solution();
        status = stepper.step(chain, 1).status;
    }

    REQUIRE(status == cartan::ik_status::stalled);
    REQUIRE((stepper.solution() - before).norm() == 0.0);
}

// The control: the correction lives entirely in the projection, so where the
// projection does not bind it must change nothing.
//
// Stated as an invariance rather than as a remembered trajectory. The line
// search clips each trial to the box unconditionally -- q_trial is
// (q + alpha*dq) clamped to the chain's limits -- so a box no iterate reaches
// leaves q_trial equal to the raw step and the projected Armijo quantity equal
// to the classical one. Two such boxes must therefore produce the same run, and
// the same run here means bit for bit: both are evaluated by one build on one
// machine, so any difference is the projection binding and not the arithmetic
// drifting. An earlier version pinned a converged error captured from the
// pre-correction build to seventeen decimal places, which is a fact about the
// machine that captured it -- it failed on AppleClang and MSVC while passing on
// x86-64 Linux, and no toolchain owed it.
TEST_CASE("newton_raphson converges unchanged when the limits never bind",
    "[ik][newton_raphson][line_search]")
{
    auto narrow = bounded_ur3e(2 * std::numbers::pi);
    policy_type narrow_stepper;
    narrow_stepper.setup(
        narrow, walked_target(narrow), Eigen::Vector<double, 6>::Zero(), budget(60));
    auto narrow_walk = walk(narrow_stepper, narrow, 60);

    auto wide = bounded_ur3e(8 * std::numbers::pi);
    policy_type wide_stepper;
    wide_stepper.setup(wide, walked_target(wide), Eigen::Vector<double, 6>::Zero(), budget(60));
    auto wide_walk = walk(wide_stepper, wide, 60);

    INFO("converged in " << narrow_walk.steps << " steps at error " << narrow_walk.final_error
        << "; the four-times-wider box took " << wide_walk.steps << " steps at error "
        << wide_walk.final_error);

    REQUIRE(narrow_walk.status == cartan::ik_status::converged);
    REQUIRE(narrow_walk.rises == 0);

    REQUIRE(wide_walk.status == narrow_walk.status);
    REQUIRE(wide_walk.steps == narrow_walk.steps);
    REQUIRE(wide_walk.final_error == narrow_walk.final_error);
}
