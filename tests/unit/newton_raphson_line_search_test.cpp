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

// The control: with the box wide enough that the projection never binds, the
// trajectory is the one the uncorrected line search produced, iteration for
// iteration. This is what confines the correction to the projection interaction.
TEST_CASE("newton_raphson converges unchanged when the limits never bind",
    "[ik][newton_raphson][line_search]")
{
    constexpr int k_control_steps = 6;
    constexpr double k_control_error = 9.7627407778e-07;

    auto chain = bounded_ur3e(2 * std::numbers::pi);
    policy_type stepper;
    stepper.setup(chain, walked_target(chain), Eigen::Vector<double, 6>::Zero(), budget(60));

    auto walked = walk(stepper, chain, 60);

    INFO("converged in " << walked.steps << " steps at error " << walked.final_error);
    REQUIRE(walked.status == cartan::ik_status::converged);
    REQUIRE(walked.rises == 0);
    REQUIRE(walked.steps == k_control_steps);
    REQUIRE(std::abs(walked.final_error - k_control_error) < 1e-17);
}
