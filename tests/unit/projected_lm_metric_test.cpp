#include "../fixtures/chain_factories.h"

#include <cartan/serial/ik/solver/projected_lm.h>

#include <cartan/types.h>
#include <cartan/lie/se3.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/ik/policy/error_weight.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

// The task weight has to change the trust-region step itself, which only the
// first step can show: convergence is a fixed point many different trajectories
// reach, so a solve that ends on the target says nothing about the metric it
// descended under.

namespace
{

using chain_type = cartan::kinematic_chain<double, 6>;
using position_type = cartan::joint_state<double, 6>::position_type;

// One encoder tick on an industrial arm is on the order of 1e-5 rad, so a
// separation an order of magnitude above that is the smallest one a machine
// could act on. Before the step mathematics read the weight the measured
// separation was exactly zero, so this floor discriminates by a wide margin
// rather than by a hair.
constexpr double separation_floor = 1e-4;

chain_type metric_chain()
{
    return cartan::fixtures::make_ur3e_chain<double>();
}

cartan::se3<double> metric_target(const chain_type& chain)
{
    position_type q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    return cartan::forward_kinematics_unchecked(chain, q_known).end_effector;
}

cartan::convergence_criteria<double> metric_criteria()
{
    cartan::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 400;
    return criteria;
}

position_type first_step(const cartan::error_weight<double>& weight)
{
    auto chain = metric_chain();
    cartan::projected_lm<chain_type> stepper;
    stepper.setup(chain, metric_target(chain), position_type::Zero(), metric_criteria(), weight);
    stepper.step(chain, 1);
    return stepper.solution();
}

position_type first_step_through_defaulted_weight()
{
    auto chain = metric_chain();
    cartan::projected_lm<chain_type> stepper;
    stepper.setup(chain, metric_target(chain), position_type::Zero(), metric_criteria());
    stepper.step(chain, 1);
    return stepper.solution();
}

cartan::error_weight<double> orientation_dominant()
{
    cartan::error_weight<double> weight;
    weight.weights << 1e3, 1e3, 1e3, 1e-3, 1e-3, 1e-3;
    return weight;
}

cartan::error_weight<double> position_dominant()
{
    cartan::error_weight<double> weight;
    weight.weights << 1e-3, 1e-3, 1e-3, 1e3, 1e3, 1e3;
    return weight;
}

double separation(const position_type& a, const position_type& b)
{
    return (a - b).norm();
}

}

TEST_CASE("projected_lm first steps differ under opposed task weights", "[ik][projected_lm][metric]")
{
    const auto q_orientation = first_step(orientation_dominant());
    const auto q_position = first_step(position_dominant());

    REQUIRE(separation(q_orientation, q_position) > separation_floor);
}

TEST_CASE("projected_lm first steps differ from the unweighted step", "[ik][projected_lm][metric]")
{
    const auto q_identity = first_step(cartan::error_weight<double>{});
    const auto q_orientation = first_step(orientation_dominant());
    const auto q_position = first_step(position_dominant());

    REQUIRE(separation(q_orientation, q_identity) > separation_floor);
    REQUIRE(separation(q_position, q_identity) > separation_floor);
}

TEST_CASE("projected_lm reproduces a first step under a repeated weight", "[ik][projected_lm][metric]")
{
    REQUIRE(separation(first_step(orientation_dominant()), first_step(orientation_dominant())) == 0.0);
    REQUIRE(separation(first_step(position_dominant()), first_step(position_dominant())) == 0.0);
}

TEST_CASE("projected_lm defaults its task weight to the identity", "[ik][projected_lm][metric]")
{
    REQUIRE(separation(first_step_through_defaulted_weight(), first_step(cartan::error_weight<double>{})) == 0.0);
}
