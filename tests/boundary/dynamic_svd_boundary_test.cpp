// Two decompositions on the solve path run a JacobiSVD over a Jacobian whose
// six rows are fixed at compile time and whose columns are not. A thin left
// factor is then sized from the row count and cannot be resized to the
// instance's column count, so such a decomposition answers only for a chain
// carrying at least six joints. What is driven here is that both answer at the
// joint count the instance actually holds.

#include "../fixtures/chain_factories.h"
#include "../support/kinematics_helpers.h"

#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/solver/newton_raphson.h>

#include <cartan/serial/ik/policy/limits_policy.h>

#include <cartan/serial/chain/joint_state.h>

#include <catch2/catch_template_test_macros.hpp>

namespace spp = cartan;

namespace
{

template <typename Chain>
using configuration =
    typename spp::joint_state<typename Chain::scalar_type, Chain::joints>::position_type;

template <typename Chain>
configuration<Chain> spread(const Chain& chain, typename Chain::scalar_type step)
{
    using scalar_type = typename Chain::scalar_type;

    configuration<Chain> q = configuration<Chain>::Zero(chain.num_joints());
    for (int i = 0; i < chain.num_joints(); ++i)
    {
        q(i) = step * scalar_type(i + 1);
    }
    return q;
}

/// The seed and the target are taken at different configurations deliberately:
/// a policy handed the pose it already stands at converges on entry and returns
/// before it ever builds the decomposition under test.
template <typename Policy, typename Chain>
void setup_then_step(const Chain& chain)
{
    using scalar_type = typename Chain::scalar_type;

    auto target = spp::testing::fk_at(chain, spread(chain, scalar_type(0.3))).end_effector;

    spp::convergence_criteria<scalar_type> criteria;
    criteria.max_iterations_per_attempt = 2;
    criteria.max_total_work_units = 4;

    Policy policy;
    policy.setup(chain, target, spread(chain, scalar_type(0.1)), criteria);
    spp::step_result<scalar_type> stepped = policy.step(chain, 1);

    CHECK(policy.status() != spp::ik_status::not_initialized);
    CHECK(stepped.status != spp::ik_status::not_initialized);
}

template <typename Chain>
void step_damped_least_squares(const Chain& chain)
{
    setup_then_step<spp::dls<Chain>>(chain);
}

template <typename Chain>
void step_with_null_space_enforcement(const Chain& chain)
{
    setup_then_step<spp::newton_raphson<Chain, spp::null_space_limits>>(chain);
}

}

TEMPLATE_TEST_CASE("damped least squares steps over a dynamic chain of fewer than six joints",
    "[dls][dynamic][svd]", double, float)
{
    step_damped_least_squares(spp::fixtures::make_3r_planar_chain<TestType>().to_dynamic());
}

TEMPLATE_TEST_CASE("damped least squares steps over a dynamic chain of six and of seven joints",
    "[dls][dynamic][svd]", double, float)
{
    step_damped_least_squares(spp::fixtures::make_ur3e_chain<TestType>().to_dynamic());
    step_damped_least_squares(spp::fixtures::make_lbr_med14_chain<TestType>().to_dynamic());
}

TEMPLATE_TEST_CASE("damped least squares steps over a fixed-size chain",
    "[dls][svd]", double, float)
{
    step_damped_least_squares(spp::fixtures::make_3r_planar_chain<TestType>());
    step_damped_least_squares(spp::fixtures::make_ur3e_chain<TestType>());
    step_damped_least_squares(spp::fixtures::make_lbr_med14_chain<TestType>());
}

/// The extended enforcement the null-space policy selects carries the same
/// decomposition and is reached through no solver's default, so it is driven
/// through a policy that builds no decomposition of its own.
TEMPLATE_TEST_CASE("null-space enforcement steps over a dynamic chain of fewer than six joints",
    "[limits][dynamic][svd]", double, float)
{
    step_with_null_space_enforcement(spp::fixtures::make_3r_planar_chain<TestType>().to_dynamic());
}

TEMPLATE_TEST_CASE("null-space enforcement steps over a dynamic chain of six and of seven joints",
    "[limits][dynamic][svd]", double, float)
{
    step_with_null_space_enforcement(spp::fixtures::make_ur3e_chain<TestType>().to_dynamic());
    step_with_null_space_enforcement(spp::fixtures::make_lbr_med14_chain<TestType>().to_dynamic());
}

TEMPLATE_TEST_CASE("null-space enforcement steps over a fixed-size chain",
    "[limits][svd]", double, float)
{
    step_with_null_space_enforcement(spp::fixtures::make_3r_planar_chain<TestType>());
    step_with_null_space_enforcement(spp::fixtures::make_ur3e_chain<TestType>());
    step_with_null_space_enforcement(spp::fixtures::make_lbr_med14_chain<TestType>());
}
