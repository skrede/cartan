// Two decompositions on the solve path run a JacobiSVD over a Jacobian whose
// six rows are fixed at compile time and whose columns are not. A thin left
// factor is then sized from the row count and cannot be resized to the
// instance's column count, so such a decomposition answers only for a chain
// carrying at least six joints. What is driven here is that both answer at the
// joint count the instance actually holds, down to one, and that neither is
// built at the count that leaves the Jacobian without columns.

#include "../fixtures/chain_factories.h"
#include "../support/kinematics_helpers.h"

#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/solver/newton_raphson.h>

#include <cartan/serial/ik/policy/limits_policy.h>

#include <cartan/serial/ik/detail/setup_validation.h>

#include <cartan/serial/chain/joint_state.h>

#include <catch2/catch_template_test_macros.hpp>

#include <vector>

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

/// The two joint counts below the smallest fixture: one, the narrowest the
/// decomposition answers at, and none, at which building one is itself the fault.
template <typename Scalar>
spp::kinematic_chain<Scalar, spp::dynamic> make_short_chain(int joints)
{
    using vec3 = spp::vector3<Scalar>;

    std::vector<spp::screw_axis<Scalar>> axes;
    std::vector<spp::joint_limits<Scalar>> bounds;
    for (int i = 0; i < joints; ++i)
    {
        axes.push_back(spp::screw_axis<Scalar>::revolute(
            vec3(Scalar(0), Scalar(0), Scalar(1)), vec3(Scalar(i), Scalar(0), Scalar(0))));
        bounds.push_back(spp::testing::limits(Scalar(-3), Scalar(3)));
    }
    auto home = spp::se3<Scalar>(
        spp::so3<Scalar>::identity(), vec3(Scalar(joints), Scalar(0), Scalar(0)));
    return spp::kinematic_chain<Scalar, spp::dynamic>(home, std::move(axes), std::move(bounds));
}

template <typename Policy, typename Chain>
spp::ik_status setup_then_step(
    const Chain& chain, const spp::se3<typename Chain::scalar_type>& target, int units = 1)
{
    using scalar_type = typename Chain::scalar_type;

    spp::convergence_criteria<scalar_type> criteria;
    criteria.max_iterations_per_attempt = 2;
    criteria.max_total_work_units = 4;

    Policy policy;
    policy.setup(chain, target, spread(chain, scalar_type(0.1)), criteria);
    spp::step_result<scalar_type> stepped = policy.step(chain, units);

    CHECK(policy.status() == stepped.status);
    CHECK(stepped.status != spp::ik_status::not_initialized);
    return stepped.status;
}

/// The seed and the target are taken at different configurations deliberately:
/// a policy handed the pose it already stands at converges on entry and returns
/// before it ever builds the decomposition under test.
template <typename Policy, typename Chain>
spp::ik_status setup_then_step(const Chain& chain)
{
    using scalar_type = typename Chain::scalar_type;
    return setup_then_step<Policy>(
        chain, spp::testing::fk_at(chain, spread(chain, scalar_type(0.3))).end_effector);
}

/// A run refused before it began never reaches the decomposition, so a case
/// meaning to drive one says so rather than accepting any terminal status.
template <typename Chain>
void step_damped_least_squares(const Chain& chain)
{
    CHECK_FALSE(spp::detail::is_precondition_failure(setup_then_step<spp::dls<Chain>>(chain)));
}

template <typename Chain>
void step_with_null_space_enforcement(const Chain& chain)
{
    CHECK_FALSE(spp::detail::is_precondition_failure(
        setup_then_step<spp::newton_raphson<Chain, spp::null_space_limits>>(chain)));
}

/// A chain of no joints holds one pose and no others, so a target read off a
/// configuration of it is the pose the run already stands at and the run
/// converges on entry. Displacing the target carries these cases as far as the
/// decomposition, where one site certifies the one-point workspace cannot hold
/// the target and the other has no joint to project and runs its budget out.
template <typename Policy, typename Chain>
void step_away_from_the_only_pose(const Chain& chain, int units, spp::ik_status expected)
{
    using scalar_type = typename Chain::scalar_type;

    const spp::se3<scalar_type> away(
        spp::so3<scalar_type>::identity(),
        spp::vector3<scalar_type>(scalar_type(0.5), scalar_type(0.4), scalar_type(0.3)));

    CHECK(setup_then_step<Policy>(chain, away, units) == expected);
}

}

TEMPLATE_TEST_CASE("damped least squares steps over a dynamic chain of fewer than six joints",
    "[dls][dynamic][svd]", double, float)
{
    step_damped_least_squares(spp::fixtures::make_3r_planar_chain<TestType>().to_dynamic());
}

TEMPLATE_TEST_CASE("damped least squares answers at one joint and certifies at none",
    "[dls][dynamic][svd]", double, float)
{
    using chain_type = spp::kinematic_chain<TestType, spp::dynamic>;

    step_damped_least_squares(make_short_chain<TestType>(1));
    step_away_from_the_only_pose<spp::dls<chain_type>>(
        make_short_chain<TestType>(0), 1, spp::ik_status::unreachable);
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

TEMPLATE_TEST_CASE("null-space enforcement answers at one joint and is skipped at none",
    "[limits][dynamic][svd]", double, float)
{
    using chain_type = spp::kinematic_chain<TestType, spp::dynamic>;

    step_with_null_space_enforcement(make_short_chain<TestType>(1));
    step_away_from_the_only_pose<spp::newton_raphson<chain_type, spp::null_space_limits>>(
        make_short_chain<TestType>(0), 4, spp::ik_status::iteration_limit);
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
