// Every backend-gated solve policy and problem adapter, driven for a fixed-size
// and a dynamic chain, and for a non-default convergence policy wherever the
// policy exposes one.
//
// A class template that is only included has its dependent expressions parsed
// and not type-checked, so an entry point nothing instantiates is one no
// compiler has checked. That includes a published alias: an alias binds
// template arguments no other instantiation supplies, so it can be ill-formed
// while every default-argument instantiation of the same template compiles.
//
// Each policy is driven through one setup() and one step() rather than to
// convergence: instantiation is what is asserted here, not solution quality,
// which the per-solver targets cover.

#include "../support/kinematics_helpers.h"

#include "../fixtures/chain_factories.h"

// Named one by one rather than through the ik.h umbrella, which does not carry
// all eleven.
#include <cartan/serial/ik/solver/mma.h>
#include <cartan/serial/ik/solver/nw_sqp.h>
#include <cartan/serial/ik/solver/argmin_lm.h>
#include <cartan/serial/ik/solver/argmin_slsqp.h>
#include <cartan/serial/ik/solver/filter_slsqp.h>
#include <cartan/serial/ik/solver/argmin_bobyqa.h>
#include <cartan/serial/ik/solver/argmin_lbfgsb.h>
#include <cartan/serial/ik/solver/filter_nw_sqp.h>
#include <cartan/serial/ik/solver/argmin_projected_gn.h>
#include <cartan/serial/ik/solver/augmented_lagrangian.h>
#include <cartan/serial/ik/solver/argmin_projected_gradient_gn.h>

#include <cartan/serial/ik/detail/argmin_problem.h>
#include <cartan/serial/ik/detail/argmin_bounded_ik_problem.h>
#include <cartan/serial/ik/detail/argmin_constrained_problem.h>
#include <cartan/serial/ik/detail/argmin_least_squares_problem.h>
#include <cartan/serial/ik/detail/argmin_unconstrained_problem.h>

#include <cartan/lie/se3.h>

#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <type_traits>

namespace spp = cartan;

namespace
{

using chain6 = spp::kinematic_chain<double, 6>;

template <typename Policy, typename Chain>
void drive(const Chain& chain, const spp::se3<double>& target)
{
    typename spp::joint_state<double, Chain::joints>::position_type q0
        = spp::joint_state<double, Chain::joints>::position_type::Zero(chain.num_joints());

    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 2;
    criteria.max_total_work_units = 4;

    Policy policy;
    policy.setup(chain, target, q0, criteria);
    auto stepped = policy.step(chain, 1);

    CHECK(policy.status() != spp::ik_status::not_initialized);
    CHECK(stepped.status != spp::ik_status::not_initialized);
    CHECK(policy.error_norm() >= 0.0);
}

template <typename Chain>
void drive_every_policy(const Chain& chain, const spp::se3<double>& target)
{
    drive<spp::argmin_bobyqa<Chain>>(chain, target);
    drive<spp::argmin_lbfgsb<Chain>>(chain, target);
    drive<spp::argmin_lm<Chain>>(chain, target);
    drive<spp::argmin_projected_gn<Chain>>(chain, target);
    drive<spp::argmin_projected_gradient_gn<Chain>>(chain, target);
    drive<spp::argmin_slsqp<Chain>>(chain, target);
    drive<spp::augmented_lagrangian<Chain>>(chain, target);
    drive<spp::filter_nw_sqp<Chain>>(chain, target);
    drive<spp::filter_slsqp<Chain>>(chain, target);
    drive<spp::mma<Chain>>(chain, target);
    drive<spp::nw_sqp<Chain>>(chain, target);
}

/// The five policies below take a Convergence parameter, and the inner solver
/// they build has to be typed on it rather than on its own default. That is a
/// distinct instantiation from the one above and the only one a published alias
/// like argmin_slsqp_nlopt_compat ever produces.
template <typename Chain>
void drive_every_convergence_parameterized(const Chain& chain, const spp::se3<double>& target)
{
    using nlopt_like = argmin::slsqp_compatible_convergence;

    drive<spp::argmin_slsqp<Chain, spp::clamp_limits, nlopt_like>>(chain, target);
    drive<spp::filter_slsqp<Chain, spp::clamp_limits, nlopt_like>>(chain, target);
    drive<spp::filter_nw_sqp<Chain, spp::clamp_limits, nlopt_like>>(chain, target);
    drive<spp::argmin_projected_gn<Chain, spp::clamp_limits, nlopt_like>>(chain, target);
    drive<spp::argmin_projected_gradient_gn<Chain, spp::clamp_limits, nlopt_like>>(chain, target);

    drive<spp::argmin_slsqp_nlopt_compat<Chain>>(chain, target);
}

template <typename Adapter>
void exercise(const Adapter& problem, int expected_dimension)
{
    REQUIRE(problem.dimension() == expected_dimension);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(expected_dimension);
    CHECK(std::isfinite(problem.value(x)));

    Eigen::VectorXd g(expected_dimension);
    problem.gradient(x, g);
    CHECK(g.allFinite());
}

template <typename Chain>
void drive_every_adapter(const Chain& chain, const spp::se3<double>& target)
{
    spp::error_weight<double> weight;
    const int n = chain.num_joints();

    exercise(spp::detail::argmin_ik_problem<Chain>(chain, target, weight), n);
    exercise(spp::detail::argmin_bounded_ik_problem<Chain>(chain, target, weight), n);
    using position_type = typename spp::joint_state<double, Chain::joints>::position_type;
    exercise(spp::detail::argmin_constrained_ik_problem<Chain>(
        chain, target, weight, position_type::Zero(n)), n);
    exercise(spp::detail::argmin_unconstrained_ik_problem<Chain>(chain, target, weight), n);

    spp::detail::argmin_ik_least_squares_problem<Chain> squares(chain, target);
    REQUIRE(squares.num_residuals() == 6);
    Eigen::VectorXd residuals(6);
    squares.residuals(Eigen::VectorXd::Zero(n), residuals);
    CHECK(residuals.allFinite());
}

}

TEST_CASE("every backend-gated policy instantiates for both chain forms",
    "[argmin][instantiation]")
{
    auto fixed = spp::fixtures::make_ur3e_chain<double>();
    auto dynamic = fixed.to_dynamic();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.2, -0.3, 0.4, 0.1, -0.2, 0.3;
    auto target = spp::testing::fk_at(fixed, q_known).end_effector;

    drive_every_policy(fixed, target);
    drive_every_policy(dynamic, target);
}

/// argmin_slsqp_fast is documented as a synonym that survives only so existing
/// call sites resolve; asserting the identity resolves the alias without
/// building a second copy of a type already driven above.
static_assert(std::is_same_v<spp::argmin_slsqp_fast<chain6>, spp::argmin_slsqp<chain6>>);

TEST_CASE("every convergence-parameterized policy and published alias instantiates",
    "[argmin][instantiation]")
{
    auto fixed = spp::fixtures::make_ur3e_chain<double>();
    auto dynamic = fixed.to_dynamic();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.2, -0.3, 0.4, 0.1, -0.2, 0.3;
    auto target = spp::testing::fk_at(fixed, q_known).end_effector;

    drive_every_convergence_parameterized(fixed, target);
    drive_every_convergence_parameterized(dynamic, target);
}

TEST_CASE("every backend-gated adapter instantiates for both chain forms",
    "[argmin][instantiation]")
{
    auto fixed = spp::fixtures::make_ur3e_chain<double>();
    auto dynamic = fixed.to_dynamic();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.2, -0.3, 0.4, 0.1, -0.2, 0.3;
    auto target = spp::testing::fk_at(fixed, q_known).end_effector;

    drive_every_adapter(fixed, target);
    drive_every_adapter(dynamic, target);
}
