// Every backend-gated solve policy and problem adapter, instantiated for a
// fixed-size and a dynamic chain.
//
// A class template that is only included has its dependent expressions parsed
// and not type-checked, so a header no translation unit instantiates is a
// header no compiler has checked. Two of these policies reached no translation
// unit at all and six more were only ever instantiated for one of the two chain
// forms; this target exists so that neither can happen silently again.
//
// It drives each policy through one setup() and one step() rather than to
// convergence: instantiation is what is being asserted, not solution quality,
// which the per-solver targets cover.

#include "../support/kinematics_helpers.h"
#include "../support/joint_limits_helpers.h"

// Named one by one rather than through the ik.h umbrella: the umbrella carries
// eleven of the thirteen, and the two it omits are exactly the two no
// translation unit reached.
#include <cartan/serial/ik/solver/mma.h>
#include <cartan/serial/ik/solver/cmaes.h>
#include <cartan/serial/ik/solver/gcmma.h>
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
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = cartan;

namespace
{

using chain6 = spp::kinematic_chain<double, 6>;
using chain_dyn = spp::kinematic_chain<double, spp::dynamic>;

chain6 make_ur5_like()
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

    auto lim = spp::testing::limits(-2 * std::numbers::pi, 2 * std::numbers::pi);
    return chain6(home, {s1, s2, s3, s4, s5, s6}, {lim, lim, lim, lim, lim, lim});
}

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
    drive<spp::cmaes<Chain>>(chain, target);
    drive<spp::filter_nw_sqp<Chain>>(chain, target);
    drive<spp::filter_slsqp<Chain>>(chain, target);
    drive<spp::gcmma<Chain>>(chain, target);
    drive<spp::mma<Chain>>(chain, target);
    drive<spp::nw_sqp<Chain>>(chain, target);
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
    exercise(spp::detail::argmin_constrained_ik_problem<Chain>(chain, target, weight), n);
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
    auto fixed = make_ur5_like();
    auto dynamic = fixed.to_dynamic();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.2, -0.3, 0.4, 0.1, -0.2, 0.3;
    auto target = spp::testing::fk_at(fixed, q_known).end_effector;

    drive_every_policy(fixed, target);
    drive_every_policy(dynamic, target);
}

TEST_CASE("every backend-gated adapter instantiates for both chain forms",
    "[argmin][instantiation]")
{
    auto fixed = make_ur5_like();
    auto dynamic = fixed.to_dynamic();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.2, -0.3, 0.4, 0.1, -0.2, 0.3;
    auto target = spp::testing::fk_at(fixed, q_known).end_effector;

    drive_every_adapter(fixed, target);
    drive_every_adapter(dynamic, target);
}
