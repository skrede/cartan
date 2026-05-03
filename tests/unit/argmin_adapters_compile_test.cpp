#include "cartan/serial/ik/detail/argmin_problem.h"
#include "cartan/serial/ik/detail/argmin_unconstrained_problem.h"
#include "cartan/serial/ik/detail/argmin_constrained_problem.h"
#include "cartan/serial/ik/detail/argmin_least_squares_problem.h"

#include "cartan/serial/ik/policy/error_weight.h"

#include "cartan/lie/se3.h"
#include "cartan/lie/so3.h"
#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include <argmin/formulation/concepts.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = cartan;

static spp::kinematic_chain<double, 6> make_chain()
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

using chain6 = spp::kinematic_chain<double, 6>;

TEST_CASE("argmin adapters compile and satisfy concepts", "[argmin][compile]")
{
    auto chain = make_chain();
    auto target = spp::se3<double>::identity();
    spp::error_weight<double> weight;

    SECTION("bound-constrained adapter")
    {
        spp::detail::argmin_ik_problem<chain6> problem(chain, target, weight);
        REQUIRE(problem.dimension() == 6);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
        double val = problem.value(x);
        REQUIRE(std::isfinite(val));

        Eigen::VectorXd g(6);
        problem.gradient(x, g);
        REQUIRE(g.size() == 6);

        auto lb = problem.lower_bounds();
        auto ub = problem.upper_bounds();
        REQUIRE(lb.size() == 6);
        REQUIRE(ub.size() == 6);

        static_assert(argmin::objective<spp::detail::argmin_ik_problem<chain6>>);
        static_assert(argmin::differentiable<spp::detail::argmin_ik_problem<chain6>>);
        static_assert(argmin::bound_constrained<spp::detail::argmin_ik_problem<chain6>>);
    }

    SECTION("unconstrained adapter")
    {
        spp::detail::argmin_unconstrained_ik_problem<chain6> problem(chain, target, weight);
        REQUIRE(problem.dimension() == 6);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
        double val = problem.value(x);
        REQUIRE(std::isfinite(val));

        Eigen::VectorXd g(6);
        problem.gradient(x, g);
        REQUIRE(g.size() == 6);

        static_assert(argmin::objective<spp::detail::argmin_unconstrained_ik_problem<chain6>>);
        static_assert(argmin::differentiable<spp::detail::argmin_unconstrained_ik_problem<chain6>>);
        static_assert(!argmin::bound_constrained<spp::detail::argmin_unconstrained_ik_problem<chain6>>);
    }

    SECTION("constrained adapter")
    {
        spp::detail::argmin_constrained_ik_problem<chain6> problem(chain, target, weight);
        REQUIRE(problem.dimension() == 6);
        REQUIRE(problem.num_equality() == 0);
        REQUIRE(problem.num_inequality() == 12);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
        double val = problem.value(x);
        REQUIRE(std::isfinite(val));

        Eigen::VectorXd c(12);
        problem.constraints(x, c);
        REQUIRE(c.size() == 12);

        Eigen::MatrixXd J(12, 6);
        problem.constraint_jacobian(x, J);
        REQUIRE(J.rows() == 12);
        REQUIRE(J.cols() == 6);

        static_assert(argmin::objective<spp::detail::argmin_constrained_ik_problem<chain6>>);
        static_assert(argmin::differentiable<spp::detail::argmin_constrained_ik_problem<chain6>>);
        static_assert(argmin::constrained<spp::detail::argmin_constrained_ik_problem<chain6>>);
    }

    SECTION("least-squares adapter")
    {
        spp::detail::argmin_ik_least_squares_problem<chain6> problem(chain, target);
        REQUIRE(problem.dimension() == 6);
        REQUIRE(problem.num_residuals() == 6);

        Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
        double val = problem.value(x);
        REQUIRE(std::isfinite(val));

        Eigen::VectorXd r(6);
        problem.residuals(x, r);
        REQUIRE(r.size() == 6);

        Eigen::MatrixXd J(6, 6);
        problem.jacobian(x, J);
        REQUIRE(J.rows() == 6);
        REQUIRE(J.cols() == 6);

        static_assert(argmin::objective<spp::detail::argmin_ik_least_squares_problem<chain6>>);
        static_assert(argmin::least_squares<spp::detail::argmin_ik_least_squares_problem<chain6>>);
    }
}
