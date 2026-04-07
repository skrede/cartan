#include <cartan/serial/ik/solver/detail/analytical_gradient.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = cartan;
using Catch::Approx;

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
// ik_se3_objective at target (identity error -> objective ~0)
// ============================================================================

TEST_CASE("ik_se3_objective at target", "[ik][analytical_gradient]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q;
    q << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk = spp::forward_kinematics(chain, q);
    auto target = fk.end_effector;

    auto result = spp::ik_se3_objective<spp::kinematic_chain<double, 6>>::evaluate(chain, target, q);

    REQUIRE(result.objective < 1e-20);
    REQUIRE(result.body_error.norm() < 1e-10);
    REQUIRE(result.weighted_error.norm() < 1e-10);
}

// ============================================================================
// ik_se3_objective gradient matches finite difference
// ============================================================================

TEST_CASE("ik_se3_objective gradient matches finite difference", "[ik][analytical_gradient]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q;
    q << 0.1, -0.3, 0.5, -0.2, 0.4, -0.1;

    // Target at a different configuration
    Eigen::Vector<double, 6> q_target;
    q_target << 0.4, -0.1, 0.3, 0.2, -0.2, 0.5;
    auto fk_target = spp::forward_kinematics(chain, q_target);
    auto target = fk_target.end_effector;

    auto [info, grad] = spp::ik_se3_objective<spp::kinematic_chain<double, 6>>::evaluate_with_gradient(chain, target, q);

    // Finite-difference gradient
    double eps = 1e-7;
    Eigen::Vector<double, 6> grad_fd;

    for (int i = 0; i < 6; ++i)
    {
        Eigen::Vector<double, 6> q_plus = q;
        Eigen::Vector<double, 6> q_minus = q;
        q_plus(i) += eps;
        q_minus(i) -= eps;

        auto r_plus = spp::ik_se3_objective<spp::kinematic_chain<double, 6>>::evaluate(chain, target, q_plus);
        auto r_minus = spp::ik_se3_objective<spp::kinematic_chain<double, 6>>::evaluate(chain, target, q_minus);

        grad_fd(i) = (r_plus.objective - r_minus.objective) / (2.0 * eps);
    }

    REQUIRE((grad - grad_fd).norm() < 1e-5);
}

// ============================================================================
// ik_se3_objective weighted gradient matches finite difference
// ============================================================================

TEST_CASE("ik_se3_objective weighted gradient matches finite difference", "[ik][analytical_gradient]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q;
    q << 0.1, -0.3, 0.5, -0.2, 0.4, -0.1;

    Eigen::Vector<double, 6> q_target;
    q_target << 0.4, -0.1, 0.3, 0.2, -0.2, 0.5;
    auto fk_target = spp::forward_kinematics(chain, q_target);
    auto target = fk_target.end_effector;

    spp::error_weight<double> w;
    w.weights << 1.0, 1.0, 1.0, 10.0, 10.0, 10.0;

    auto [info_w, grad_w] = spp::ik_se3_objective<spp::kinematic_chain<double, 6>>::evaluate_with_gradient(chain, target, q, w);

    // Weighted gradient should differ from unweighted
    auto [info_u, grad_u] = spp::ik_se3_objective<spp::kinematic_chain<double, 6>>::evaluate_with_gradient(chain, target, q);
    REQUIRE((grad_w - grad_u).norm() > 1e-6);

    // Finite-difference gradient of the weighted objective
    double eps = 1e-7;
    Eigen::Vector<double, 6> grad_fd;

    for (int i = 0; i < 6; ++i)
    {
        Eigen::Vector<double, 6> q_plus = q;
        Eigen::Vector<double, 6> q_minus = q;
        q_plus(i) += eps;
        q_minus(i) -= eps;

        auto r_plus = spp::ik_se3_objective<spp::kinematic_chain<double, 6>>::evaluate(chain, target, q_plus, w);
        auto r_minus = spp::ik_se3_objective<spp::kinematic_chain<double, 6>>::evaluate(chain, target, q_minus, w);

        grad_fd(i) = (r_plus.objective - r_minus.objective) / (2.0 * eps);
    }

    REQUIRE((grad_w - grad_fd).norm() < 1e-5);
}
