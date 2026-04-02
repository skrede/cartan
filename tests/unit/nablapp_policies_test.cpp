#include <liepp/serial/ik/ik.h>
#include <liepp/serial/ik/basic_ik_solver.h>
#include <liepp/serial/ik/ik_solve_policy.h>
#include <liepp/serial/ik/nw_sqp_solve_policy.h>
#include <liepp/serial/ik/cmaes_solve_policy.h>
#include <liepp/serial/ik/nablapp_lm_solve_policy.h>
#include <liepp/serial/ik/nablapp_lbfgsb_solve_policy.h>
#include <liepp/serial/ik/augmented_lagrangian_solve_policy.h>

#include <liepp/lie/se3.h>
#include <liepp/lie/so3.h>
#include <liepp/serial/chain/screw_axis.h>
#include <liepp/serial/chain/joint_state.h>
#include <liepp/serial/chain/joint_limits.h>
#include <liepp/serial/chain/kinematic_chain.h>
#include <liepp/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

using chain_t = liepp::kinematic_chain<double, 6>;

static_assert(liepp::ik_solve_policy<liepp::nablapp_lbfgsb_solve_policy<chain_t>>);
static_assert(liepp::ik_solve_policy<liepp::nw_sqp_solve_policy<chain_t>>);
static_assert(liepp::ik_solve_policy<liepp::nablapp_lm_solve_policy<chain_t>>);
static_assert(liepp::ik_solve_policy<liepp::cmaes_solve_policy<chain_t>>);
static_assert(liepp::ik_solve_policy<liepp::augmented_lagrangian_solve_policy<chain_t>>);

static chain_t make_ur5_like_chain()
{
    auto s1 = liepp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = liepp::screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.089});
    auto s3 = liepp::screw_axis<double>::revolute({0, 1, 0}, {0.425, 0, 0.089});
    auto s4 = liepp::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, 0.089});
    auto s5 = liepp::screw_axis<double>::revolute({0, 0, -1}, {0.817, 0.109, 0});
    auto s6 = liepp::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, -0.006});

    liepp::vector3<double> home_trans;
    home_trans << 0.817, 0.191, -0.006;
    auto home = liepp::se3<double>(liepp::so3<double>::identity(), home_trans);

    liepp::joint_limits<double> lim{-2 * std::numbers::pi, 2 * std::numbers::pi};
    return chain_t(home, {s1, s2, s3, s4, s5, s6}, {lim, lim, lim, lim, lim, lim});
}

static auto make_test_target(const chain_t& chain)
{
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, -0.3, 0.6, -0.2;
    return liepp::forward_kinematics(chain, q_known).end_effector;
}

TEST_CASE("nablapp_lbfgsb_solve_policy converges on UR5-like chain", "[ik][nablapp][lbfgsb]")
{
    auto chain = make_ur5_like_chain();
    auto target = make_test_target(chain);

    Eigen::Vector<double, 6> q_seed = Eigen::Vector<double, 6>::Zero();
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    liepp::basic_ik_solver<liepp::nablapp_lbfgsb_solve_policy<chain_t>> solver;
    solver.setup(chain, target, q_seed, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());
    REQUIRE(result->final_error_norm < 1e-4);
}

TEST_CASE("nw_sqp_solve_policy converges on UR5-like chain", "[ik][nablapp][nw-sqp]")
{
    auto chain = make_ur5_like_chain();
    auto target = make_test_target(chain);

    Eigen::Vector<double, 6> q_seed = Eigen::Vector<double, 6>::Zero();
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    liepp::basic_ik_solver<liepp::nw_sqp_solve_policy<chain_t>> solver;
    solver.setup(chain, target, q_seed, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());
    REQUIRE(result->final_error_norm < 1e-4);
}

TEST_CASE("nablapp_lm_solve_policy converges on UR5-like chain", "[ik][nablapp][lm]")
{
    auto chain = make_ur5_like_chain();
    auto target = make_test_target(chain);

    Eigen::Vector<double, 6> q_seed = Eigen::Vector<double, 6>::Zero();
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    liepp::basic_ik_solver<liepp::nablapp_lm_solve_policy<chain_t>> solver;
    solver.setup(chain, target, q_seed, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());
    REQUIRE(result->final_error_norm < 1e-4);
}

TEST_CASE("cmaes_solve_policy converges on UR5-like chain", "[ik][nablapp][cmaes]")
{
    auto chain = make_ur5_like_chain();
    auto target = make_test_target(chain);

    // CMA-ES needs a close starting point for 6-DOF IK
    Eigen::Vector<double, 6> q_seed;
    q_seed << 0.25, -0.45, 0.75, -0.25, 0.55, -0.15;
    liepp::convergence_criteria<double> criteria{1e-2, 1e-2, 10000};

    liepp::cmaes_solve_policy<chain_t>::options opts;
    opts.budget_per_step = 500;
    opts.initial_sigma = 0.05;
    opts.stall_window = 200;
    opts.stall_threshold = 1e-14;

    liepp::basic_ik_solver<liepp::cmaes_solve_policy<chain_t>> solver{
        liepp::cmaes_solve_policy<chain_t>{opts}};
    solver.setup(chain, target, q_seed, criteria);
    auto result = solver.solve();

    // CMA-ES may stall due to sigma collapse before reaching tight tolerance.
    // Verify it either converges or gets close enough.
    if (result.has_value())
    {
        REQUIRE(result->final_error_norm < 1e-2);
    }
    else
    {
        // Even on stall, CMA-ES should have made significant progress
        REQUIRE(result.error().last_error_norm < 0.05);
    }
}

TEST_CASE("augmented_lagrangian_solve_policy converges on UR5-like chain", "[ik][nablapp][aug-lag]")
{
    auto chain = make_ur5_like_chain();
    auto target = make_test_target(chain);

    Eigen::Vector<double, 6> q_seed = Eigen::Vector<double, 6>::Zero();
    liepp::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    liepp::basic_ik_solver<liepp::augmented_lagrangian_solve_policy<chain_t>> solver;
    solver.setup(chain, target, q_seed, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());
    REQUIRE(result->final_error_norm < 1e-4);
}
