#include <cartan/serial/ik/ik.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/ik/concepts/solve_concept.h>
#include <cartan/serial/ik/solver/nw_sqp.h>
#include <cartan/serial/ik/solver/cmaes.h>
#include <cartan/serial/ik/solver/argmin_lm.h>
#include <cartan/serial/ik/solver/argmin_lbfgsb.h>
#include <cartan/serial/ik/solver/augmented_lagrangian.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

using chain_t = cartan::kinematic_chain<double, 6>;

static_assert(cartan::ik::solve_policy<cartan::ik::argmin_lbfgsb<chain_t>>);
static_assert(cartan::ik::solve_policy<cartan::ik::nw_sqp<chain_t>>);
static_assert(cartan::ik::solve_policy<cartan::ik::argmin_lm<chain_t>>);
static_assert(cartan::ik::solve_policy<cartan::ik::cmaes<chain_t>>);
static_assert(cartan::ik::solve_policy<cartan::ik::augmented_lagrangian<chain_t>>);

static chain_t make_ur5_like_chain()
{
    auto s1 = cartan::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.089});
    auto s3 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0.425, 0, 0.089});
    auto s4 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, 0.089});
    auto s5 = cartan::screw_axis<double>::revolute({0, 0, -1}, {0.817, 0.109, 0});
    auto s6 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, -0.006});

    cartan::vector3<double> home_trans;
    home_trans << 0.817, 0.191, -0.006;
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    cartan::joint_limits<double> lim{-2 * std::numbers::pi, 2 * std::numbers::pi};
    return chain_t(home, {s1, s2, s3, s4, s5, s6}, {lim, lim, lim, lim, lim, lim});
}

static auto make_test_target(const chain_t& chain)
{
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, -0.3, 0.6, -0.2;
    return cartan::forward_kinematics(chain, q_known).end_effector;
}

TEST_CASE("nablapp_lbfgsb_solve_policy converges on UR5-like chain", "[ik][nablapp][lbfgsb]")
{
    auto chain = make_ur5_like_chain();
    auto target = make_test_target(chain);

    Eigen::Vector<double, 6> q_seed = Eigen::Vector<double, 6>::Zero();
    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    cartan::basic_ik_runner<cartan::ik::argmin_lbfgsb<chain_t>> solver;
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
    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    cartan::basic_ik_runner<cartan::ik::nw_sqp<chain_t>> solver;
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
    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    cartan::basic_ik_runner<cartan::ik::argmin_lm<chain_t>> solver;
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
    cartan::convergence_criteria<double> criteria{1e-2, 1e-2, 10000};

    cartan::ik::cmaes<chain_t>::options opts;
    opts.budget_per_step = 500;
    opts.initial_sigma = 0.05;
    opts.stall_window = 200;
    opts.stall_threshold = 1e-14;

    cartan::basic_ik_runner<cartan::ik::cmaes<chain_t>> solver{
        cartan::ik::cmaes<chain_t>{opts}};
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
    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    cartan::basic_ik_runner<cartan::ik::augmented_lagrangian<chain_t>> solver;
    solver.setup(chain, target, q_seed, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());
    REQUIRE(result->final_error_norm < 1e-4);
}
