#include <cartan/serial/ik/solver/argmin_slsqp.h>

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

TEST_CASE("argmin_slsqp restart_count reaches max_restarts on hard target", "[ik][argmin][slsqp][restart]")
{
    auto chain = make_ur5_like_chain();
    auto target = cartan::se3<double>(cartan::so3<double>::identity(), {10.0, 10.0, 10.0});

    cartan::ik::argmin_slsqp<chain_t>::options opts{};
    opts.max_restarts = 2;
    opts.rng_seed = 42;

    cartan::ik::argmin_slsqp<chain_t> solver{opts};
    Eigen::Vector<double, 6> q_seed = Eigen::Vector<double, 6>::Zero();
    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 500};
    solver.setup(chain, target, q_seed, criteria);

    while (solver.status() == cartan::ik_status::running)
        solver.step(chain);

    REQUIRE(solver.status() == cartan::ik_status::stalled);
    REQUIRE(solver.restart_count() == 2);
}

TEST_CASE("argmin_slsqp zero restarts on easy target", "[ik][argmin][slsqp][restart]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, -0.3, 0.6, -0.2;
    auto target = cartan::forward_kinematics(chain, q_known).end_effector;

    cartan::ik::argmin_slsqp<chain_t> solver{};
    Eigen::Vector<double, 6> q_seed = Eigen::Vector<double, 6>::Zero();
    cartan::convergence_criteria<double> criteria{1e-4, 1e-4, 500};
    solver.setup(chain, target, q_seed, criteria);

    while (solver.status() == cartan::ik_status::running)
        solver.step(chain);

    REQUIRE(solver.converged());
    REQUIRE(solver.restart_count() == 0);
}

TEST_CASE("argmin_slsqp deterministic under fixed RNG seed", "[ik][argmin][slsqp][restart]")
{
    auto chain = make_ur5_like_chain();
    auto target = cartan::se3<double>(cartan::so3<double>::identity(), {10.0, 10.0, 10.0});

    Eigen::Vector<double, 6> q_seed = Eigen::Vector<double, 6>::Zero();
    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    auto run_solver = [&](unsigned seed)
    {
        cartan::ik::argmin_slsqp<chain_t>::options opts{};
        opts.max_restarts = 3;
        opts.rng_seed = seed;

        cartan::ik::argmin_slsqp<chain_t> solver{opts};
        solver.setup(chain, target, q_seed, criteria);

        while (solver.status() == cartan::ik_status::running)
            solver.step(chain);

        return solver;
    };

    auto s1 = run_solver(12345);
    auto s2 = run_solver(12345);

    REQUIRE(s1.restart_count() == s2.restart_count());
    REQUIRE(s1.solution() == s2.solution());

    auto s3 = run_solver(99999);
    REQUIRE(s3.solution() != s1.solution());
}
