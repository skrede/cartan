#include <cartan/serial/ik/ik_validation.h>
#include <cartan/serial/ik/solver/exhaustive_ik_runner.h>

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/solver/projected_lm.h>
#ifdef CARTAN_BUILD_ARGMIN
#include <cartan/serial/ik/solver/argmin_slsqp.h>
#endif

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

namespace spp = cartan;

using ur5_chain = spp::kinematic_chain<double, 6>;
using ur5_plm = spp::projected_lm<ur5_chain>;

static ur5_chain make_ur5_like_chain()
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
    return ur5_chain(home, {s1, s2, s3, s4, s5, s6},
                     {lim, lim, lim, lim, lim, lim});
}

TEST_CASE("verify_solution validates FK-consistent configurations", "[ik][validation]")
{
    auto chain = make_ur5_like_chain();
    Eigen::Vector<double, 6> q;
    q << 0.5, -0.3, 0.8, -0.2, 0.4, 0.1;

    auto fk = spp::forward_kinematics(chain, q);
    auto target = fk.end_effector;

    spp::convergence_criteria<double> criteria;
    criteria.position_tol = 1e-4;
    criteria.orientation_tol = 1e-4;

    SECTION("valid solution")
    {
        REQUIRE(spp::verify_solution(chain, target, q, criteria));
    }

    SECTION("invalid solution - perturbed q")
    {
        Eigen::Vector<double, 6> q_bad = q.array() + 1.0;
        REQUIRE_FALSE(spp::verify_solution(chain, target, q_bad, criteria));
    }
}

TEST_CASE("filter_valid_solutions removes invalid entries", "[ik][validation]")
{
    auto chain = make_ur5_like_chain();
    Eigen::Vector<double, 6> q_good;
    q_good << 0.5, -0.3, 0.8, -0.2, 0.4, 0.1;

    auto fk = spp::forward_kinematics(chain, q_good);
    auto target = fk.end_effector;

    spp::convergence_criteria<double> criteria;
    criteria.position_tol = 1e-4;
    criteria.orientation_tol = 1e-4;

    std::vector<spp::ik_result<double, 6>> solutions;

    solutions.push_back(spp::ik_result<double, 6>{
        .solution = spp::joint_state<double, 6>::from_position(q_good),
        .final_error_norm = 1e-8,
        .iterations = 10,
        .solver_index = 0
    });

    Eigen::Vector<double, 6> q_bad1;
    q_bad1 << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0;
    solutions.push_back(spp::ik_result<double, 6>{
        .solution = spp::joint_state<double, 6>::from_position(q_bad1),
        .final_error_norm = 5.0,
        .iterations = 100,
        .solver_index = 1
    });

    Eigen::Vector<double, 6> q_bad2;
    q_bad2 << -2.0, -2.0, -2.0, -2.0, -2.0, -2.0;
    solutions.push_back(spp::ik_result<double, 6>{
        .solution = spp::joint_state<double, 6>::from_position(q_bad2),
        .final_error_norm = 8.0,
        .iterations = 100,
        .solver_index = 2
    });

    auto filtered = spp::filter_valid_solutions(chain, target, std::move(solutions), criteria);
    REQUIRE(filtered.size() == 1);
}

TEST_CASE("exhaustive_ik_runner finds multiple solutions", "[ik][exhaustive]")
{
    auto chain = make_ur5_like_chain();
    Eigen::Vector<double, 6> q_known;
    q_known << 0.5, -0.3, 0.8, -0.2, 0.4, 0.1;

    auto fk = spp::forward_kinematics(chain, q_known);
    auto target = fk.end_effector;

    spp::convergence_criteria<double> criteria;
    criteria.position_tol = 1e-4;
    criteria.orientation_tol = 1e-4;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 400;

    spp::exhaustive_options<double> options;
    options.max_restarts = 50;

    Eigen::Vector<double, 6> seed = Eigen::Vector<double, 6>::Zero();

    spp::exhaustive_ik_runner<ur5_chain, ur5_plm> runner;
    auto result = runner.solve(chain, target, seed, criteria, options);

    REQUIRE(result.solutions.size() >= 1);
    REQUIRE(result.restarts_attempted == 50);

    for (const auto& sol : result.solutions)
    {
        REQUIRE(spp::verify_solution(chain, target, sol.solution.position, criteria));
    }

    for (std::size_t i = 0; i < result.solutions.size(); ++i)
    {
        for (std::size_t j = i + 1; j < result.solutions.size(); ++j)
        {
            double dist = (result.solutions[i].solution.position
                         - result.solutions[j].solution.position).norm();
            REQUIRE(dist >= options.dedup_tolerance);
        }
    }
}

TEST_CASE("exhaustive_ik_runner returns empty for unreachable target", "[ik][exhaustive]")
{
    auto chain = make_ur5_like_chain();
    spp::vector3<double> far_trans;
    far_trans << 100.0, 100.0, 100.0;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    spp::convergence_criteria<double> criteria;
    criteria.position_tol = 1e-4;
    criteria.orientation_tol = 1e-4;
    criteria.max_iterations_per_attempt = 100;
    criteria.max_total_work_units = 200;

    spp::exhaustive_options<double> options;
    options.max_restarts = 20;

    Eigen::Vector<double, 6> seed = Eigen::Vector<double, 6>::Zero();

    spp::exhaustive_ik_runner<ur5_chain, ur5_plm> runner;
    auto result = runner.solve(chain, target, seed, criteria, options);

    REQUIRE(result.solutions.empty());
    REQUIRE(result.restarts_attempted == 20);
}

TEST_CASE("exhaustive_ik_runner dedup removes near-identical solutions", "[ik][exhaustive]")
{
    auto chain = make_ur5_like_chain();
    Eigen::Vector<double, 6> q_known;
    q_known << 0.5, -0.3, 0.8, -0.2, 0.4, 0.1;

    auto fk = spp::forward_kinematics(chain, q_known);
    auto target = fk.end_effector;

    spp::convergence_criteria<double> criteria;
    criteria.position_tol = 1e-4;
    criteria.orientation_tol = 1e-4;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 400;

    Eigen::Vector<double, 6> seed = Eigen::Vector<double, 6>::Zero();
    spp::exhaustive_ik_runner<ur5_chain, ur5_plm> runner;

    spp::exhaustive_options<double> loose_opts;
    loose_opts.max_restarts = 50;
    loose_opts.dedup_tolerance = 0.5;
    auto result_loose = runner.solve(chain, target, seed, criteria, loose_opts);

    spp::exhaustive_options<double> tight_opts;
    tight_opts.max_restarts = 50;
    tight_opts.dedup_tolerance = 1e-6;
    auto result_tight = runner.solve(chain, target, seed, criteria, tight_opts);

    REQUIRE(result_tight.solutions.size() >= result_loose.solutions.size());
    REQUIRE(result_loose.solutions_before_dedup >= static_cast<int>(result_loose.solutions.size()));
}

TEST_CASE("exhaustive_ik_runner ranking strategies", "[ik][exhaustive]")
{
    auto chain = make_ur5_like_chain();
    Eigen::Vector<double, 6> q_known;
    q_known << 0.5, -0.3, 0.8, -0.2, 0.4, 0.1;

    auto fk = spp::forward_kinematics(chain, q_known);
    auto target = fk.end_effector;

    spp::convergence_criteria<double> criteria;
    criteria.position_tol = 1e-4;
    criteria.orientation_tol = 1e-4;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 400;

    Eigen::Vector<double, 6> seed = Eigen::Vector<double, 6>::Zero();
    spp::exhaustive_ik_runner<ur5_chain, ur5_plm> runner;

    SECTION("distance_to_seed")
    {
        spp::exhaustive_options<double> opts;
        opts.max_restarts = 50;
        opts.ranking = spp::ranking_strategy::distance_to_seed;
        auto result = runner.solve(chain, target, seed, criteria, opts);

        if (result.solutions.size() >= 2)
        {
            double d0 = (result.solutions[0].solution.position - seed).norm();
            double d1 = (result.solutions[1].solution.position - seed).norm();
            REQUIRE(d0 <= d1);
        }
    }

    SECTION("min_error")
    {
        spp::exhaustive_options<double> opts;
        opts.max_restarts = 50;
        opts.ranking = spp::ranking_strategy::min_error;
        auto result = runner.solve(chain, target, seed, criteria, opts);

        if (result.solutions.size() >= 2)
        {
            REQUIRE(result.solutions[0].final_error_norm
                 <= result.solutions[1].final_error_norm);
        }
    }

    SECTION("mid_range")
    {
        spp::exhaustive_options<double> opts;
        opts.max_restarts = 50;
        opts.ranking = spp::ranking_strategy::mid_range;
        auto result = runner.solve(chain, target, seed, criteria, opts);

        if (result.solutions.size() >= 2)
        {
            Eigen::Vector<double, 6> mid;
            for (int j = 0; j < 6; ++j)
            {
                auto lim = chain.limits()[static_cast<std::size_t>(j)];
                mid[j] = (lim.position_min + lim.position_max) / 2.0;
            }
            double d0 = (result.solutions[0].solution.position - mid).norm();
            double d1 = (result.solutions[1].solution.position - mid).norm();
            REQUIRE(d0 <= d1);
        }
    }
}

#ifdef CARTAN_BUILD_ARGMIN

using ur5_argmin_slsqp = spp::argmin_slsqp<ur5_chain>;

TEST_CASE("exhaustive_ik_runner<argmin_slsqp> concept satisfaction and solve", "[ik][exhaustive][argmin]")
{
    auto chain = make_ur5_like_chain();
    Eigen::Vector<double, 6> q_known;
    q_known << 0.5, -0.3, 0.8, -0.2, 0.4, 0.1;

    auto fk = spp::forward_kinematics(chain, q_known);
    auto target = fk.end_effector;

    spp::convergence_criteria<double> criteria;
    criteria.position_tol = 1e-4;
    criteria.orientation_tol = 1e-4;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 400;

    spp::exhaustive_options<double> options;
    options.max_restarts = 20;

    Eigen::Vector<double, 6> seed = Eigen::Vector<double, 6>::Zero();

    spp::exhaustive_ik_runner<ur5_chain, ur5_argmin_slsqp> runner;
    auto result = runner.solve(chain, target, seed, criteria, options);

    REQUIRE(result.restarts_attempted == 20);
    REQUIRE(result.solutions.size() >= 1);

    for (const auto& sol : result.solutions)
    {
        REQUIRE(spp::verify_solution(chain, target, sol.solution.position, criteria));
    }
}

#endif
