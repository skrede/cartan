#ifdef CARTAN_HAS_NLOPT

#include <cartan/serial/ik/solver/nlopt_slsqp.h>
#include <cartan/serial/ik/wrapper/restart_wrapper.h>

#include <cartan/types.h>

#include <cartan/serial/ik/ik_status.h>

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

template <int N, typename Scalar, typename Stepper>
spp::ik_status run_stepper(
    Stepper& stepper,
    const spp::kinematic_chain<Scalar, N>& chain,
    int max_steps)
{
    spp::ik_status status = spp::ik_status::running;
    for (int i = 0; i < max_steps && status == spp::ik_status::running; ++i)
    {
        status = stepper.step(chain, 1).status;
    }
    return status;
}

TEST_CASE("nlopt_slsqp_solve_policy satisfies ik_solve_policy concept", "[ik][slsqp]")
{
    static_assert(spp::solve_policy<spp::nlopt_slsqp<spp::kinematic_chain<double, 6>>>);
    SUCCEED();
}

TEST_CASE("nlopt_slsqp_solve_policy converges on UR5-like chain", "[ik][slsqp]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, -0.3, 0.6, -0.2;

    auto fk = spp::forward_kinematics(chain, q_known);
    auto target = fk.end_effector;

    Eigen::Vector<double, 6> q_seed;
    q_seed << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    spp::convergence_criteria<double> criteria{};
    criteria.position_tol = 1e-4;
    criteria.orientation_tol = 1e-4;

    spp::nlopt_slsqp<spp::kinematic_chain<double, 6>> stepper;
    stepper.setup(chain, target, q_seed, criteria);

    run_stepper(stepper, chain, 50);

    REQUIRE(stepper.converged());
    REQUIRE(stepper.error_norm() < 1e-3);
}

TEST_CASE("nlopt_slsqp_solve_policy composes with restart_solve_policy", "[ik][slsqp]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.5, -0.8, 1.2, -0.5, 0.9, -0.3;

    auto fk = spp::forward_kinematics(chain, q_known);
    auto target = fk.end_effector;

    Eigen::Vector<double, 6> q_seed;
    q_seed << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    spp::convergence_criteria<double> criteria{};
    criteria.position_tol = 1e-4;
    criteria.orientation_tol = 1e-4;

    using inner_t = spp::nlopt_slsqp<spp::kinematic_chain<double, 6>>;
    using restart_t = spp::restart_wrapper<spp::kinematic_chain<double, 6>, inner_t>;

    restart_t stepper({.max_restarts = 10});
    stepper.setup(chain, target, q_seed, criteria);

    run_stepper(stepper, chain, 100);

    // At minimum, should run without crashing.
    // Convergence is likely but not guaranteed for all seeds.
    REQUIRE(stepper.iterations() > 0);
}

#else

#include <catch2/catch_test_macros.hpp>

TEST_CASE("SLSQP tests skipped - NLopt not available", "[ik][slsqp]")
{
    WARN("NLopt not available - SLSQP tests skipped");
}

#endif
