#include <liepp/ik/ik_types.h>
#include <liepp/ik/limits_policy.h>
#include <liepp/ik/basic_ik_solver.h>
#include <liepp/ik/lbfgsb_solve_policy.h>
#include <liepp/ik/restart_solve_policy.h>
#include <liepp/ik/projected_lm_solve_policy.h>

#include <liepp/types.h>

#include <liepp/lie/se3.h>
#include <liepp/lie/so3.h>

#include <liepp/chain/screw_axis.h>
#include <liepp/chain/joint_state.h>
#include <liepp/chain/joint_limits.h>
#include <liepp/chain/kinematic_chain.h>

#include <liepp/kinematics/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <concepts>
#include <numbers>

namespace spp = liepp;

// Local aliases matching default_solvers.h definitions (avoiding inclusion of
// default_solvers.h which transitively includes the not-yet-updated
// dual_racing_scheduler.h). Plan 02 will fix default_solvers.h.
template <typename Scalar = double, int N = spp::dynamic>
using speed_solver = spp::restart_solve_policy<Scalar, N,
    spp::projected_lm_solve_policy<Scalar, N, spp::no_limits>, spp::no_limits>;

template <typename Scalar = double, int N = spp::dynamic>
using convergence_solver = spp::restart_solve_policy<Scalar, N,
    spp::lbfgsb_solve_policy<Scalar, N, spp::no_limits>, spp::no_limits>;

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
// Helper: generate reachable FK target from known configuration
// ============================================================================

static spp::se3<double> reachable_target(
    const spp::kinematic_chain<double, 6>& chain,
    const Eigen::Vector<double, 6>& q)
{
    return spp::forward_kinematics(chain, q).end_effector;
}

// ============================================================================
// Two-policy solver compiles and converges
// ============================================================================

TEST_CASE("two-policy solver compiles and converges", "[ik][variadic_solver]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    spp::basic_ik_solver solver{
        speed_solver<double, 6>{},
        convergence_solver<double, 6>{}
    };

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}

// ============================================================================
// Single-policy solver still works
// ============================================================================

TEST_CASE("single-policy solver still works", "[ik][variadic_solver]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    spp::basic_ik_solver solver{speed_solver<double, 6>{}};

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}

// ============================================================================
// step() returns running then converged
// ============================================================================

TEST_CASE("step() returns running then converged", "[ik][variadic_solver]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    spp::basic_ik_solver solver{
        speed_solver<double, 6>{},
        convergence_solver<double, 6>{}
    };

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};
    solver.setup(chain, target, q0, criteria);

    bool saw_running = false;
    spp::ik_status final_status = spp::ik_status::running;

    for (int i = 0; i < 1000; ++i)
    {
        auto s = solver.step();
        if (s == spp::ik_status::running)
        {
            saw_running = true;
        }
        else
        {
            final_status = s;
            break;
        }
    }

    REQUIRE(saw_running);
    REQUIRE(final_status == spp::ik_status::converged);
}

// ============================================================================
// step_n runs multiple rounds
// ============================================================================

TEST_CASE("step_n runs multiple rounds", "[ik][variadic_solver]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    spp::basic_ik_solver solver{
        speed_solver<double, 6>{},
        convergence_solver<double, 6>{}
    };

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};
    solver.setup(chain, target, q0, criteria);

    solver.step_n(5);
    REQUIRE(solver.iterations() >= 5);
}

// ============================================================================
// speed objective stops on first convergence
// ============================================================================

TEST_CASE("speed objective stops on first convergence", "[ik][variadic_solver]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    spp::basic_ik_solver solver{
        speed_solver<double, 6>{},
        convergence_solver<double, 6>{}
    };

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};
    spp::solver_options<double> opts{.objective = spp::ik_objective::speed};
    solver.setup(chain, target, q0, criteria, opts);

    auto result = solver.solve();
    REQUIRE(result.has_value());
    REQUIRE((result->solver_index == 0 || result->solver_index == 1));
}

// ============================================================================
// solver_options halton_seed affects results
// ============================================================================

TEST_CASE("solver_options halton_seed affects results", "[ik][variadic_solver]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    // Run with halton_seed = 42
    spp::basic_ik_solver solver_a{
        speed_solver<double, 6>{},
        convergence_solver<double, 6>{}
    };
    spp::solver_options<double> opts_a{.halton_seed = 42};
    solver_a.setup(chain, target, q0, criteria, opts_a);
    auto result_a = solver_a.solve();

    // Run with halton_seed = 99
    spp::basic_ik_solver solver_b{
        speed_solver<double, 6>{},
        convergence_solver<double, 6>{}
    };
    spp::solver_options<double> opts_b{.halton_seed = 99};
    solver_b.setup(chain, target, q0, criteria, opts_b);
    auto result_b = solver_b.solve();

    REQUIRE(result_a.has_value());
    REQUIRE(result_b.has_value());
}

// ============================================================================
// CTAD deduction guide works for two policies
// ============================================================================

TEST_CASE("CTAD deduction guide works for two policies", "[ik][variadic_solver]")
{
    using solver_type = decltype(spp::basic_ik_solver{
        speed_solver<double, 6>{},
        convergence_solver<double, 6>{}
    });

    static_assert(std::same_as<typename solver_type::scalar_type, double>);
    static_assert(solver_type::joints == 6);
}

// ============================================================================
// harder target benefits from racing
// ============================================================================

TEST_CASE("harder target benefits from racing", "[ik][variadic_solver]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 2.5, -1.8, 1.2, -2.0, 1.5, -1.0;
    auto target = reachable_target(chain, q_known);

    spp::basic_ik_solver solver{
        speed_solver<double, 6>{},
        convergence_solver<double, 6>{}
    };

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};
    spp::solver_options<double> opts{.max_total_iterations = 600};
    solver.setup(chain, target, q0, criteria, opts);

    auto result = solver.solve();
    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-3);
}
