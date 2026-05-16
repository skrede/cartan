#include <cartan/urdf.h>

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/solver/lbfgsb.h>
#include <cartan/serial/ik/solver/filter_nw_sqp.h>
#include <cartan/serial/ik/solver/filter_slsqp.h>
#include <cartan/serial/ik/wrapper/restart_wrapper.h>
#include <cartan/serial/ik/solver/detail/halton_seed_generator.h>

#ifdef CARTAN_TEST_HAVE_ARGMIN
#include <cartan/serial/ik/solver/argmin_slsqp.h>
#include <cartan/serial/ik/solver/argmin_projected_gn.h>
#include <cartan/serial/ik/solver/argmin_projected_gradient_gn.h>
#endif

#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <cartan/lie/se3.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <random>
#include <vector>
#include <string>
#include <filesystem>

namespace
{

std::filesystem::path fixture_path(const char* name)
{
    return std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / name;
}

using chain_t = cartan::kinematic_chain<double, cartan::dynamic>;

chain_t load_continuous_wrist_chain()
{
    auto result = cartan::load_urdf<double>(fixture_path("extractor_continuous_wrist.urdf"));
    REQUIRE(result.has_value());
    return std::move(result->chain);
}

/// Generate a reachable target by running FK on a hand-picked configuration
/// the solver should recover. Using a deterministic q_known keeps the test
/// reproducible across CI runners.
template <typename Solver>
void verify_continuous_wrist_roundtrip(unsigned seed_offset)
{
    auto chain = load_continuous_wrist_chain();

    Eigen::Vector<double, Eigen::Dynamic> q_known(chain.num_joints());
    q_known << 0.4, -0.6, 0.9;
    auto fk_target = cartan::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    Solver solver{};
    Eigen::Vector<double, Eigen::Dynamic> q0(chain.num_joints());
    q0.setZero();

    cartan::convergence_criteria<double> criteria{};
    criteria.position_tol = 1e-7;
    criteria.orientation_tol = 1e-7;
    criteria.max_iterations_per_attempt = 400;
    criteria.max_total_work_units = 2000;

    solver.setup(chain, target, q0, criteria);
    // Drain the solver until it terminates. We poll the step_result.status
    // rather than a solver-side accessor because restart_wrapper exposes
    // status only through its step_result.
    cartan::ik_status status = cartan::ik_status::running;
    int safety = 4000;
    while (status == cartan::ik_status::running && safety-- > 0)
    {
        status = solver.step(chain, 1).status;
    }

    REQUIRE(solver.converged());

    auto fk_sol = cartan::forward_kinematics(chain, solver.solution());
    const auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.template head<3>().norm() < 1e-5);
    REQUIRE(err.template tail<3>().norm() < 1e-5);
    (void)seed_offset;
}

}

// Solver-tag aliases for TEMPLATE_TEST_CASE. Each tag instantiates a different
// solver that exercises one or more of the patched +/-inf call sites.

using restart_lm = cartan::ik::restart_wrapper<chain_t,
    cartan::ik::builtin_lm<chain_t, cartan::no_limits>, cartan::no_limits>;
using restart_lbfgsb = cartan::ik::restart_wrapper<chain_t,
    cartan::ik::builtin_lbfgsb<chain_t, cartan::no_limits>, cartan::no_limits>;

using filter_nw_sqp_solver = cartan::ik::filter_nw_sqp<chain_t>;
using filter_slsqp_solver = cartan::ik::filter_slsqp<chain_t>;

#ifdef CARTAN_TEST_HAVE_ARGMIN
using argmin_slsqp_solver = cartan::ik::argmin_slsqp<chain_t>;
using argmin_projected_gn_solver = cartan::ik::argmin_projected_gn<chain_t>;
using argmin_projected_gradient_gn_solver = cartan::ik::argmin_projected_gradient_gn<chain_t>;
#endif

TEMPLATE_TEST_CASE("continuous joint: IK roundtrip via builtin restart paths",
                   "[solver_continuous_joint]",
                   restart_lm, restart_lbfgsb,
                   filter_nw_sqp_solver, filter_slsqp_solver)
{
    verify_continuous_wrist_roundtrip<TestType>(0);
}

#ifdef CARTAN_TEST_HAVE_ARGMIN
TEMPLATE_TEST_CASE("continuous joint: IK roundtrip via argmin solvers",
                   "[solver_continuous_joint]",
                   argmin_slsqp_solver,
                   argmin_projected_gn_solver,
                   argmin_projected_gradient_gn_solver)
{
    verify_continuous_wrist_roundtrip<TestType>(0);
}
#endif

TEST_CASE("continuous joint: Halton seed generator produces finite seeds",
          "[solver_continuous_joint]")
{
    auto chain = load_continuous_wrist_chain();
    cartan::halton_seed_generator<chain_t> gen{chain};

    for (int i = 0; i < 100; ++i)
    {
        auto q = gen(i);
        REQUIRE(q.size() == chain.num_joints());
        for (int j = 0; j < q.size(); ++j)
        {
            REQUIRE(std::isfinite(q[j]));
        }
    }
}
