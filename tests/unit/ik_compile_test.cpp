#include <cartan/serial/ik/lm_solve_policy.h>
#include <cartan/serial/ik/dls_solve_policy.h>

#include <catch2/catch_test_macros.hpp>

namespace spp = cartan;

TEST_CASE("IK types compile", "[ik][compile]")
{
    // Enum values exist
    auto s1 = spp::ik_status::running;
    auto s2 = spp::ik_status::converged;
    auto s3 = spp::ik_status::diverged;
    auto s4 = spp::ik_status::stalled;
    auto s5 = spp::ik_status::joint_limit_hit;
    auto s6 = spp::ik_status::iteration_limit;
    (void)s1; (void)s2; (void)s3; (void)s4; (void)s5; (void)s6;

    auto o1 = spp::ik_objective::speed;
    auto o2 = spp::ik_objective::min_distance;
    auto o3 = spp::ik_objective::max_manipulability;
    auto o4 = spp::ik_objective::max_isotropy;
    (void)o1; (void)o2; (void)o3; (void)o4;

    auto f1 = spp::ik_failure::unreachable;
    auto f2 = spp::ik_failure::diverged;
    auto f3 = spp::ik_failure::stalled;
    auto f4 = spp::ik_failure::iteration_limit;
    auto f5 = spp::ik_failure::joint_limit_violation;
    auto f6 = spp::ik_failure::aborted;
    (void)f1; (void)f2; (void)f3; (void)f4; (void)f5; (void)f6;

    // Convergence criteria fields
    spp::convergence_criteria<double> cc;
    REQUIRE(cc.position_tol > 0);
    REQUIRE(cc.orientation_tol > 0);
    REQUIRE(cc.max_iterations > 0);

    // static_asserts in dls_solve_policy.h and lm_solve_policy.h verify concept satisfaction
    REQUIRE(true);
}
