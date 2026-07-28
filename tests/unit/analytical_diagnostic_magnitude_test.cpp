#include "../support/joint_limits_helpers.h"

#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include "../fixtures/opw_chains.h"
#include "../fixtures/analytical_chains.h"

#include <catch2/catch_test_macros.hpp>

#include <vector>
#include <limits>

using namespace cartan;

namespace
{

using dyn_chain = kinematic_chain<double, dynamic>;
using axis_list = std::vector<screw_axis<double>>;
constexpr double nan_v = std::numeric_limits<double>::quiet_NaN();

screw_axis<double> rev(const Eigen::Vector3d& omega, const Eigen::Vector3d& point)
{
    return screw_axis<double>::revolute(omega, point);
}

dyn_chain chain_of(axis_list axes, const Eigen::Vector3d& end_effector)
{
    std::vector<joint_limits<double>> limits(axes.size(), testing::limits(-10.0, 10.0));
    return dyn_chain(
        se3<double>(so3<double>::identity(), end_effector), std::move(axes), std::move(limits));
}

se3<double> at(double x, double y, double z)
{
    return se3<double>(so3<double>::identity(), Eigen::Vector3d(x, y, z));
}

/// Ortho-parallel spherical-wrist geometry that passes every construction gate,
/// so a single substituted axis isolates the one gate under test.
axis_list opw_axes()
{
    const Eigen::Vector3d z(0, 0, 1), y(0, 1, 0), wrist(0, 0, 1.2);
    return {rev(z, {0, 0, 0}), rev(y, {0, 0, 0.4}), rev(y, {0, 0, 0.8}),
        rev(z, wrist), rev(y, wrist), rev(z, wrist)};
}

axis_list puma_axes()
{
    const Eigen::Vector3d z(0, 0, 1), y(0, 1, 0), wrist(0.7, 0, 0.5);
    return {rev(z, {0, 0, 0}), rev(y, {0, 0, 0.5}), rev(y, {0.4, 0, 0.5}),
        rev(z, wrist), rev(y, wrist), rev(z, wrist)};
}

axis_list with_prismatic_wrist(axis_list axes)
{
    axes.back() = screw_axis<double>::prismatic({0, 0, 1});
    return axes;
}
void absent(const char* site, const auto& result)
{
    INFO(site);
    REQUIRE_FALSE(result.has_value());
    CHECK_FALSE(result.error().workspace_distance.has_value());
}

void deficit(const char* site, const auto& result)
{
    INFO(site);
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error().workspace_distance.has_value());
    CHECK(*result.error().workspace_distance > 0.0);
}

}

TEST_CASE("analytical diagnostics: a 2R failure that evaluated no inequality carries nothing")
{
    const Eigen::Vector3d y(0, 1, 0);
    auto two = chain_of({rev(y, {0, 0, 0}), rev(y, {1, 0, 0})}, {2, 0, 0});
    auto three = chain_of({rev(y, {0, 0, 0}), rev(y, {1, 0, 0}), rev(y, {2, 0, 0})}, {3, 0, 0});

    absent("2R factory: joint count", planar_2r_solver<dyn_chain>::make(three));
    absent("2R factory: non-revolute axis", planar_2r_solver<dyn_chain>::make(
        chain_of({rev(y, {0, 0, 0}), screw_axis<double>::prismatic({0, 0, 1})}, {1, 0, 0})));
    absent("2R factory: zero-length link", planar_2r_solver<dyn_chain>::make(
        chain_of({rev(y, {0, 0, 0}), rev(y, {0, 0, 0})}, {1, 0, 0})));
    // A zero acceptance length rejects every candidate, evaluating no inequality.
    absent("2R solve: verification failed", planar_2r_solver<dyn_chain>::make(
        two, verification_tolerance<double>(0.0, 0.0))->solve(at(1, 0, 0)));

    deficit("2R solve: beyond the outer reach",
        planar_2r_solver<dyn_chain>::make(two)->solve(at(5, 0, 0)));
    deficit("2R solve: inside the reach hole", planar_2r_solver<dyn_chain>::make(
        chain_of({rev(y, {0, 0, 0}), rev(y, {3, 0, 0})}, {4, 0, 0}))->solve(at(0, 0, 0)));
}

TEST_CASE("analytical diagnostics: a 3R failure that evaluated no inequality carries nothing")
{
    const Eigen::Vector3d z(0, 0, 1), y(0, 1, 0);
    const axis_list zyy = {rev(z, {0, 0, 0}), rev(y, {0, 0, 0}), rev(y, {0.4, 0, 0})};
    const axis_list zyz = {rev(z, {0, 0, 0}), rev(y, {0, 0, 0}), rev(z, {0, 0, 0})};

    // Tool point on the third axis leaves the achieved distance independent of
    // the angle, so no angle is determined and none is left to reject.
    absent("3R factory: home end-effector on the third axis",
        spatial_3r_solver<dyn_chain>::make(chain_of(zyy, {0.4, 0, 0})));
    // A zero acceptance length rejects every candidate, evaluating no inequality.
    absent("3R solve: verification failed", spatial_3r_solver<dyn_chain>::make(
        chain_of(zyy, {0.7, 0, 0}), verification_tolerance<double>(0.0, 0.0))
            ->solve(at(0.7, 0, 0)));
    // On a nonfinite input the length the caller would attach is itself a NaN.
    absent("3R solve: subproblem reports a nonfinite input",
        spatial_3r_solver<dyn_chain>::make(chain_of(zyy, {0.7, 0, 0}))->solve(at(nan_v, 0, 0)));
    // A third axis through the shoulder point leaves it singular the same way.
    absent("3R solve: subproblem reports a singular configuration",
        spatial_3r_solver<dyn_chain>::make(chain_of(zyz, {0.5, 0, 0}))->solve(at(0, 0.5, 0)));

    // A forwarded subproblem reason carries nothing, unreachable included.
    CHECK_FALSE(subproblem_error<double>(analytical_failure::unreachable)
        .workspace_distance.has_value());
}

TEST_CASE("analytical diagnostics: a 6R failure that evaluated no inequality carries nothing")
{
    auto puma = chain_of(puma_axes(), {0.8, 0, 0.5});
    auto two = chain_of({rev({0, 1, 0}, {0, 0, 0}), rev({0, 1, 0}, {1, 0, 0})}, {2, 0, 0});
    auto offset_shoulder = fixtures::make_offset_shoulder_puma<double>();
    auto near_spherical = *fixtures::make_near_spherical_wrist_puma<double>(5e-4);

    absent("6R factory: joint count", pieper_6r_solver<dyn_chain>::make(two));
    absent("6R factory: non-revolute axis", pieper_6r_solver<dyn_chain>::make(
        chain_of(with_prismatic_wrist(puma_axes()), {0.8, 0, 0.5})));
    absent("6R factory: shoulder axes do not intersect",
        pieper_6r_solver<decltype(offset_shoulder)>::make(offset_shoulder));
    absent("6R factory: wrist is not spherical",
        pieper_6r_solver<decltype(near_spherical)>::make(near_spherical));
    // Wrist center on the axis-1 line leaves joint 1 undetermined.
    absent("6R solve: shoulder singularity",
        pieper_6r_solver<dyn_chain>::make(puma)->solve(at(0.1, 0.0, 0.6)));
    absent("6R solve: subproblem reports a nonfinite input",
        pieper_6r_solver<dyn_chain>::make(puma)->solve(at(nan_v, 0, 0)));

    absent("6R solve: subproblem reports an unreachable target",
        pieper_6r_solver<dyn_chain>::make(puma)->solve(at(50, 50, 50)));
}

TEST_CASE("analytical diagnostics: an OPW failure that evaluated no inequality carries nothing")
{
    const auto params = fixtures::kr6_r900_opw_parameters<double>();
    const Eigen::Vector3d ee(0, 0, 1.3);
    auto kr6 = fixtures::make_kr6_r900_opw_chain<double>();

    auto not_ortho = opw_axes();
    not_ortho[1] = rev({0, 0, 1}, {0, 0, 0.4});
    auto not_parallel = opw_axes();
    not_parallel[2] = rev({1, 0, 0}, {0, 0, 0.8});
    auto not_spherical = opw_axes();
    not_spherical[4] = rev({0, 1, 0}, {0.3, 0, 1.2});

    // The base geometry passes every gate, so each substitution above isolates one.
    REQUIRE(opw_6r_solver<dyn_chain>::make(chain_of(opw_axes(), ee), params).has_value());

    absent("OPW factory: joint count", opw_6r_solver<dyn_chain>::make(
        chain_of({rev({0, 1, 0}, {0, 0, 0}), rev({0, 1, 0}, {1, 0, 0})}, {2, 0, 0}), params));
    absent("OPW factory: non-revolute axis", opw_6r_solver<dyn_chain>::make(
        chain_of(with_prismatic_wrist(opw_axes()), ee), params));
    absent("OPW factory: axes 1 and 2 not perpendicular",
        opw_6r_solver<dyn_chain>::make(chain_of(not_ortho, ee), params));
    absent("OPW factory: axes 2 and 3 not parallel",
        opw_6r_solver<dyn_chain>::make(chain_of(not_parallel, ee), params));
    absent("OPW factory: wrist is not spherical",
        opw_6r_solver<dyn_chain>::make(chain_of(not_spherical, ee), params));

    // The gates judge the chain's axes and never cross-check the lengths, so a
    // mismatched pair leaves every branch in range yet none reproducing the target.
    auto mismatched = params;
    mismatched.c2 += 0.05;
    mismatched.c3 -= 0.03;
    absent("OPW solve: every branch degenerates",
        opw_6r_solver<decltype(kr6)>::make(kr6, mismatched)->solve(at(0.5, 0.0, 0.5)));

    deficit("OPW solve: no branch verified",
        opw_6r_solver<decltype(kr6)>::make(kr6, params)->solve(at(100, 100, 100)));
}

TEST_CASE("analytical diagnostics: an empty solution set carries nothing")
{
    const auto seed = Eigen::Vector<double, 6>::Zero().eval();

    analytical_result<double, 6, 8> empty;
    empty.count = 0;
    absent("branch collapse: empty solution set", closest_to_seed(empty, seed));

    unwrapped_result<double, 6, 8> empty_unwrapped;
    empty_unwrapped.count = 0;
    absent("branch collapse: empty unwrapped set", closest_to_seed(empty_unwrapped, seed));
}
