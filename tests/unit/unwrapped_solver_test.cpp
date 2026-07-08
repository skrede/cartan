#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include "../fixtures/opw_chains.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <numbers>

using namespace cartan;

namespace
{

constexpr double pi = std::numbers::pi;
constexpr double two_pi = 2.0 * pi;
constexpr double tolerance = 1e-9;

// Worst of position and orientation reconstruction error of q against target.
template <typename Chain>
double fk_error(
    const Chain& chain, const Eigen::Vector<double, 6>& q, const se3<double>& target)
{
    auto fk = forward_kinematics(chain, q);
    const double pe = (fk.end_effector.translation() - target.translation()).norm();
    const double oe =
        (fk.end_effector.rotation().inverse() * target.rotation()).log().norm();
    return std::max(pe, oe);
}

double fk_position_error(
    const auto& chain, const Eigen::Vector<double, 3>& q, const se3<double>& target)
{
    auto fk = forward_kinematics(chain, q);
    return (fk.end_effector.translation() - target.translation()).norm();
}

// ZYZ 3R chain (axes 1 and 2 intersect at the origin) with caller-chosen limits,
// so a single geometry can realize each of the four unwrap spans.
auto make_3r_zyz(const std::array<joint_limits<double>, 3>& limits)
{
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {0.5, 0, 0});
    auto home = se3<double>(so3<double>::identity(), Eigen::Vector3d(0.8, 0, 0));
    return static_chain<double, revolute_z, revolute_y, revolute_z>(
        home, {s0, s1, s2}, limits);
}

std::array<joint_limits<double>, 3> uniform_limits(double lo, double hi)
{
    joint_limits<double> lim{lo, hi};
    return {lim, lim, lim};
}

}

TEST_CASE("unwrapped_solver: symmetric range yields the single in-range representative")
{
    auto chain = make_3r_zyz(uniform_limits(-pi, pi));
    auto inner = spatial_3r_solver(chain);
    unwrapped_solver wrapper(inner);

    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;
    auto target = forward_kinematics(chain, q_known).end_effector;

    auto result = wrapper.solve(target);
    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);

    bool saw_in_range = false;
    for (int i = 0; i < result->count; ++i)
    {
        if (result->tags[static_cast<std::size_t>(i)] != range_status::in_range)
            continue;
        saw_in_range = true;
        const auto& q = result->solutions[static_cast<std::size_t>(i)];
        for (int k = 0; k < 3; ++k)
            CHECK((q(k) >= -pi - tolerance && q(k) <= pi + tolerance));
        CHECK(fk_position_error(chain, q, target) < tolerance);
    }
    CHECK(saw_in_range);
}

TEST_CASE("unwrapped_solver: asymmetric range keeps in-range branches within limits")
{
    auto chain = make_3r_zyz(uniform_limits(-0.5, 2.5));
    auto inner = spatial_3r_solver(chain);
    unwrapped_solver wrapper(inner);

    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, 0.4;
    auto target = forward_kinematics(chain, q_known).end_effector;

    auto result = wrapper.solve(target);
    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);

    bool saw_in_range = false;
    for (int i = 0; i < result->count; ++i)
    {
        if (result->tags[static_cast<std::size_t>(i)] != range_status::in_range)
            continue;
        saw_in_range = true;
        const auto& q = result->solutions[static_cast<std::size_t>(i)];
        for (int k = 0; k < 3; ++k)
            CHECK((q(k) >= -0.5 - tolerance && q(k) <= 2.5 + tolerance));
        CHECK(fk_position_error(chain, q, target) < tolerance);
    }
    CHECK(saw_in_range);
}

TEST_CASE("unwrapped_solver: multi-turn range honors the per-call reference")
{
    auto chain = make_3r_zyz(uniform_limits(-3.0 * pi, 3.0 * pi));
    auto inner = spatial_3r_solver(chain);
    unwrapped_solver wrapper(inner);

    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;
    auto target = forward_kinematics(chain, q_known).end_effector;

    auto toward_zero = wrapper.solve(target);
    Eigen::Vector3d seed;
    seed << 0.3 + two_pi, 0.5, -0.2;
    auto toward_seed = wrapper.solve(target, seed);

    REQUIRE(toward_zero.has_value());
    REQUIRE(toward_seed.has_value());
    REQUIRE(toward_zero->count == toward_seed->count);

    bool saw_full_turn = false;
    for (int i = 0; i < toward_zero->count; ++i)
    {
        auto idx = static_cast<std::size_t>(i);
        const auto& qz = toward_zero->solutions[idx];
        const auto& qs = toward_seed->solutions[idx];
        if (std::abs(qs(0) - qz(0) - two_pi) < 1e-6)
        {
            saw_full_turn = true;
            CHECK(fk_position_error(chain, qz, target) < tolerance);
            CHECK(fk_position_error(chain, qs, target) < tolerance);
        }
    }
    CHECK(saw_full_turn);
}

TEST_CASE("unwrapped_solver: tight range tags out-of-arc branches, never drops them")
{
    auto chain = make_3r_zyz(uniform_limits(0.1, 0.2));
    auto inner = spatial_3r_solver(chain);
    unwrapped_solver wrapper(inner);

    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;
    auto target = forward_kinematics(chain, q_known).end_effector;

    auto raw = inner.solve(target);
    auto result = wrapper.solve(target);
    REQUIRE(raw.has_value());
    REQUIRE(result.has_value());
    REQUIRE(result->count == raw->count);
    REQUIRE(result->count >= 1);

    for (int i = 0; i < result->count; ++i)
        CHECK(result->tags[static_cast<std::size_t>(i)]
            == range_status::joint_limits_violated);
}

TEST_CASE("unwrapped_solver: passes an inner whole-solve failure through unchanged")
{
    auto chain = make_3r_zyz(uniform_limits(-pi, pi));
    auto inner = spatial_3r_solver(chain);
    unwrapped_solver wrapper(inner);

    auto far_target =
        se3<double>(so3<double>::identity(), Eigen::Vector3d(100.0, 100.0, 100.0));

    auto raw = inner.solve(far_target);
    auto result = wrapper.solve(far_target);
    REQUIRE_FALSE(raw.has_value());
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == raw.error().reason);
}

TEST_CASE("unwrapped_solver: composes over the OPW solver with identical wrapper code")
{
    auto chain = fixtures::make_kr6_r900_opw_chain<double>();
    auto params = fixtures::kr6_r900_opw_parameters<double>();
    auto inner = opw_6r_solver<decltype(chain)>::make(chain, params, tolerance);
    REQUIRE(inner.has_value());

    unwrapped_solver wrapper(*inner);

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.4, 0.5, 0.2, -0.3, 0.1;
    auto target = forward_kinematics(chain, q_known).end_effector;

    auto result = wrapper.solve(target);
    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);

    bool saw_in_range = false;
    for (int i = 0; i < result->count; ++i)
    {
        if (result->tags[static_cast<std::size_t>(i)] != range_status::in_range)
            continue;
        saw_in_range = true;
        const auto& q = result->solutions[static_cast<std::size_t>(i)];
        for (int k = 0; k < 6; ++k)
            CHECK((q(k) >= -pi - tolerance && q(k) <= pi + tolerance));
        CHECK(fk_error(chain, q, target) < tolerance);
    }
    CHECK(saw_in_range);

    auto far_target =
        se3<double>(so3<double>::identity(), Eigen::Vector3d(10.0, 10.0, 10.0));
    auto raw_far = inner->solve(far_target);
    auto wrapped_far = wrapper.solve(far_target);
    REQUIRE_FALSE(raw_far.has_value());
    REQUIRE_FALSE(wrapped_far.has_value());
    CHECK(wrapped_far.error().reason == raw_far.error().reason);
}
