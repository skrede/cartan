#include "six_joint_chain.h"

#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <string_view>

namespace spp = cartan;

using spp::testing::filled;
using spp::testing::make_six_joint_dynamic_chain;

/// The live demonstration behind every nonfinite cell: the unchecked path
/// answers where the checked path refuses, and the answer is worthless either
/// way. Either the poison propagates into the twist, or the twist is finite and
/// entirely plausible while being wholly independent of the poisoned value --
/// which is the case for the last joint position, whose intermediate product
/// the space Jacobian never reads, and is the worse of the two.
template <typename Scalar>
static void expect_wrong_success(
    const spp::vector6<Scalar>& answered, const spp::vector6<Scalar>& altered)
{
    const bool propagated = !answered.allFinite();
    const bool ignored = (answered - altered).norm() == Scalar(0);
    REQUIRE((propagated || ignored));
}

/// Poison one joint-position component at a time, over every component, so a
/// guard that inspects only the first or last entry cannot pass.
template <typename Scalar>
static void expect_positions_rejected(Scalar poison)
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();
    const Eigen::VectorX<Scalar> dq = filled(n, Scalar(0.2));
    for (int i = 0; i < n; ++i)
    {
        Eigen::VectorX<Scalar> q = filled(n, Scalar(0.1));
        Eigen::VectorX<Scalar> altered = filled(n, Scalar(0.1));
        q(i) = poison;
        altered(i) = Scalar(0.7);

        auto result = spp::end_effector_velocity(chain, q, dq);
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error() == spp::chain_failure::non_finite_input);

        expect_wrong_success<Scalar>(
            spp::end_effector_velocity_unchecked(chain, q, dq),
            spp::end_effector_velocity_unchecked(chain, altered, dq));
    }
}

template <typename Scalar>
static void expect_velocities_rejected(Scalar poison)
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();
    const Eigen::VectorX<Scalar> q = filled(n, Scalar(0.1));
    for (int i = 0; i < n; ++i)
    {
        Eigen::VectorX<Scalar> dq = filled(n, Scalar(0.2));
        Eigen::VectorX<Scalar> altered = filled(n, Scalar(0.2));
        dq(i) = poison;
        altered(i) = Scalar(0.7);

        auto result = spp::end_effector_velocity(chain, q, dq);
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error() == spp::chain_failure::non_finite_input);

        expect_wrong_success<Scalar>(
            spp::end_effector_velocity_unchecked(chain, q, dq),
            spp::end_effector_velocity_unchecked(chain, q, altered));
    }
}

TEST_CASE("end_effector_velocity rejects nonfinite joint values in double",
    "[velocity][boundary]")
{
    expect_positions_rejected<double>(std::numeric_limits<double>::quiet_NaN());
    expect_positions_rejected<double>(-std::numeric_limits<double>::quiet_NaN());
    expect_positions_rejected<double>(std::numeric_limits<double>::infinity());
    expect_positions_rejected<double>(-std::numeric_limits<double>::infinity());
    expect_velocities_rejected<double>(std::numeric_limits<double>::quiet_NaN());
    expect_velocities_rejected<double>(-std::numeric_limits<double>::quiet_NaN());
    expect_velocities_rejected<double>(std::numeric_limits<double>::infinity());
    expect_velocities_rejected<double>(-std::numeric_limits<double>::infinity());
}

TEST_CASE("end_effector_velocity rejects nonfinite joint values in float",
    "[velocity][boundary]")
{
    expect_positions_rejected<float>(std::numeric_limits<float>::quiet_NaN());
    expect_positions_rejected<float>(-std::numeric_limits<float>::quiet_NaN());
    expect_positions_rejected<float>(std::numeric_limits<float>::infinity());
    expect_positions_rejected<float>(-std::numeric_limits<float>::infinity());
    expect_velocities_rejected<float>(std::numeric_limits<float>::quiet_NaN());
    expect_velocities_rejected<float>(-std::numeric_limits<float>::quiet_NaN());
    expect_velocities_rejected<float>(std::numeric_limits<float>::infinity());
    expect_velocities_rejected<float>(-std::numeric_limits<float>::infinity());
}

TEST_CASE("chain_failure carries a diagnostic for both boundary failures",
    "[velocity][boundary]")
{
    REQUIRE_FALSE(std::string_view(
        spp::message(spp::chain_failure::dimension_mismatch)).empty());
    REQUIRE_FALSE(std::string_view(
        spp::message(spp::chain_failure::non_finite_input)).empty());
}
