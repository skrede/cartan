#include "boundary_fixtures.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <limits>
#include <string_view>

namespace spp = cartan;

using spp::fixtures::joint_vector;
using spp::fixtures::make_six_joint_dynamic_chain;

/// Poison one component at a time over every component, so a guard that inspects
/// only the first or the last entry cannot pass.
template <typename Scalar>
static void expect_positions_rejected(Scalar poison)
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();
    for (int i = 0; i < n; ++i)
    {
        Eigen::VectorX<Scalar> q = joint_vector(n, Scalar(0.1));
        q(i) = poison;
        auto result = spp::end_effector_velocity(chain, q, joint_vector(n, Scalar(0.2)));
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error() == spp::chain_failure::non_finite_input);
    }
}

template <typename Scalar>
static void expect_velocities_rejected(Scalar poison)
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();
    for (int i = 0; i < n; ++i)
    {
        Eigen::VectorX<Scalar> dq = joint_vector(n, Scalar(0.2));
        dq(i) = poison;
        auto result = spp::end_effector_velocity(chain, joint_vector(n, Scalar(0.1)), dq);
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error() == spp::chain_failure::non_finite_input);
    }
}

TEST_CASE("end_effector_velocity rejects nonfinite joint values in double",
    "[velocity][boundary]")
{
    expect_positions_rejected<double>(std::numeric_limits<double>::quiet_NaN());
    expect_positions_rejected<double>(std::numeric_limits<double>::infinity());
    expect_positions_rejected<double>(-std::numeric_limits<double>::infinity());
    expect_velocities_rejected<double>(std::numeric_limits<double>::quiet_NaN());
    expect_velocities_rejected<double>(std::numeric_limits<double>::infinity());
    expect_velocities_rejected<double>(-std::numeric_limits<double>::infinity());
}

TEST_CASE("end_effector_velocity rejects nonfinite joint values in float",
    "[velocity][boundary]")
{
    expect_positions_rejected<float>(std::numeric_limits<float>::quiet_NaN());
    expect_positions_rejected<float>(std::numeric_limits<float>::infinity());
    expect_positions_rejected<float>(-std::numeric_limits<float>::infinity());
    expect_velocities_rejected<float>(std::numeric_limits<float>::quiet_NaN());
    expect_velocities_rejected<float>(std::numeric_limits<float>::infinity());
    expect_velocities_rejected<float>(-std::numeric_limits<float>::infinity());
}

/// Joint positions are validated before joint velocities, so a call that
/// violates both contracts reports the position failure and the caller has to
/// come back a second time.
TEMPLATE_TEST_CASE("end_effector_velocity reports the joint-position failure first",
    "[velocity][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();
    Eigen::VectorX<Scalar> dq = joint_vector(n, Scalar(0.2));
    dq(0) = std::numeric_limits<Scalar>::quiet_NaN();

    auto ill_sized = spp::end_effector_velocity(chain, joint_vector(n - 1, Scalar(0.1)), dq);
    REQUIRE_FALSE(ill_sized.has_value());
    REQUIRE(ill_sized.error() == spp::chain_failure::dimension_mismatch);

    Eigen::VectorX<Scalar> q = joint_vector(n, Scalar(0.1));
    q(0) = std::numeric_limits<Scalar>::infinity();
    auto both = spp::end_effector_velocity(chain, q, dq);
    REQUIRE_FALSE(both.has_value());
    REQUIRE(both.error() == spp::chain_failure::non_finite_input);
}

TEST_CASE("chain_failure carries a diagnostic for both boundary failures",
    "[velocity][boundary]")
{
    REQUIRE_FALSE(std::string_view(
        spp::message(spp::chain_failure::dimension_mismatch)).empty());
    REQUIRE_FALSE(std::string_view(
        spp::message(spp::chain_failure::non_finite_input)).empty());
}
