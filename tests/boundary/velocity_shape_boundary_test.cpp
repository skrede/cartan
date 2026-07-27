#include "six_joint_chain.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

namespace spp = cartan;

using spp::testing::filled;
using spp::testing::make_six_joint_dynamic_chain;

TEMPLATE_TEST_CASE("end_effector_velocity rejects an ill-sized joint vector",
    "[velocity][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();

    for (int size : {0, 5, 7})
    {
        auto result = spp::end_effector_velocity(
            chain, filled(size, Scalar(0.1)), filled(n, Scalar(0.2)));
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error() == spp::chain_failure::dimension_mismatch);
    }

    for (int size : {0, 5, 7})
    {
        auto result = spp::end_effector_velocity(
            chain, filled(n, Scalar(0.1)), filled(size, Scalar(0.2)));
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error() == spp::chain_failure::dimension_mismatch);
    }

    auto accepted = spp::end_effector_velocity(
        chain, filled(n, Scalar(0.1)), filled(n, Scalar(0.2)));
    REQUIRE(accepted.has_value());
}

/// An over-long joint-position vector trips no sanitizer: the accumulation loop
/// is bounded by the chain's joint count, so the extra entry is dropped and a
/// credible twist comes back. The pair below is the standing demonstration that
/// the truncation is real and that the check is what prevents it.
TEMPLATE_TEST_CASE("the unchecked path truncates an over-long joint-position vector that the checked path refuses",
    "[velocity][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();
    const Eigen::VectorX<Scalar> dq = filled(n, Scalar(0.2));

    spp::vector6<Scalar> reference =
        spp::end_effector_velocity_unchecked(chain, filled(n, Scalar(0.1)), dq);
    spp::vector6<Scalar> truncated =
        spp::end_effector_velocity_unchecked(chain, filled(n + 1, Scalar(0.6)), dq);
    spp::vector6<Scalar> dropped_tail =
        spp::end_effector_velocity_unchecked(chain, filled(n + 1, Scalar(0.1)), dq);

    REQUIRE((truncated - reference).norm() > Scalar(1e-3));
    REQUIRE((dropped_tail - reference).norm() == Scalar(0));

    auto refused = spp::end_effector_velocity(chain, filled(n + 1, Scalar(0.6)), dq);
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == spp::chain_failure::dimension_mismatch);
}

/// An over-long joint-velocity vector splits by scalar, measured: in double the
/// Jacobian product reads no further than its own columns, so the extra entry is
/// dropped and a credible twist comes back, while in float the vectorized
/// evaluator reads past the last column and the address sanitizer fires. Only
/// the double half can be asserted on a returned value; the float half is a
/// memory-error cell and is carried by a sanitizer tripwire instead.
TEST_CASE("the unchecked path drops an over-long joint-velocity entry in double",
    "[velocity][boundary]")
{
    auto chain = make_six_joint_dynamic_chain<double>();
    const int n = chain.num_joints();
    const Eigen::VectorXd q = filled(n, 0.1);

    spp::vector6<double> reference =
        spp::end_effector_velocity_unchecked(chain, q, filled(n, 0.2));
    spp::vector6<double> truncated =
        spp::end_effector_velocity_unchecked(chain, q, filled(n + 1, 0.5));
    spp::vector6<double> dropped_tail =
        spp::end_effector_velocity_unchecked(chain, q, filled(n + 1, 0.2));

    REQUIRE((truncated - reference).norm() > 1e-3);
    REQUIRE((dropped_tail - reference).norm() == 0.0);

    auto refused = spp::end_effector_velocity(chain, q, filled(n + 1, 0.5));
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == spp::chain_failure::dimension_mismatch);
}
