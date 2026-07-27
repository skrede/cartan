#include "boundary_fixtures.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

namespace spp = cartan;

using spp::fixtures::joint_vector;
using spp::fixtures::make_six_joint_dynamic_chain;

TEMPLATE_TEST_CASE("end_effector_velocity rejects an ill-sized joint vector",
    "[velocity][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();

    for (int size : {0, 5, 7})
    {
        auto result = spp::end_effector_velocity(
            chain, joint_vector(size, Scalar(0.1)), joint_vector(n, Scalar(0.2)));
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error() == spp::chain_failure::dimension_mismatch);
    }

    for (int size : {0, 5, 7})
    {
        auto result = spp::end_effector_velocity(
            chain, joint_vector(n, Scalar(0.1)), joint_vector(size, Scalar(0.2)));
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error() == spp::chain_failure::dimension_mismatch);
    }

    auto accepted = spp::end_effector_velocity(
        chain, joint_vector(n, Scalar(0.1)), joint_vector(n, Scalar(0.2)));
    REQUIRE(accepted.has_value());
}

/// The Jacobian family takes a cached forward-kinematics result rather than a
/// joint vector, so its structural precondition is a different predicate.
TEMPLATE_TEST_CASE("check_fk_shape rejects a result holding the wrong number of intermediates",
    "[velocity][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto fk = spp::forward_kinematics(
        chain, joint_vector(chain.num_joints(), Scalar(0.1)));

    REQUIRE(spp::detail::check_fk_shape(chain, fk).has_value());

    spp::fk_result<Scalar, spp::dynamic> empty;
    auto mismatched = spp::detail::check_fk_shape(chain, empty);
    REQUIRE_FALSE(mismatched.has_value());
    REQUIRE(mismatched.error() == spp::chain_failure::dimension_mismatch);
}
