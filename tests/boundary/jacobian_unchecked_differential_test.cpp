#include "boundary_fixtures.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

namespace spp = cartan;

using spp::fixtures::joint_vector;
using spp::fixtures::make_dynamic_chain;
using spp::fixtures::make_six_joint_dynamic_chain;
using spp::fixtures::six_joints;

template <typename Scalar>
static spp::fk_result<Scalar, spp::dynamic> result_at(int joints, Scalar value)
{
    auto chain = make_dynamic_chain<Scalar>(joints);
    return spp::forward_kinematics(chain, joint_vector(joints, value)).value();
}

template <typename Scalar>
static spp::fk_matrix_result<Scalar, spp::dynamic> matrix_result_at(int joints, Scalar value)
{
    auto chain = make_dynamic_chain<Scalar>(joints);
    return spp::forward_kinematics_matrix(chain, joint_vector(joints, value)).value();
}

/// A result holding more intermediates than the chain has joints is read
/// entirely in bounds, so the unchecked path answers with a full, credible
/// matrix computed against a cache belonging to another configuration.
///
/// The extra joints alone are not enough to make the answer wrong: the fixture
/// chains share their leading axes, so a longer result taken at the same joint
/// values reproduces the correct matrix exactly. What the guard is protecting
/// is the pairing of a result with the chain that produced it, and the value
/// difference only shows when the foreign result encodes a different
/// configuration.
TEMPLATE_TEST_CASE("the unchecked Jacobians answer an over-long result",
    "[jacobian][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const auto own = result_at<Scalar>(six_joints, Scalar(0.1));
    const auto foreign = result_at<Scalar>(3 * six_joints, Scalar(0.7));

    REQUIRE((spp::space_jacobian_unchecked(chain, foreign)
             - spp::space_jacobian_unchecked(chain, own)).norm() > Scalar(1e-3));
    REQUIRE((spp::body_jacobian_unchecked(chain, foreign)
             - spp::body_jacobian_unchecked(chain, own)).norm() > Scalar(1e-3));

    auto refused_space = spp::space_jacobian(chain, foreign);
    REQUIRE_FALSE(refused_space.has_value());
    REQUIRE(refused_space.error() == spp::chain_failure::dimension_mismatch);
    auto refused_body = spp::body_jacobian(chain, foreign);
    REQUIRE_FALSE(refused_body.has_value());
    REQUIRE(refused_body.error() == spp::chain_failure::dimension_mismatch);
}

TEMPLATE_TEST_CASE("the unchecked matrix-form Jacobian answers an over-long result",
    "[jacobian][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const auto own = matrix_result_at<Scalar>(six_joints, Scalar(0.1));
    const auto foreign = matrix_result_at<Scalar>(3 * six_joints, Scalar(0.7));

    REQUIRE((spp::space_jacobian_unchecked(chain, foreign)
             - spp::space_jacobian_unchecked(chain, own)).norm() > Scalar(1e-3));

    auto refused = spp::space_jacobian(chain, foreign);
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == spp::chain_failure::dimension_mismatch);
}
