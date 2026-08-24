#include "boundary_fixtures.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

namespace spp = cartan;

using spp::fixtures::dynamic_chain_adapter;
using spp::fixtures::fixed_joint_vector;
using spp::fixtures::joint_vector;
using spp::fixtures::make_dynamic_chain;
using spp::fixtures::make_six_joint_dynamic_chain;
using spp::fixtures::make_six_joint_fixed_chain;
using spp::fixtures::make_six_joint_static_chain;
using spp::fixtures::six_joints;

/// A cached result carrying a joint count other than the chain's, produced the
/// only way a caller can produce one: from a different chain.
template <typename Scalar>
static spp::fk_result<Scalar, spp::dynamic> result_of(int joints)
{
    auto chain = make_dynamic_chain<Scalar>(joints);
    return spp::forward_kinematics(chain, joint_vector(joints, Scalar(0.2))).value();
}

template <typename Scalar>
static spp::fk_matrix_result<Scalar, spp::dynamic> matrix_result_of(int joints)
{
    auto chain = make_dynamic_chain<Scalar>(joints);
    return spp::forward_kinematics_matrix(chain, joint_vector(joints, Scalar(0.2))).value();
}

template <typename Scalar, typename Chain, typename Result, typename Entry>
static void expect_mismatched_result_rejected(
    const Chain& chain, const Result& matching, Entry entry)
{
    for (int joints : {1, six_joints - 1, six_joints + 1, 3 * six_joints})
    {
        auto rejected = entry(chain, result_of<Scalar>(joints));
        REQUIRE_FALSE(rejected.has_value());
        REQUIRE(rejected.error() == spp::chain_failure::dimension_mismatch);
    }
    REQUIRE(entry(chain, matching).has_value());
}

static constexpr auto space = [](const auto& c, const auto& fk)
{
    return spp::space_jacobian(c, fk);
};

static constexpr auto body = [](const auto& c, const auto& fk)
{
    return spp::body_jacobian(c, fk);
};

TEMPLATE_TEST_CASE("the Jacobians reject a result from a chain of another size",
    "[jacobian][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const auto matching = result_of<Scalar>(six_joints);
    expect_mismatched_result_rejected<Scalar>(chain, matching, space);
    expect_mismatched_result_rejected<Scalar>(chain, matching, body);

    const dynamic_chain_adapter<Scalar> adapter(chain);
    expect_mismatched_result_rejected<Scalar>(adapter, matching, space);
    expect_mismatched_result_rejected<Scalar>(adapter, matching, body);
}

TEMPLATE_TEST_CASE("the matrix-form Jacobian rejects a result from a chain of another size",
    "[jacobian][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const auto matching = matrix_result_of<Scalar>(six_joints);

    for (int joints : {1, six_joints - 1, six_joints + 1, 3 * six_joints})
    {
        auto rejected = spp::space_jacobian(chain, matrix_result_of<Scalar>(joints));
        REQUIRE_FALSE(rejected.has_value());
        REQUIRE(rejected.error() == spp::chain_failure::dimension_mismatch);
    }
    REQUIRE(spp::space_jacobian(chain, matching).has_value());
}

/// A result for a chain whose joint count is in its type carries that count in
/// its own type too, so a mismatch is unrepresentable at these three entry
/// points and only the accepting half is testable. The predicate call stays
/// anyway, because the joint count is not always in the type.
TEMPLATE_TEST_CASE("the fixed-size Jacobians accept the only result they can be handed",
    "[jacobian][boundary]", double, float)
{
    using Scalar = TestType;
    auto fixed = make_six_joint_fixed_chain<Scalar>();
    auto tagged = make_six_joint_static_chain<Scalar>();
    const auto q = fixed_joint_vector(Scalar(0.2));

    const auto fixed_fk = spp::forward_kinematics(fixed, q).value();
    REQUIRE(spp::space_jacobian(fixed, fixed_fk).has_value());
    REQUIRE(spp::body_jacobian(fixed, fixed_fk).has_value());

    const auto tagged_fk = spp::forward_kinematics(tagged, q).value();
    REQUIRE(spp::space_jacobian(tagged, tagged_fk).has_value());
    REQUIRE(spp::body_jacobian(tagged, tagged_fk).has_value());

    const auto fixed_matrix_fk = spp::forward_kinematics_matrix(fixed, q).value();
    REQUIRE(spp::space_jacobian(fixed, fixed_matrix_fk).has_value());
    const auto tagged_matrix_fk = spp::forward_kinematics_matrix(tagged, q).value();
    REQUIRE(spp::space_jacobian(tagged, tagged_matrix_fk).has_value());
}

/// A result of the chain's own length taken from a different chain of that
/// length passes the predicate: this guard is shape, not provenance.
TEMPLATE_TEST_CASE("a same-length result from another configuration is still admitted",
    "[jacobian][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto other = make_dynamic_chain<Scalar>(six_joints);
    const auto elsewhere =
        spp::forward_kinematics(other, joint_vector(six_joints, Scalar(0.7))).value();

    auto admitted = spp::space_jacobian(chain, elsewhere);
    REQUIRE(admitted.has_value());
    REQUIRE((admitted.value() - spp::space_jacobian(chain, result_of<Scalar>(six_joints))
                                    .value())
                .norm()
            > Scalar(1e-3));
}

/// The check is the only thing the wrappers add: on a result that satisfies the
/// precondition, checked and unchecked agree exactly at every entry point.
TEMPLATE_TEST_CASE("the checked Jacobians answer what the unchecked ones do",
    "[jacobian][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const dynamic_chain_adapter<Scalar> adapter(chain);
    const auto fk = result_of<Scalar>(six_joints);

    REQUIRE((spp::space_jacobian(chain, fk).value()
             - spp::space_jacobian_unchecked(chain, fk)).norm() == Scalar(0));
    REQUIRE((spp::body_jacobian(chain, fk).value()
             - spp::body_jacobian_unchecked(chain, fk)).norm() == Scalar(0));
    REQUIRE((spp::space_jacobian(adapter, fk).value()
             - spp::space_jacobian_unchecked(adapter, fk)).norm() == Scalar(0));
    REQUIRE((spp::body_jacobian(adapter, fk).value()
             - spp::body_jacobian_unchecked(adapter, fk)).norm() == Scalar(0));

    const auto matrix_fk = matrix_result_of<Scalar>(six_joints);
    REQUIRE((spp::space_jacobian(chain, matrix_fk).value()
             - spp::space_jacobian_unchecked(chain, matrix_fk)).norm() == Scalar(0));

    auto tagged = make_six_joint_static_chain<Scalar>();
    const auto tagged_fk =
        spp::forward_kinematics(tagged, fixed_joint_vector(Scalar(0.2))).value();
    REQUIRE((spp::space_jacobian(tagged, tagged_fk).value()
             - spp::space_jacobian_unchecked(tagged, tagged_fk)).norm() == Scalar(0));
    REQUIRE((spp::body_jacobian(tagged, tagged_fk).value()
             - spp::body_jacobian_unchecked(tagged, tagged_fk)).norm() == Scalar(0));
}
