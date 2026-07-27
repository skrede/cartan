#include "boundary_fixtures.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <limits>

namespace spp = cartan;

using spp::fixtures::dynamic_chain_adapter;
using spp::fixtures::fixed_joint_vector;
using spp::fixtures::joint_vector;
using spp::fixtures::make_six_joint_dynamic_chain;
using spp::fixtures::make_six_joint_fixed_chain;
using spp::fixtures::make_six_joint_static_chain;

/// Every length but the chain's, and the chain's length accepted, for an entry
/// point whose joint vector is sized at runtime.
template <typename Scalar, typename Chain, typename Entry>
static void expect_ill_sized_rejected(const Chain& chain, Entry entry)
{
    const int n = chain.num_joints();
    for (int size : {0, n - 1, n + 1})
    {
        auto result = entry(chain, joint_vector(size, Scalar(0.1)));
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error() == spp::chain_failure::dimension_mismatch);
    }
    REQUIRE(entry(chain, joint_vector(n, Scalar(0.1))).has_value());
}

/// Poison one component at a time over every component, so a guard that
/// inspects only the first or the last entry cannot pass.
template <typename Scalar, typename Chain, typename Vector, typename Entry>
static void expect_nonfinite_rejected(const Chain& chain, const Vector& finite, Entry entry)
{
    for (Scalar poison : {std::numeric_limits<Scalar>::quiet_NaN(),
             std::numeric_limits<Scalar>::infinity(),
             -std::numeric_limits<Scalar>::infinity()})
    {
        for (int i = 0; i < chain.num_joints(); ++i)
        {
            Vector q = finite;
            q(i) = poison;
            auto result = entry(chain, q);
            REQUIRE_FALSE(result.has_value());
            REQUIRE(result.error() == spp::chain_failure::non_finite_input);
        }
    }
    REQUIRE(entry(chain, finite).has_value());
}

static constexpr auto scalar_fk = [](const auto& c, const auto& q)
{
    return spp::forward_kinematics(c, q);
};

static constexpr auto matrix_fk = [](const auto& c, const auto& q)
{
    return spp::forward_kinematics_matrix(c, q);
};

TEMPLATE_TEST_CASE("forward_kinematics rejects an ill-sized joint vector",
    "[fk][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    expect_ill_sized_rejected<Scalar>(chain, scalar_fk);
    expect_ill_sized_rejected<Scalar>(dynamic_chain_adapter<Scalar>(chain), scalar_fk);
    expect_ill_sized_rejected<Scalar>(chain, matrix_fk);
}

TEMPLATE_TEST_CASE("forward_kinematics rejects a nonfinite joint value",
    "[fk][boundary]", double, float)
{
    using Scalar = TestType;
    auto dynamic = make_six_joint_dynamic_chain<Scalar>();
    const auto finite = joint_vector(dynamic.num_joints(), Scalar(0.1));
    expect_nonfinite_rejected<Scalar>(dynamic, finite, scalar_fk);
    expect_nonfinite_rejected<Scalar>(dynamic_chain_adapter<Scalar>(dynamic), finite, scalar_fk);
    expect_nonfinite_rejected<Scalar>(dynamic, finite, matrix_fk);

    auto fixed = make_six_joint_fixed_chain<Scalar>();
    auto tagged = make_six_joint_static_chain<Scalar>();
    const auto fixed_finite = fixed_joint_vector(Scalar(0.1));
    expect_nonfinite_rejected<Scalar>(fixed, fixed_finite, scalar_fk);
    expect_nonfinite_rejected<Scalar>(tagged, fixed_finite, scalar_fk);
    expect_nonfinite_rejected<Scalar>(fixed, fixed_finite, matrix_fk);
    expect_nonfinite_rejected<Scalar>(tagged, fixed_finite, matrix_fk);
}

/// Length is established before finiteness, so a call violating both contracts
/// reports the length failure and the caller has to come back a second time.
TEMPLATE_TEST_CASE("forward_kinematics reports the length failure first",
    "[fk][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    Eigen::VectorX<Scalar> q = joint_vector(chain.num_joints() - 1, Scalar(0.1));
    q(0) = std::numeric_limits<Scalar>::quiet_NaN();

    auto pose = spp::forward_kinematics(chain, q);
    REQUIRE_FALSE(pose.has_value());
    REQUIRE(pose.error() == spp::chain_failure::dimension_mismatch);

    auto matrix_pose = spp::forward_kinematics_matrix(chain, q);
    REQUIRE_FALSE(matrix_pose.has_value());
    REQUIRE(matrix_pose.error() == spp::chain_failure::dimension_mismatch);
}

/// The check is the only thing the wrapper adds: on an input that satisfies the
/// precondition the two entry points agree exactly, which is what keeps a caller
/// on the unchecked path from being on a different code path.
TEMPLATE_TEST_CASE("the checked entry point answers what the unchecked one does",
    "[fk][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const auto q = joint_vector(chain.num_joints(), Scalar(0.3));

    auto checked = spp::forward_kinematics(chain, q);
    REQUIRE(checked.has_value());
    const auto unchecked = spp::forward_kinematics_unchecked(chain, q);
    REQUIRE((checked->end_effector.matrix() - unchecked.end_effector.matrix()).norm()
            == Scalar(0));
    REQUIRE(checked->num_joints() == unchecked.num_joints());

    auto checked_matrix = spp::forward_kinematics_matrix(chain, q);
    REQUIRE(checked_matrix.has_value());
    const auto unchecked_matrix = spp::forward_kinematics_matrix_unchecked(chain, q);
    REQUIRE((checked_matrix->end_effector.R - unchecked_matrix.end_effector.R).norm()
            == Scalar(0));
    REQUIRE((checked_matrix->end_effector.p - unchecked_matrix.end_effector.p).norm()
            == Scalar(0));
}
