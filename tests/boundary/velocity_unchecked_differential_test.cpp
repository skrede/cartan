#include "boundary_fixtures.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <limits>
#include <algorithm>

namespace spp = cartan;

using spp::fixtures::joint_vector;
using spp::fixtures::make_six_joint_dynamic_chain;

/// Both twists come out of the same instruction sequence, so the difference is
/// zero; the tolerance keeps the comparison from reading as an exact-equality
/// claim about floating point.
template <typename Scalar>
static bool same_twist(const spp::vector6<Scalar>& lhs, const spp::vector6<Scalar>& rhs)
{
    const Scalar scale = std::max(lhs.norm(), Scalar(1));
    return (lhs - rhs).norm() <= Scalar(8) * std::numeric_limits<Scalar>::epsilon() * scale;
}

/// The space Jacobian's column i is built from the intermediate product T_{i-1},
/// so the final intermediate is never read and a nonfinite value in the last
/// joint position cannot reach the twist. Every earlier position can.
template <typename Scalar>
static void expect_position_poison_reaches_the_twist(Scalar poison)
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();
    const Eigen::VectorX<Scalar> dq = joint_vector(n, Scalar(0.2));
    for (int i = 0; i < n - 1; ++i)
    {
        Eigen::VectorX<Scalar> q = joint_vector(n, Scalar(0.1));
        q(i) = poison;
        REQUIRE_FALSE(spp::end_effector_velocity_unchecked(chain, q, dq).allFinite());
    }
}

template <typename Scalar>
static void expect_last_position_poison_leaves_no_trace(Scalar poison)
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();
    const Eigen::VectorX<Scalar> dq = joint_vector(n, Scalar(0.2));

    Eigen::VectorX<Scalar> poisoned = joint_vector(n, Scalar(0.1));
    Eigen::VectorX<Scalar> altered = joint_vector(n, Scalar(0.1));
    poisoned(n - 1) = poison;
    altered(n - 1) = Scalar(0.7);

    spp::vector6<Scalar> answered =
        spp::end_effector_velocity_unchecked(chain, poisoned, dq);
    REQUIRE(answered.allFinite());
    REQUIRE(same_twist(answered, spp::end_effector_velocity_unchecked(chain, altered, dq)));
}

template <typename Scalar>
static void expect_velocity_poison_reaches_the_twist(Scalar poison)
{
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();
    const Eigen::VectorX<Scalar> q = joint_vector(n, Scalar(0.1));
    for (int i = 0; i < n; ++i)
    {
        Eigen::VectorX<Scalar> dq = joint_vector(n, Scalar(0.2));
        dq(i) = poison;
        REQUIRE_FALSE(spp::end_effector_velocity_unchecked(chain, q, dq).allFinite());
    }
}

TEMPLATE_TEST_CASE("the unchecked path answers a nonfinite joint vector",
    "[velocity][boundary]", double, float)
{
    using Scalar = TestType;
    for (Scalar poison : {std::numeric_limits<Scalar>::quiet_NaN(),
             std::numeric_limits<Scalar>::infinity(),
             -std::numeric_limits<Scalar>::infinity()})
    {
        expect_position_poison_reaches_the_twist(poison);
        expect_last_position_poison_leaves_no_trace(poison);
        expect_velocity_poison_reaches_the_twist(poison);
    }
}

/// An over-long joint-position vector reads inside its own allocation, so the
/// accumulation loop simply stops at the joint count and a credible twist comes
/// back with no diagnostic anywhere.
TEMPLATE_TEST_CASE("the unchecked path truncates an over-long joint-position vector",
    "[velocity][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();
    const Eigen::VectorX<Scalar> dq = joint_vector(n, Scalar(0.2));

    spp::vector6<Scalar> reference =
        spp::end_effector_velocity_unchecked(chain, joint_vector(n, Scalar(0.1)), dq);
    spp::vector6<Scalar> from_other_values =
        spp::end_effector_velocity_unchecked(chain, joint_vector(n + 1, Scalar(0.6)), dq);
    spp::vector6<Scalar> with_dropped_tail =
        spp::end_effector_velocity_unchecked(chain, joint_vector(n + 1, Scalar(0.1)), dq);

    REQUIRE((from_other_values - reference).norm() > Scalar(1e-3));
    REQUIRE(same_twist(with_dropped_tail, reference));

    auto refused = spp::end_effector_velocity(chain, joint_vector(n + 1, Scalar(0.6)), dq);
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == spp::chain_failure::dimension_mismatch);
}

/// In double the Jacobian product reads no further than its own columns, so the
/// extra entry is dropped. In float the vectorized evaluator reads past the last
/// column, which is a memory error and not a value this test could observe, so
/// only double is exercised here.
TEST_CASE("the unchecked path drops an over-long joint-velocity entry in double",
    "[velocity][boundary]")
{
    auto chain = make_six_joint_dynamic_chain<double>();
    const int n = chain.num_joints();
    const Eigen::VectorXd q = joint_vector(n, 0.1);

    spp::vector6<double> reference =
        spp::end_effector_velocity_unchecked(chain, q, joint_vector(n, 0.2));
    spp::vector6<double> from_other_values =
        spp::end_effector_velocity_unchecked(chain, q, joint_vector(n + 1, 0.5));
    spp::vector6<double> with_dropped_tail =
        spp::end_effector_velocity_unchecked(chain, q, joint_vector(n + 1, 0.2));

    REQUIRE((from_other_values - reference).norm() > 1e-3);
    REQUIRE(same_twist(with_dropped_tail, reference));

    auto refused = spp::end_effector_velocity(chain, q, joint_vector(n + 1, 0.5));
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == spp::chain_failure::dimension_mismatch);
}
