#include "boundary_fixtures.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <limits>
#include <algorithm>

namespace spp = cartan;

using spp::fixtures::fixed_joint_vector;
using spp::fixtures::joint_vector;
using spp::fixtures::make_six_joint_dynamic_chain;
using spp::fixtures::make_six_joint_fixed_chain;
using spp::fixtures::six_joints;

/// Both poses come out of the same instruction sequence, so the difference is
/// zero; the tolerance keeps the comparison from reading as an exact-equality
/// claim about floating point.
template <typename Matrix>
static bool same_pose(const Matrix& lhs, const Matrix& rhs)
{
    using Scalar = typename Matrix::Scalar;
    const Scalar scale = std::max(lhs.norm(), Scalar(1));
    return (lhs - rhs).norm() <= Scalar(8) * std::numeric_limits<Scalar>::epsilon() * scale;
}

/// An over-long joint vector reads inside its own allocation, so the
/// accumulation loop simply stops at the joint count and a credible pose comes
/// back with no diagnostic anywhere.
TEMPLATE_TEST_CASE("the unchecked path truncates an over-long joint vector",
    "[fk][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();

    const auto reference =
        spp::forward_kinematics_unchecked(chain, joint_vector(n, Scalar(0.1)));
    const auto from_other_values =
        spp::forward_kinematics_unchecked(chain, joint_vector(n + 1, Scalar(0.6)));
    const auto with_dropped_tail =
        spp::forward_kinematics_unchecked(chain, joint_vector(n + 1, Scalar(0.1)));

    REQUIRE((from_other_values.end_effector.matrix() - reference.end_effector.matrix()).norm()
            > Scalar(1e-3));
    REQUIRE(same_pose(with_dropped_tail.end_effector.matrix(),
        reference.end_effector.matrix()));

    auto refused = spp::forward_kinematics(chain, joint_vector(n + 1, Scalar(0.6)));
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == spp::chain_failure::dimension_mismatch);
}

TEMPLATE_TEST_CASE("the unchecked matrix path truncates an over-long joint vector",
    "[fk][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();

    const auto reference =
        spp::forward_kinematics_matrix_unchecked(chain, joint_vector(n, Scalar(0.1)));
    const auto from_other_values =
        spp::forward_kinematics_matrix_unchecked(chain, joint_vector(n + 1, Scalar(0.6)));
    const auto with_dropped_tail =
        spp::forward_kinematics_matrix_unchecked(chain, joint_vector(n + 1, Scalar(0.1)));

    REQUIRE((from_other_values.end_effector.p - reference.end_effector.p).norm()
            > Scalar(1e-3));
    REQUIRE(same_pose(with_dropped_tail.end_effector.p, reference.end_effector.p));
    REQUIRE(same_pose(with_dropped_tail.end_effector.R, reference.end_effector.R));

    auto refused = spp::forward_kinematics_matrix(chain, joint_vector(n + 1, Scalar(0.6)));
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == spp::chain_failure::dimension_mismatch);
}

/// Unlike the velocity path, which never reads the last intermediate product,
/// the end-effector pose accumulates every joint, so a poisoned value in any
/// slot reaches the returned pose.
TEMPLATE_TEST_CASE("the unchecked path answers a nonfinite joint vector",
    "[fk][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    const int n = chain.num_joints();

    for (Scalar poison : {std::numeric_limits<Scalar>::quiet_NaN(),
             std::numeric_limits<Scalar>::infinity(),
             -std::numeric_limits<Scalar>::infinity()})
    {
        for (int i = 0; i < n; ++i)
        {
            Eigen::VectorX<Scalar> q = joint_vector(n, Scalar(0.1));
            q(i) = poison;
            REQUIRE_FALSE(spp::forward_kinematics_unchecked(chain, q)
                              .end_effector.matrix()
                              .allFinite());
            REQUIRE_FALSE(spp::forward_kinematics_matrix_unchecked(chain, q)
                              .end_effector.R.allFinite());
        }
    }
}

/// The unchecked sibling still spells its joint vector as the chain's
/// fixed-size position_type, so an over-long dynamically-sized argument is
/// converted in the caller's frame, silently truncated to the joint count, and
/// answered with a credible pose. That is what the checked entry point looked
/// like at every fixed-size chain before the conversion moved inside it.
TEMPLATE_TEST_CASE("the unchecked fixed-size path truncates a dynamically-sized joint vector",
    "[fk][boundary]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_fixed_chain<Scalar>();

    const auto reference =
        spp::forward_kinematics_unchecked(chain, fixed_joint_vector(Scalar(0.1)));
    const auto from_other_values =
        spp::forward_kinematics_unchecked(chain, joint_vector(six_joints + 1, Scalar(0.6)));
    const auto with_dropped_tail =
        spp::forward_kinematics_unchecked(chain, joint_vector(six_joints + 1, Scalar(0.1)));

    REQUIRE((from_other_values.end_effector.matrix() - reference.end_effector.matrix()).norm()
            > Scalar(1e-3));
    REQUIRE(same_pose(with_dropped_tail.end_effector.matrix(),
        reference.end_effector.matrix()));

    auto refused = spp::forward_kinematics(chain, joint_vector(six_joints + 1, Scalar(0.6)));
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == spp::chain_failure::dimension_mismatch);
}
