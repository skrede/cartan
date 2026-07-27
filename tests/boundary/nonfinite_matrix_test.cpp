#include "boundary_fixtures.h"

#include <cartan/lie/se2.h>

#include <cartan/serial/ik/detail/limit_enforcement.h>

#include <cartan/analytical/detail/fk_verification.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <array>
#include <limits>
#include <vector>
#include <utility>
#include <optional>

namespace
{

/// Every boundary here is swept with all three classes rather than with a NaN
/// alone. An infinity reaches a tolerance comparison as a NaN once the algebra
/// has multiplied it by a zero, so a boundary can refuse one class and admit
/// the other, and several of these did.
template <typename Scalar>
std::array<Scalar, 3> nonfinite_values()
{
    return {std::numeric_limits<Scalar>::quiet_NaN(),
        std::numeric_limits<Scalar>::infinity(),
        -std::numeric_limits<Scalar>::infinity()};
}

/// Poisons one entry at a time over the whole matrix, so a factory that reads
/// only the block it validates cannot pass. The rotation block, the affine row
/// and the translation block of a transform are all covered by one sweep, and
/// the survey measured those positions behaving differently from each other.
template <typename Matrix, typename Factory>
void expect_every_entry_refused(const Matrix& seed, Factory factory)
{
    using Scalar = typename Matrix::Scalar;
    for (Scalar poison : nonfinite_values<Scalar>())
    {
        for (int row = 0; row < int(seed.rows()); ++row)
        {
            for (int col = 0; col < int(seed.cols()); ++col)
            {
                Matrix poisoned = seed;
                poisoned(row, col) = poison;
                auto result = factory(poisoned);
                REQUIRE_FALSE(result.has_value());
                REQUIRE(result.error() == cartan::lie_failure::non_finite_input);
            }
        }
    }
}

template <typename Scalar>
cartan::kinematic_chain<Scalar, cartan::dynamic> one_joint_chain(Scalar lo, Scalar hi)
{
    std::vector<cartan::screw_axis<Scalar>> axes{
        cartan::screw_axis<Scalar>::revolute({0, 0, 1}, {0, 0, 0})};
    std::vector<cartan::joint_limits<Scalar>> limits{cartan::testing::limits(lo, hi)};
    return cartan::kinematic_chain<Scalar, cartan::dynamic>(
        cartan::se3<Scalar>::identity(), std::move(axes), std::move(limits));
}

template <typename Scalar>
bool feasible(Scalar q, Scalar lo, Scalar hi)
{
    auto chain = one_joint_chain(lo, hi);
    Eigen::VectorX<Scalar> qv(1);
    qv << q;
    return cartan::detail::within_limits(
        qv, chain, cartan::detail::default_feasibility_tol<Scalar>());
}

template <typename Chain, typename Scalar>
void expect_candidates_refused(
    const Chain& chain, const cartan::se3<Scalar>& target, bool check_orientation)
{
    const Eigen::Vector<Scalar, 6> home = Eigen::Vector<Scalar, 6>::Zero();
    REQUIRE(cartan::detail::verify_analytical_solution(chain, home, target, check_orientation));
    for (Scalar poison : nonfinite_values<Scalar>())
    {
        for (int i = 0; i < 6; ++i)
        {
            Eigen::Vector<Scalar, 6> candidate = home;
            candidate(i) = poison;
            REQUIRE_FALSE(cartan::detail::verify_analytical_solution(
                chain, candidate, target, check_orientation));
        }
    }
}

}

TEMPLATE_TEST_CASE("every checked matrix factory refuses a nonfinite entry at every position",
    "[nonfinite][boundary]", double, float)
{
    using S = TestType;
    expect_every_entry_refused(cartan::matrix2<S>::Identity().eval(),
        [](const cartan::matrix2<S>& m) { return cartan::so2<S>::from_matrix(m); });
    expect_every_entry_refused(cartan::matrix3<S>::Identity().eval(),
        [](const cartan::matrix3<S>& m) { return cartan::so3<S>::from_matrix(m); });
    expect_every_entry_refused(cartan::matrix3<S>::Identity().eval(),
        [](const cartan::matrix3<S>& m) { return cartan::se2<S>::from_matrix(m); });
    expect_every_entry_refused(cartan::matrix4<S>::Identity().eval(),
        [](const cartan::matrix4<S>& m) { return cartan::se3<S>::from_matrix(m); });
}

TEMPLATE_TEST_CASE("so3::from_quaternion refuses a nonfinite coefficient",
    "[nonfinite][boundary]", double, float)
{
    using S = TestType;
    for (S poison : nonfinite_values<S>())
    {
        for (int i = 0; i < 4; ++i)
        {
            cartan::quaternion<S> q = cartan::quaternion<S>::Identity();
            q.coeffs()(i) = poison;
            auto result = cartan::so3<S>::from_quaternion(q);
            REQUIRE_FALSE(result.has_value());
            REQUIRE(result.error() == cartan::lie_failure::non_finite_input);
        }
    }
}

/// Both branches, because the pre-guard factory validated only the component
/// its branch selected on: a poisoned linear part was never examined on the
/// revolute branch, and a poisoned angular part routed a revolute axis into the
/// prismatic branch and was accepted there as a prismatic joint.
TEMPLATE_TEST_CASE("screw_axis::from_vector refuses a nonfinite component on both branches",
    "[nonfinite][boundary]", double, float)
{
    using S = TestType;
    cartan::vector6<S> revolute;
    revolute << S(0), S(0), S(1), S(0), S(0), S(0);
    cartan::vector6<S> prismatic;
    prismatic << S(0), S(0), S(0), S(1), S(0), S(0);
    for (S poison : nonfinite_values<S>())
    {
        for (int branch = 0; branch < 2; ++branch)
        {
            for (int i = 0; i < 6; ++i)
            {
                cartan::vector6<S> poisoned = branch == 0 ? revolute : prismatic;
                poisoned(i) = poison;
                auto result = cartan::screw_axis<S>::from_vector(poisoned);
                REQUIRE_FALSE(result.has_value());
                REQUIRE(result.error() == cartan::lie_failure::non_finite_input);
            }
        }
    }
}

TEMPLATE_TEST_CASE("joint_limits::make refuses a nonfinite bound",
    "[nonfinite][boundary]", double, float)
{
    using S = TestType;
    const S inf_b = std::numeric_limits<S>::infinity();
    const S nan_b = std::numeric_limits<S>::quiet_NaN();

    REQUIRE_FALSE(cartan::joint_limits<S>::make(nan_b, S(1)).has_value());
    REQUIRE_FALSE(cartan::joint_limits<S>::make(S(-1), nan_b).has_value());
    REQUIRE_FALSE(cartan::joint_limits<S>::make(inf_b, inf_b).has_value());
    REQUIRE_FALSE(cartan::joint_limits<S>::make(-inf_b, -inf_b).has_value());

    for (S poison : nonfinite_values<S>())
    {
        REQUIRE_FALSE(cartan::joint_limits<S>::make(S(-1), S(1), poison).has_value());
        REQUIRE_FALSE(
            cartan::joint_limits<S>::make(S(-1), S(1), std::nullopt, poison).has_value());
        REQUIRE_FALSE(cartan::joint_limits<S>::make(S(-1), S(1), std::nullopt, std::nullopt, poison)
                          .has_value());
    }
}

TEMPLATE_TEST_CASE("limit feasibility refuses a nonfinite joint value against any bounds",
    "[nonfinite][boundary]", double, float)
{
    using S = TestType;
    const S inf_b = std::numeric_limits<S>::infinity();
    for (S poison : nonfinite_values<S>())
    {
        REQUIRE_FALSE(feasible(poison, S(-3), S(3)));
        REQUIRE_FALSE(feasible(poison, -inf_b, inf_b));
        REQUIRE_FALSE(feasible(poison, -inf_b, S(3)));
        REQUIRE_FALSE(feasible(poison, S(-3), inf_b));
    }
}

TEMPLATE_TEST_CASE("the analytical verifier refuses a nonfinite candidate whether or not "
                   "orientation is checked",
    "[nonfinite][boundary]", double, float)
{
    using S = TestType;
    auto chain = cartan::fixtures::make_six_joint_dynamic_chain<S>();
    Eigen::VectorX<S> home = Eigen::VectorX<S>::Zero(6);
    auto reference = cartan::forward_kinematics(chain, home);
    REQUIRE(reference.has_value());

    expect_candidates_refused<decltype(chain), S>(chain, reference->end_effector, false);
    expect_candidates_refused<decltype(chain), S>(chain, reference->end_effector, true);
}

/// Over-rejecting an infinity is the regression a phase about refusing things
/// is most likely to introduce, and these two are the guard against it: the
/// unbounded joint is *encoded* as a pair of infinities.
TEMPLATE_TEST_CASE("a finite joint value against an infinite bound stays feasible",
    "[nonfinite][boundary]", double, float)
{
    using S = TestType;
    const S inf_b = std::numeric_limits<S>::infinity();
    REQUIRE(feasible(S(1000), -inf_b, inf_b));
    REQUIRE(feasible(S(-1000), -inf_b, inf_b));
    REQUIRE(feasible(S(-1000), -inf_b, S(3)));
    REQUIRE(feasible(S(1000), S(-3), inf_b));
    REQUIRE(feasible(S(0), S(-3), S(3)));
    REQUIRE_FALSE(feasible(S(4), S(-3), S(3)));
}

TEMPLATE_TEST_CASE("the unbounded joint encoding still constructs",
    "[nonfinite][boundary]", double, float)
{
    using S = TestType;
    const S inf_b = std::numeric_limits<S>::infinity();
    auto unbounded = cartan::joint_limits<S>::make(-inf_b, inf_b);
    REQUIRE(unbounded.has_value());
    REQUIRE(unbounded->position_min() == -inf_b);
    REQUIRE(unbounded->position_max() == inf_b);
    REQUIRE(one_joint_chain(-inf_b, inf_b).num_joints() == 1);
}

/// Without this the sweeps above are satisfiable by a factory that refuses
/// everything, which is the failure mode a rejection corpus cannot see.
TEMPLATE_TEST_CASE("the same boundaries still accept well-formed input",
    "[nonfinite][boundary]", double, float)
{
    using S = TestType;
    REQUIRE(cartan::so2<S>::from_matrix(cartan::matrix2<S>::Identity()).has_value());
    REQUIRE(cartan::so3<S>::from_matrix(cartan::matrix3<S>::Identity()).has_value());
    REQUIRE(cartan::se2<S>::from_matrix(cartan::matrix3<S>::Identity()).has_value());
    REQUIRE(cartan::se3<S>::from_matrix(cartan::matrix4<S>::Identity()).has_value());
    REQUIRE(cartan::so3<S>::from_quaternion(cartan::quaternion<S>::Identity()).has_value());
    cartan::vector6<S> revolute;
    revolute << S(0), S(0), S(1), S(0), S(0), S(0);
    REQUIRE(cartan::screw_axis<S>::from_vector(revolute).has_value());
}
