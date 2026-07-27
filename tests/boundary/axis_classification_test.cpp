/// Two-sided evidence for the axis-snap tolerance and for the exact
/// tag-versus-axis check, in both scalars.
///
/// Neither cell this file covers is visible to a sanitizer: a contradicting tag
/// used to construct successfully in a release build and evaluate a robot with
/// one immovable joint, and an over-loose classification used to evaluate a
/// different axis than the one it was given. Both are wrong values, so both are
/// demonstrated as wrong values.

#include <cartan/serial/chain/joint_kind.h>
#include <cartan/serial/chain/joint_tags.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/chain/chain_failure.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/fk/detail/axis_specializations.h>

#include <cartan/lie/se3.h>
#include <cartan/detail/epsilon.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <array>
#include <vector>
#include <limits>

namespace
{

using cartan::joint_kind;

/// The largest per-component deviation any fixture in the tree requires to be
/// snapped, measured across every factory in both scalars. It is bit-identical
/// in float because the fixture literals are narrowed doubles. The comment in
/// the fixture header quotes half this figure; a test written against that
/// figure would pass with half the intended margin.
template <typename Scalar>
constexpr Scalar k_worst_fixture_deviation = Scalar(4.102071e-10);

/// A deviation that must never be snapped.
template <typename Scalar>
constexpr Scalar k_must_not_snap_deviation = Scalar(3e-4);

template <typename Scalar>
cartan::screw_axis<Scalar> revolute_near_z(Scalar deviation)
{
    return cartan::screw_axis<Scalar>::revolute(
        cartan::vector3<Scalar>(deviation, Scalar(0), Scalar(1)),
        cartan::vector3<Scalar>::Zero());
}

/// The per-component deviation an axis actually carries after normalization,
/// so a case asserts against the stored geometry rather than the requested one.
template <typename Scalar>
Scalar stored_deviation_from_z(const cartan::screw_axis<Scalar>& axis)
{
    const auto& w = axis.omega();
    return std::max({std::abs(w(0)), std::abs(w(1)),
                     std::abs(std::abs(w(2)) - Scalar(1))});
}

/// The classification rule as it stood before the absolute tolerance: the same
/// branch structure, with the threshold derived from machine precision. It is
/// what made a physically real misalignment invisible in single precision.
template <typename Scalar>
bool snapped_under_precision_derived_tolerance(const cartan::screw_axis<Scalar>& axis)
{
    const Scalar tol = cartan::detail::sqrt_epsilon_v<Scalar>;
    auto is_unit = [tol](Scalar x)
    { return std::abs(std::abs(x) - Scalar(1)) < tol; };
    auto is_zero = [tol](Scalar x) { return std::abs(x) < tol; };

    const auto& w = axis.omega();
    if (axis.is_revolute())
    {
        return (is_unit(w(0)) && is_zero(w(1)) && is_zero(w(2)))
               || (is_zero(w(0)) && is_unit(w(1)) && is_zero(w(2)))
               || (is_zero(w(0)) && is_zero(w(1)) && is_unit(w(2)));
    }
    const auto& v = axis.v();
    return (is_unit(v(0)) && is_zero(v(1)) && is_zero(v(2)))
           || (is_zero(v(0)) && is_unit(v(1)) && is_zero(v(2)))
           || (is_zero(v(0)) && is_zero(v(1)) && is_unit(v(2)));
}

template <typename Scalar>
cartan::joint_limits<Scalar> wide_limits()
{
    auto made = cartan::joint_limits<Scalar>::make(Scalar(-4), Scalar(4));
    REQUIRE(made.has_value());
    return *made;
}

/// A 3R chain whose second joint carries the given deviation off +z.
template <typename Scalar>
cartan::kinematic_chain<Scalar, 3> chain_with_deviation(Scalar deviation)
{
    using namespace cartan;
    std::array<screw_axis<Scalar>, 3> axes{
        screw_axis<Scalar>::revolute({0, 0, 1}, vector3<Scalar>::Zero()),
        screw_axis<Scalar>::revolute(
            vector3<Scalar>(deviation, Scalar(0), Scalar(1)),
            vector3<Scalar>(Scalar(0), Scalar(0), Scalar(0.5))),
        screw_axis<Scalar>::revolute(
            {0, 1, 0}, vector3<Scalar>(Scalar(0), Scalar(0), Scalar(1)))};
    auto lim = wide_limits<Scalar>();
    std::array<joint_limits<Scalar>, 3> limits{lim, lim, lim};
    return kinematic_chain<Scalar, 3>(
        se3<Scalar>(so3<Scalar>::identity(),
                    vector3<Scalar>(Scalar(0.5), Scalar(0), Scalar(1.5))),
        axes, limits);
}

}

// ============================================================================
// The two-sided classification target
// ============================================================================

TEMPLATE_TEST_CASE("the worst fixture deviation still classifies to its principal kind",
    "[axis_classification]", double, float)
{
    using Scalar = TestType;

    auto axis = revolute_near_z(k_worst_fixture_deviation<Scalar>);
    REQUIRE(stored_deviation_from_z(axis) == k_worst_fixture_deviation<Scalar>);
    REQUIRE(cartan::detect_joint_kind(axis) == joint_kind::revolute_z);
}

TEMPLATE_TEST_CASE("a deviation that must not snap classifies as general",
    "[axis_classification]", double, float)
{
    using Scalar = TestType;

    auto axis = revolute_near_z(k_must_not_snap_deviation<Scalar>);
    REQUIRE(stored_deviation_from_z(axis) > Scalar(1e-5));
    REQUIRE(cartan::detect_joint_kind(axis) == joint_kind::general);
}

// This one is a behavior change in double, where the precision-derived
// threshold used to admit it, and it is the intended half of the change: the
// tolerance is now the same physical angle in both scalars.
TEMPLATE_TEST_CASE("a deviation an order above the tolerance classifies as general",
    "[axis_classification]", double, float)
{
    using Scalar = TestType;

    auto axis = revolute_near_z(Scalar(1e-8));
    REQUIRE(cartan::detect_joint_kind(axis) == joint_kind::general);
}

TEMPLATE_TEST_CASE("an exactly principal axis and its negation share a kind",
    "[axis_classification]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto zero = vector3<Scalar>::Zero();

    REQUIRE(detect_joint_kind(screw_axis<Scalar>::revolute({1, 0, 0}, zero))
            == joint_kind::revolute_x);
    REQUIRE(detect_joint_kind(screw_axis<Scalar>::revolute({-1, 0, 0}, zero))
            == joint_kind::revolute_x);
    REQUIRE(detect_joint_kind(screw_axis<Scalar>::revolute({0, 0, 1}, zero))
            == joint_kind::revolute_z);
    REQUIRE(detect_joint_kind(screw_axis<Scalar>::revolute({0, 0, -1}, zero))
            == joint_kind::revolute_z);
    REQUIRE(detect_joint_kind(screw_axis<Scalar>::prismatic({0, -1, 0}))
            == joint_kind::prismatic_y);
}

TEMPLATE_TEST_CASE("a nonfinite rotation axis classifies as general",
    "[axis_classification]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    for (Scalar poison : {std::numeric_limits<Scalar>::quiet_NaN(),
                          std::numeric_limits<Scalar>::infinity(),
                          -std::numeric_limits<Scalar>::infinity()})
    {
        auto axis = screw_axis<Scalar>::revolute(
            vector3<Scalar>(poison, Scalar(0), Scalar(1)), vector3<Scalar>::Zero());
        REQUIRE_FALSE(axis.to_vector().allFinite());
        REQUIRE(detect_joint_kind(axis) == joint_kind::general);
    }
}

// The classifier is not a finiteness gate, and the half it cannot see is worth
// pinning rather than assuming: on the revolute branch it reads only omega, so
// a nonfinite linear component leaves the axis classified as a principal
// revolute kind. The construction factory's finiteness check is what refuses
// it, which is why that check reads the whole six-vector.
TEMPLATE_TEST_CASE("a nonfinite linear component leaves the classification principal",
    "[axis_classification]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto axis = screw_axis<Scalar>::revolute(
        {0, 0, 1},
        vector3<Scalar>(std::numeric_limits<Scalar>::quiet_NaN(), Scalar(0), Scalar(0)));

    REQUIRE_FALSE(axis.to_vector().allFinite());
    REQUIRE(detect_joint_kind(axis) == joint_kind::revolute_z);

    auto refused = static_chain<Scalar, revolute_z>::make(
        se3<Scalar>::identity(), {axis}, {wide_limits<Scalar>()});
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == chain_failure::non_finite_input);
}

// Scalar independence is the property the absolute tolerance buys, so it is
// asserted directly rather than inferred from the two per-scalar runs above.
TEST_CASE("the classification agrees in single and double precision",
    "[axis_classification]")
{
    const std::vector<double> deviations{
        0.0, 4.102071e-10, 1e-9, 1e-8, 1e-6, 3e-4, 1e-2};

    for (double deviation : deviations)
    {
        auto in_double = cartan::detect_joint_kind(revolute_near_z<double>(deviation));
        auto in_float
            = cartan::detect_joint_kind(revolute_near_z<float>(static_cast<float>(deviation)));
        REQUIRE(in_double == in_float);
    }
}

// ============================================================================
// What the over-loose threshold was doing, demonstrated
// ============================================================================

// Single precision only: sqrt(epsilon) is 3.45e-4 in float and 1.49e-8 in
// double, so this deviation was snapped in float and refused in double. The
// assertion is on the shipped classifier against a replica of the threshold it
// replaced, then on the two models the disagreement selects between.
TEST_CASE("the precision-derived threshold snapped an axis the tolerance refuses",
    "[axis_classification][differential]")
{
    const float deviation = k_must_not_snap_deviation<float>;
    auto axis = revolute_near_z(deviation);

    REQUIRE(snapped_under_precision_derived_tolerance(axis));
    REQUIRE(cartan::detect_joint_kind(axis) == joint_kind::general);

    auto true_model = chain_with_deviation(deviation);
    auto snapped_model = chain_with_deviation(0.0f);
    REQUIRE(true_model.kind(1) == joint_kind::general);
    REQUIRE(snapped_model.kind(1) == joint_kind::revolute_z);

    Eigen::Vector3f q(1.1f, 0.9f, -0.6f);
    auto fk_true = cartan::forward_kinematics(true_model, q);
    auto fk_snapped = cartan::forward_kinematics(snapped_model, q);
    REQUIRE(fk_true.has_value());
    REQUIRE(fk_snapped.has_value());

    float divergence = (fk_true->end_effector.translation()
                        - fk_snapped->end_effector.translation()).norm();

    // |dp| <= 1.4 * n * L * delta, at n = 3 joints and L = 1.5 m of reach.
    const float licensed
        = 1.4f * 3.0f * 1.5f * cartan::detail::k_axis_snap_tolerance_v<float>;
    REQUIRE(divergence > licensed);
}

// The replica above is only evidence while it still describes the classifier's
// shape. Nothing in the case that uses it enforces that, so it is anchored on a
// corpus far from either threshold, where the two must agree. This binds the
// branch structure and the either-sign rule, not the threshold -- the threshold
// is the one thing the two deliberately disagree about.
TEMPLATE_TEST_CASE("the replaced threshold agrees with the classifier away from both",
    "[axis_classification][differential]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto zero = vector3<Scalar>::Zero();
    const std::array<screw_axis<Scalar>, 6> corpus{
        screw_axis<Scalar>::revolute({1, 0, 0}, zero),
        screw_axis<Scalar>::revolute({0, -1, 0}, zero),
        screw_axis<Scalar>::revolute({0, 0, 1}, zero),
        screw_axis<Scalar>::revolute({1, 1, 1}, zero),
        revolute_near_z(Scalar(1e-1)),
        screw_axis<Scalar>::prismatic({0, 0, -1})};

    for (const auto& axis : corpus)
    {
        bool classifier_snaps = detect_joint_kind(axis) != joint_kind::general;
        REQUIRE(snapped_under_precision_derived_tolerance(axis) == classifier_snaps);
    }
}

// ============================================================================
// What a contradicting tag was doing, demonstrated
// ============================================================================

// exp_joint is the shipped tag-dispatched kernel, so this is the wrong model
// itself rather than a description of it: under a revolute_z tag it reads
// omega(2) as the signed magnitude, and for a screw about y that component is
// zero, so the joint contributes nothing at any joint value. The release build
// used to construct such a chain without complaint and return this.
TEMPLATE_TEST_CASE("a contradicting tag is refused, and used to freeze the joint",
    "[axis_classification][differential]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto about_y = screw_axis<Scalar>::revolute({0, 1, 0}, vector3<Scalar>::Zero());
    auto lim = wide_limits<Scalar>();

    auto refused = static_chain<Scalar, revolute_z>::make(
        se3<Scalar>::identity(), {about_y}, {lim});
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == chain_failure::tag_axis_contradiction);

    const Scalar theta = Scalar(0.7);
    auto under_wrong_tag = detail::exp_joint<revolute_z>(theta, about_y);
    auto true_step = se3<Scalar>::exp(about_y.to_vector() * theta);

    REQUIRE((under_wrong_tag.matrix() - matrix4<Scalar>::Identity()).norm()
            == Scalar(0));
    REQUIRE((true_step.matrix() - matrix4<Scalar>::Identity()).norm()
            > Scalar(0.5));
}

// The factory's finiteness guard closes the hole the normalizing screw-axis
// factories leave open by design: they divide by the input's norm and hand back
// a nonfinite axis without complaint. The dynamic chain refuses the same value
// today, so the poisoned evaluation it used to produce is a recorded run rather
// than a live one; what stays live is the poisoned value reaching the factory.
TEMPLATE_TEST_CASE("the factory refuses a nonfinite axis the axis factories admit",
    "[axis_classification][differential]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto lim = wide_limits<Scalar>();
    auto poisoned = screw_axis<Scalar>::revolute(
        vector3<Scalar>(std::numeric_limits<Scalar>::quiet_NaN(), Scalar(0), Scalar(1)),
        vector3<Scalar>::Zero());

    REQUIRE_FALSE(poisoned.to_vector().allFinite());

    auto refused = static_chain<Scalar, revolute_z>::make(
        se3<Scalar>::identity(), {poisoned}, {lim});
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == chain_failure::non_finite_input);
}

// A nonfinite home pose is multiplied into every returned pose, so it poisons
// the answer rather than being discarded. Composition is the shipped path and
// carries no validity assert, so this runs in every configuration.
TEMPLATE_TEST_CASE("the factory refuses a home pose that would poison every result",
    "[axis_classification][differential]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto lim = wide_limits<Scalar>();
    auto good = screw_axis<Scalar>::revolute({0, 0, 1}, vector3<Scalar>::Zero());
    auto poisoned_home = se3<Scalar>(
        so3<Scalar>::identity(),
        vector3<Scalar>(std::numeric_limits<Scalar>::quiet_NaN(), Scalar(0), Scalar(0)));

    auto step = se3<Scalar>::exp(good.to_vector() * Scalar(0.3));
    REQUIRE_FALSE((step * poisoned_home).matrix().allFinite());

    auto refused = static_chain<Scalar, revolute_z>::make(
        poisoned_home, {good}, {lim});
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == chain_failure::non_finite_input);
}
