#include <cartan/serial/chain/joint_limits.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <limits>

namespace
{

template <typename Scalar>
Scalar nan_v()
{
    return std::numeric_limits<Scalar>::quiet_NaN();
}

template <typename Scalar>
Scalar inf_v()
{
    return std::numeric_limits<Scalar>::infinity();
}

}

TEMPLATE_TEST_CASE("joint_limits::make reports back what it accepted",
    "[joint_limits]", double, float)
{
    using S = TestType;
    auto made = cartan::joint_limits<S>::make(S(-1), S(1), S(2), S(50), S(10));
    REQUIRE(made.has_value());
    REQUIRE(made->position_min() == S(-1));
    REQUIRE(made->position_max() == S(1));
    REQUIRE(made->velocity_max() == S(2));
    REQUIRE(made->effort_max() == S(50));
    REQUIRE(made->acceleration_max() == S(10));

    auto bare = cartan::joint_limits<S>::make(S(-1), S(1));
    REQUIRE(bare.has_value());
    REQUIRE_FALSE(bare->velocity_max().has_value());
    REQUIRE_FALSE(bare->effort_max().has_value());
    REQUIRE_FALSE(bare->acceleration_max().has_value());
}

TEMPLATE_TEST_CASE("joint_limits::make rejects reversed position bounds",
    "[joint_limits]", double, float)
{
    using S = TestType;
    auto reversed = cartan::joint_limits<S>::make(S(1), S(-1));
    REQUIRE_FALSE(reversed.has_value());
    REQUIRE(reversed.error() == cartan::chain_failure::reversed_position_bounds);

    auto flipped_infinities = cartan::joint_limits<S>::make(inf_v<S>(), -inf_v<S>());
    REQUIRE_FALSE(flipped_infinities.has_value());
    REQUIRE(flipped_infinities.error() == cartan::chain_failure::reversed_position_bounds);

    REQUIRE(cartan::joint_limits<S>::make(S(1), S(1)).has_value());
}

/// An infinity is only a bound when it is signed outward. Both same-sign pairs
/// survive an ordering test, because an infinity is not less than itself, and
/// the object they would produce is read two contradictory ways downstream: an
/// empty feasible set by contains, a full +/-fallback box by the helpers that
/// substitute finite values for infinite bounds.
TEMPLATE_TEST_CASE("joint_limits::make rejects a same-sign infinite pair",
    "[joint_limits]", double, float)
{
    using S = TestType;
    for (S bound : {inf_v<S>(), -inf_v<S>()})
    {
        auto made = cartan::joint_limits<S>::make(bound, bound);
        REQUIRE_FALSE(made.has_value());
        REQUIRE(made.error() == cartan::chain_failure::reversed_position_bounds);
    }

    REQUIRE(cartan::joint_limits<S>::make(-inf_v<S>(), inf_v<S>()).has_value());
}

TEMPLATE_TEST_CASE("joint_limits::make rejects a NaN position bound",
    "[joint_limits]", double, float)
{
    using S = TestType;
    for (auto made : {cartan::joint_limits<S>::make(nan_v<S>(), S(1)),
             cartan::joint_limits<S>::make(S(-1), nan_v<S>()),
             cartan::joint_limits<S>::make(nan_v<S>(), nan_v<S>())})
    {
        REQUIRE_FALSE(made.has_value());
        REQUIRE(made.error() == cartan::chain_failure::non_finite_input);
    }
}

TEMPLATE_TEST_CASE("joint_limits::make accepts the unbounded joint encoding",
    "[joint_limits]", double, float)
{
    using S = TestType;
    auto unbounded = cartan::joint_limits<S>::make(-inf_v<S>(), inf_v<S>());
    REQUIRE(unbounded.has_value());
    REQUIRE(unbounded->position_min() == -inf_v<S>());
    REQUIRE(unbounded->position_max() == inf_v<S>());

    REQUIRE(cartan::joint_limits<S>::make(-inf_v<S>(), S(1)).has_value());
    REQUIRE(cartan::joint_limits<S>::make(S(-1), inf_v<S>()).has_value());
}

TEMPLATE_TEST_CASE("joint_limits::make rejects a negative dynamic bound",
    "[joint_limits]", double, float)
{
    using S = TestType;
    auto velocity = cartan::joint_limits<S>::make(S(-1), S(1), S(-2));
    REQUIRE_FALSE(velocity.has_value());
    REQUIRE(velocity.error() == cartan::chain_failure::negative_velocity_limit);

    auto effort = cartan::joint_limits<S>::make(S(-1), S(1), S(2), S(-50));
    REQUIRE_FALSE(effort.has_value());
    REQUIRE(effort.error() == cartan::chain_failure::negative_effort_limit);

    auto acceleration = cartan::joint_limits<S>::make(S(-1), S(1), S(2), S(50), S(-10));
    REQUIRE_FALSE(acceleration.has_value());
    REQUIRE(acceleration.error() == cartan::chain_failure::negative_acceleration_limit);

    REQUIRE(cartan::joint_limits<S>::make(S(-1), S(1), S(0), S(0), S(0)).has_value());
}

TEMPLATE_TEST_CASE("joint_limits::make rejects a nonfinite dynamic bound",
    "[joint_limits]", double, float)
{
    using S = TestType;
    for (S poison : {nan_v<S>(), inf_v<S>(), -inf_v<S>()})
    {
        auto velocity = cartan::joint_limits<S>::make(S(-1), S(1), poison);
        REQUIRE_FALSE(velocity.has_value());
        REQUIRE(velocity.error() == cartan::chain_failure::non_finite_input);

        auto effort = cartan::joint_limits<S>::make(S(-1), S(1), S(2), poison);
        REQUIRE_FALSE(effort.has_value());
        REQUIRE(effort.error() == cartan::chain_failure::non_finite_input);

        auto acceleration = cartan::joint_limits<S>::make(S(-1), S(1), S(2), S(50), poison);
        REQUIRE_FALSE(acceleration.has_value());
        REQUIRE(acceleration.error() == cartan::chain_failure::non_finite_input);
    }
}

TEMPLATE_TEST_CASE("joint_limits::contains has no answer for a nonfinite position",
    "[joint_limits]", double, float)
{
    using S = TestType;
    auto lim = cartan::joint_limits<S>::make(S(-1), S(1)).value();
    REQUIRE(lim.contains(S(0)) == true);
    REQUIRE(lim.contains(S(-1)) == true);
    REQUIRE(lim.contains(S(1)) == true);
    REQUIRE(lim.contains(S(4)) == false);
    REQUIRE_FALSE(lim.contains(nan_v<S>()).has_value());
    REQUIRE_FALSE(lim.contains(inf_v<S>()).has_value());

    REQUIRE(lim.contains_or(S(0), false));
    REQUIRE_FALSE(lim.contains_or(S(4), true));
    REQUIRE_FALSE(lim.contains_or(nan_v<S>(), false));
    REQUIRE(lim.contains_or(nan_v<S>(), true));
}

/// The pre-change type: five public fields, no validation, and a contains()
/// answering through two comparisons. Constructing it reversed is the wrong
/// answer the factory now refuses -- a joint whose feasible set is empty
/// everywhere, reported as though it were merely out of range.
template <typename Scalar>
struct unvalidated_limits
{
    Scalar position_min;
    Scalar position_max;

    bool contains(Scalar position) const
    {
        return position >= position_min && position <= position_max;
    }
};

TEMPLATE_TEST_CASE("the unvalidated type admits bounds the factory refuses",
    "[joint_limits][boundary]", double, float)
{
    using S = TestType;
    const unvalidated_limits<S> reversed{S(1), S(-1)};
    for (S q : {S(-2), S(-1), S(0), S(1), S(2)})
    {
        REQUIRE_FALSE(reversed.contains(q));
    }
    REQUIRE_FALSE(cartan::joint_limits<S>::make(S(1), S(-1)).has_value());

    const unvalidated_limits<S> same_sign_infinities{inf_v<S>(), inf_v<S>()};
    REQUIRE_FALSE(same_sign_infinities.contains(S(0)));
    REQUIRE_FALSE(cartan::joint_limits<S>::make(inf_v<S>(), inf_v<S>()).has_value());
}

/// Pins the replica's comparison to the shipped one, so it cannot drift into
/// describing a containment rule the library no longer applies. Constructibility
/// is the half that cannot be pinned: the spelling the replica reproduces no
/// longer exists, which is the whole point of the change.
TEMPLATE_TEST_CASE("the unvalidated type agrees with contains on sound bounds",
    "[joint_limits][boundary]", double, float)
{
    using S = TestType;
    for (S lo : {S(-3), S(0)})
    {
        for (S hi : {S(3), S(0)})
        {
            auto made = cartan::joint_limits<S>::make(lo, hi);
            if (!made.has_value())
            {
                continue;
            }
            const unvalidated_limits<S> replica{lo, hi};
            for (S q : {S(-4), S(-3), S(-0.5), S(0), S(0.5), S(3), S(4)})
            {
                REQUIRE(replica.contains(q) == made->contains(q).value());
            }
        }
    }
}
