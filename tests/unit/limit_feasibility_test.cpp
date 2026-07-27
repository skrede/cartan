#include <cartan/serial/ik/detail/limit_enforcement.h>

#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <vector>
#include <limits>

namespace
{

template <typename Scalar>
cartan::kinematic_chain<Scalar, cartan::dynamic> one_joint_chain(Scalar lo, Scalar hi)
{
    std::vector<cartan::screw_axis<Scalar>> axes = {
        cartan::screw_axis<Scalar>::revolute({0, 0, 1}, {0, 0, 0})};
    std::vector<cartan::joint_limits<Scalar>> limits = {
        *cartan::joint_limits<Scalar>::make(lo, hi)};
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

/// The feasibility comparison as it stood before the joint value itself was
/// tested: each bound guarded by its own finiteness test, and nothing looking at
/// q. Every comparison against a NaN is false, so it answered "feasible" for
/// one -- and for an unbounded joint neither comparison ran at all.
///
/// A replica is only evidence while it still describes the shipped predicate,
/// which is what the agreement case below is for.
template <typename Scalar>
bool unguarded_feasibility_admits(Scalar q, Scalar lo, Scalar hi, Scalar tol)
{
    if (std::isfinite(lo) && q < lo - tol)
    {
        return false;
    }
    if (std::isfinite(hi) && q > hi + tol)
    {
        return false;
    }
    return true;
}

}

TEMPLATE_TEST_CASE("within_limits refuses a nonfinite joint value the unguarded "
                   "comparison admitted",
    "[within_limits][boundary]", double, float)
{
    using S = TestType;
    const S tol = cartan::detail::default_feasibility_tol<S>();
    const S nan_q = std::numeric_limits<S>::quiet_NaN();
    const S inf_b = std::numeric_limits<S>::infinity();

    REQUIRE(unguarded_feasibility_admits(nan_q, S(-3), S(3), tol));
    REQUIRE_FALSE(feasible(nan_q, S(-3), S(3)));

    REQUIRE(unguarded_feasibility_admits(nan_q, -inf_b, inf_b, tol));
    REQUIRE_FALSE(feasible(nan_q, -inf_b, inf_b));

    REQUIRE(unguarded_feasibility_admits(inf_b, -inf_b, inf_b, tol));
    REQUIRE_FALSE(feasible(inf_b, -inf_b, inf_b));
    REQUIRE_FALSE(feasible(-inf_b, S(-3), S(3)));
}

TEMPLATE_TEST_CASE("within_limits leaves the unbounded joint path alone",
    "[within_limits][boundary]", double, float)
{
    using S = TestType;
    const S inf_b = std::numeric_limits<S>::infinity();

    REQUIRE(feasible(S(1000), -inf_b, inf_b));
    REQUIRE(feasible(S(-1000), -inf_b, inf_b));
    REQUIRE(feasible(S(-1000), -inf_b, S(3)));
    REQUIRE_FALSE(feasible(S(1000), -inf_b, S(3)));
    REQUIRE(feasible(S(1000), S(-3), inf_b));
    REQUIRE_FALSE(feasible(S(-1000), S(-3), inf_b));
}

TEMPLATE_TEST_CASE("within_limits accepts inside the box and refuses outside it",
    "[within_limits][boundary]", double, float)
{
    using S = TestType;
    REQUIRE(feasible(S(0), S(-3), S(3)));
    REQUIRE(feasible(S(-3), S(-3), S(3)));
    REQUIRE(feasible(S(3), S(-3), S(3)));
    REQUIRE_FALSE(feasible(S(3.5), S(-3), S(3)));
    REQUIRE_FALSE(feasible(S(-3.5), S(-3), S(3)));
}

/// Binds the replica above to the shipped predicate: without this the replica
/// could drift into describing a comparison the library no longer makes, and the
/// differential case would keep passing while demonstrating nothing.
TEMPLATE_TEST_CASE("the unguarded comparison agrees with within_limits on finite input",
    "[within_limits][boundary]", double, float)
{
    using S = TestType;
    const S tol = cartan::detail::default_feasibility_tol<S>();
    const S inf_b = std::numeric_limits<S>::infinity();

    for (S lo : {S(-3), -inf_b})
    {
        for (S hi : {S(3), inf_b})
        {
            for (S q : {S(0), S(-3), S(3), S(3.5), S(-3.5), S(3) + tol / S(2),
                     S(3) + tol * S(4), S(-3) - tol / S(2), S(-3) - tol * S(4)})
            {
                REQUIRE(unguarded_feasibility_admits(q, lo, hi, tol) == feasible(q, lo, hi));
            }
        }
    }
}
