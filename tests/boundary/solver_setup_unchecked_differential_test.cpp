// What the boundaries this plan guards did before the guard, asserted live
// rather than recorded once. Every replica below reproduces the body of a
// shipped function with its new guard removed, and each is bound to the
// function it replicates by an agreement case on well-formed input: a replica
// that has drifted stops agreeing and the drift is a failure, not a silent pass.
//
// Registered only where assertions are compiled out. The replicas call the
// unchecked entry points with a violated precondition, and cartan asserts on the
// nonfinite quaternion the exponential map produces before the behavior under
// demonstration can be observed.

#include "boundary_fixtures.h"

#include <cartan/analytical/detail/fk_verification.h>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>

namespace spp = cartan;

using spp::fixtures::make_dynamic_chain;
using spp::fixtures::make_six_joint_dynamic_chain;
using spp::fixtures::joint_vector;

template <typename Scalar>
using dyn_chain = spp::kinematic_chain<Scalar, spp::dynamic>;

template <typename Scalar>
static spp::se3<Scalar> reachable_target(const dyn_chain<Scalar>& chain)
{
    return spp::forward_kinematics(chain, joint_vector<Scalar>(chain.num_joints(), Scalar(0.2)))
        ->end_effector;
}

/// verify_analytical_solution with its candidate-finiteness test removed, and
/// nothing else changed.
template <typename Scalar, int N>
static bool verify_without_the_finiteness_test(
    const dyn_chain<Scalar>& chain,
    const Eigen::Vector<Scalar, N>& q,
    const spp::se3<Scalar>& target,
    bool check_orientation,
    const spp::verification_tolerance<Scalar>& tolerance
        = spp::default_verification_tolerance_v<Scalar>)
{
    Eigen::Vector<Scalar, Eigen::Dynamic> q_dyn(N);
    for (int i = 0; i < N; ++i)
    {
        q_dyn(i) = q(i);
    }
    auto fk = spp::forward_kinematics_unchecked(chain, q_dyn);

    Scalar position_error = (fk.end_effector.translation() - target.translation()).norm();
    if (position_error >= tolerance.position())
        return false;
    if (!check_orientation)
        return true;
    Scalar orientation_error =
        (fk.end_effector.rotation().inverse() * target.rotation()).log().norm();
    return orientation_error < tolerance.orientation();
}

TEMPLATE_TEST_CASE("the unguarded verifier and the shipped one agree on finite candidates",
    "[analytical][boundary][differential]", double, float)
{
    using Scalar = TestType;
    auto chain = make_dynamic_chain<Scalar>(2);
    auto target = reachable_target<Scalar>(chain);

    for (Scalar first : {Scalar(0.2), Scalar(0.9), Scalar(-0.4)})
    {
        Eigen::Vector<Scalar, 2> q;
        q << first, Scalar(0.2);
        for (bool check_orientation : {false, true})
        {
            REQUIRE(verify_without_the_finiteness_test<Scalar, 2>(
                        chain, q, target, check_orientation)
                == spp::detail::verify_analytical_solution<dyn_chain<Scalar>, 2>(
                    chain, q, target, check_orientation,
                    spp::default_verification_tolerance_v<Scalar>));
        }
    }
}

TEMPLATE_TEST_CASE("the unguarded verifier reports a nonfinite candidate acceptable",
    "[analytical][boundary][differential]", double, float)
{
    using Scalar = TestType;
    auto chain = make_dynamic_chain<Scalar>(2);
    auto target = reachable_target<Scalar>(chain);

    for (Scalar poison : {std::numeric_limits<Scalar>::quiet_NaN(),
             std::numeric_limits<Scalar>::infinity(),
             -std::numeric_limits<Scalar>::infinity()})
    {
        Eigen::Vector<Scalar, 2> q;
        q << poison, Scalar(0.2);

        // Both comparisons are false for a nonfinite pose error, so the position
        // test does not fire and the orientation test is never reached: the
        // candidate is accepted with NaN joint angles. This is the outcome the
        // finiteness test now blocks.
        REQUIRE(verify_without_the_finiteness_test<Scalar, 2>(chain, q, target, false));
        REQUIRE_FALSE(spp::detail::verify_analytical_solution<dyn_chain<Scalar>, 2>(
            chain, q, target, false, spp::default_verification_tolerance_v<Scalar>));
    }
}

/// The forward-kinematics and Jacobian calls a solver's setup made before it
/// validated anything, in the order it made them.
template <typename Scalar>
static Scalar unguarded_setup_error_norm(
    const dyn_chain<Scalar>& chain,
    const spp::se3<Scalar>& target,
    const Eigen::VectorX<Scalar>& q0)
{
    auto fk = spp::forward_kinematics_unchecked(chain, q0);
    auto V_b = (fk.end_effector.inverse() * target).log();
    return V_b.norm();
}

TEMPLATE_TEST_CASE("the unguarded setup path still answers a nonfinite seed",
    "[ik][boundary][differential]", double, float)
{
    using Scalar = TestType;
    auto chain = make_six_joint_dynamic_chain<Scalar>();
    auto target = reachable_target<Scalar>(chain);
    const int n = chain.num_joints();

    auto finite = joint_vector<Scalar>(n, Scalar(0.15));
    REQUIRE(std::isfinite(unguarded_setup_error_norm<Scalar>(chain, target, finite)));

    for (int i = 0; i < n; ++i)
    {
        auto seed = finite;
        seed(i) = std::numeric_limits<Scalar>::quiet_NaN();

        // The unguarded path completes and hands the loop a nonfinite error
        // norm, which is what the solver then iterated on.
        REQUIRE_FALSE(std::isfinite(unguarded_setup_error_norm<Scalar>(chain, target, seed)));

        spp::builtin_lm<dyn_chain<Scalar>> solver;
        solver.setup(chain, target, seed, spp::convergence_criteria<Scalar>{});
        auto stepped = solver.step(chain, 4);
        REQUIRE(stepped.status == spp::ik_status::non_finite_input);
        REQUIRE(stepped.metrics.units_consumed == 0);
    }
}
