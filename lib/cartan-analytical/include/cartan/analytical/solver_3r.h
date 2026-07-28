#ifndef HPP_GUARD_CARTAN_ANALYTICAL_SOLVER_3R_H
#define HPP_GUARD_CARTAN_ANALYTICAL_SOLVER_3R_H

#include "cartan/analytical/analytical_types.h"
#include "cartan/analytical/analytical_solver.h"
#include "cartan/analytical/paden_kahan.h"
#include "cartan/analytical/detail/axis_rotation.h"
#include "cartan/analytical/detail/wrist_center.h"
#include "cartan/analytical/detail/fk_verification.h"

#include "cartan/serial/chain/joint_tags.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/chain/static_chain.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include "cartan/lie/se3.h"
#include "cartan/detail/epsilon.h"

#include <array>
#include <cmath>
#include <optional>
#include "cartan/expected.h"

namespace cartan
{

/// Closed-form IK solver for spatial 3R mechanisms using Paden-Kahan subproblems.
///
/// Solves position-only IK for a 3-joint revolute chain. Returns up to 4
/// solutions. Requires that the first two joint axes intersect at a common
/// point (the standard configuration for 3R mechanisms, e.g. spherical wrists
/// with an offset third joint).
///
/// Decomposition (Murray, Li, Sastry, Section 3.3.2):
///   1. SP3 finds up to 2 candidates for theta3 via a distance constraint.
///   2. SP2 finds up to 2 (theta1, theta2) pairs for each theta3 candidate.
///   3. All candidates are FK-verified; only verified solutions are returned.
///
/// Reference: Murray, Li and Sastry, A Mathematical Introduction to Robotic
///            Manipulation (1994), Section 3.3.
template <chain Chain>
class spatial_3r_solver
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = 3;
    static constexpr int max_solutions = 4;

    /// The only way to build one of these. Admits exactly the chains the
    /// subproblem decomposition is written for: three revolute joints whose
    /// first two axes are not parallel and meet within the acceptance length,
    /// and whose home end-effector is off the third axis by more than that
    /// length. The reference point the decomposition rotates about is derived
    /// once here rather than on every solve.
    static cartan::expected<spatial_3r_solver, analytical_error<scalar_type>>
    make(const Chain& chain,
         verification_tolerance<scalar_type> tolerance
             = default_verification_tolerance_v<scalar_type>)
    {
        if (std::optional<analytical_error<scalar_type>> rejection
                = reject_chain(chain, tolerance.position()))
        {
            return cartan::unexpected(*rejection);
        }

        return spatial_3r_solver(chain, tolerance);
    }

    cartan::expected<
        analytical_result<scalar_type, 3, 4>,
        analytical_error<scalar_type>>
    solve(const se3<scalar_type>& target) const
    {
        vector3<Scalar> p_target = target.translation();
        const vector3<Scalar>& r = m_reference;

        // SP3: find theta3 such that ||rot(w3,q3,t3)*p_ee - r|| = ||p_d - r||.
        // Rotation about axes 1-2 through r preserves distance from r, so the
        // distance constraint decouples theta3 from theta1, theta2.
        Scalar delta = (p_target - r).norm();

        auto sp3_result = paden_kahan_3(m_omega[2], m_q[2], m_p_ee, r, delta);
        if (!sp3_result)
        {
            return cartan::unexpected(
                subproblem_error<Scalar>(sp3_result.error()));
        }

        analytical_result<Scalar, 3, 4> result;

        for (int i = 0; i < sp3_result->count; ++i)
        {
            Scalar theta3 = sp3_result->solutions[static_cast<std::size_t>(i)];

            vector3<Scalar> p_prime = detail::rotate_point_about_axis(
                m_omega[2], m_q[2], m_p_ee, theta3);

            // SP2: find (theta1, theta2) such that
            //   exp(S1*t1) * exp(S2*t2) * p' = p_d
            // with both axes referenced to their intersection point r.
            auto sp2_result = paden_kahan_2(
                m_omega[0], m_omega[1], r, p_prime, p_target);
            if (!sp2_result)
                continue;

            for (int j = 0; j < sp2_result->count; ++j)
            {
                auto [theta1, theta2] = sp2_result->solutions[static_cast<std::size_t>(j)];

                Eigen::Vector<Scalar, 3> q_candidate;
                q_candidate << theta1, theta2, theta3;

                if (detail::verify_analytical_solution(
                        m_chain, q_candidate, target, false, m_tolerance))
                {
                    result.solutions[static_cast<std::size_t>(result.count)] = q_candidate;
                    ++result.count;
                    if (result.count >= max_solutions)
                        return result;
                }
            }
        }

        if (result.count > 0)
            return result;

        return cartan::unexpected(analytical_error<Scalar>{
            analytical_failure::verification_failed, std::nullopt});
    }

    const chain_type& chain() const { return m_chain; }

private:
    /// Alias for the chain's floating-point type. Keeps the private helpers and
    /// solve() body verbatim after the class template was re-shaped from
    /// <typename Scalar, joint_tag... Joints> to <chain Chain>.
    using Scalar = scalar_type;

    chain_type m_chain;
    std::array<vector3<Scalar>, 3> m_omega;
    std::array<vector3<Scalar>, 3> m_q;
    verification_tolerance<Scalar> m_tolerance;
    vector3<Scalar> m_p_ee;
    vector3<Scalar> m_reference;

    spatial_3r_solver(
        const Chain& chain,
        verification_tolerance<scalar_type> tolerance)
        : m_chain(chain)
        , m_omega{vector3<Scalar>::Zero(), vector3<Scalar>::Zero(), vector3<Scalar>::Zero()}
        , m_q{vector3<Scalar>::Zero(), vector3<Scalar>::Zero(), vector3<Scalar>::Zero()}
        , m_tolerance(tolerance)
        , m_p_ee(chain.home().translation())
        , m_reference(vector3<Scalar>::Zero())
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            const auto& s = chain.axis(static_cast<int>(i));
            m_omega[i] = s.omega();
            m_q[i] = s.omega().cross(s.v());
        }
        m_reference = detail::closest_approach_midpoint<Scalar>(
            m_q[0], m_omega[0], m_q[1], m_omega[1]);
    }

    /// The chains the subproblem decomposition has no answer for. The first two
    /// axes must meet, because the decomposition rotates about their common
    /// point; they must not be parallel first, because the closest-approach
    /// helpers answer a parallel pair with a fallback point rather than
    /// reporting, so an intersection test alone would pass vacuously. The home
    /// end-effector must be off the third axis, or the distance constraint is
    /// met by every angle or by none. A separation is not a workspace excess,
    /// so no rejection carries a magnitude.
    static std::optional<analytical_error<Scalar>> reject_chain(
        const Chain& chain, Scalar accept)
    {
        const analytical_error<Scalar> degenerate{
            analytical_failure::degenerate_geometry, std::nullopt};

        if (chain.num_joints() != 3)
            return degenerate;
        for (int i = 0; i < 3; ++i)
            if (!chain.axis(i).is_revolute())
                return degenerate;

        const auto& s0 = chain.axis(0);
        const auto& s1 = chain.axis(1);
        const auto& s2 = chain.axis(2);

        if (s0.omega().cross(s1.omega()).norm() <= detail::sqrt_epsilon_v<Scalar>)
            return degenerate;
        if (detail::closest_approach_distance<Scalar>(
                s0.omega().cross(s0.v()), s0.omega(),
                s1.omega().cross(s1.v()), s1.omega()) > accept)
            return degenerate;

        vector3<Scalar> to_ee
            = chain.home().translation() - s2.omega().cross(s2.v());
        if ((to_ee - to_ee.dot(s2.omega()) * s2.omega()).norm() <= accept)
            return degenerate;

        return std::nullopt;
    }
};

static_assert(analytical_solver<spatial_3r_solver<static_chain<double, revolute_z, revolute_y, revolute_z>>>);

static_assert(analytical_solver<spatial_3r_solver<kinematic_chain<double, dynamic>>>,
    "spatial_3r_solver must also satisfy analytical_solver concept against dynamic chain");

/// Convenience wrapper around spatial_3r_solver: validates the given
/// static_chain through the factory and immediately solves for the target pose.
/// A rejected chain travels out on the same diagnostic channel a failed solve
/// uses, so the two failures need no separate handling at the call site.
template <typename Scalar, joint_tag... Joints>
cartan::expected<analytical_result<Scalar, 3, 4>, analytical_error<Scalar>>
solve_3r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target)
{
    auto solver = spatial_3r_solver<static_chain<Scalar, Joints...>>::make(chain);
    if (!solver)
        return cartan::unexpected(solver.error());
    return solver->solve(target);
}

}

#endif
