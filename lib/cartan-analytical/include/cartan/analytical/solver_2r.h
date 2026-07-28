#ifndef HPP_GUARD_CARTAN_ANALYTICAL_SOLVER_2R_H
#define HPP_GUARD_CARTAN_ANALYTICAL_SOLVER_2R_H

#include "cartan/analytical/analytical_types.h"
#include "cartan/analytical/analytical_solver.h"

#include "cartan/analytical/detail/clamped_trig.h"
#include "cartan/analytical/detail/fk_verification.h"

#include "cartan/serial/chain/joint_tags.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/chain/static_chain.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include "cartan/lie/se3.h"

#include <cmath>
#include <cstddef>
#include "cartan/expected.h"
#include <numbers>
#include <optional>

namespace cartan
{

/// Closed-form IK solver for planar 2R mechanisms (two revolute joints whose
/// axes are parallel and define a common mechanism plane). Returns up to 2
/// solutions ("elbow up" / "elbow down") using the planar law-of-cosines.
/// All candidates are FK-verified; only verified solutions are returned.
///
/// Reference: Lynch & Park, Modern Robotics, Section 6.1.2 (planar
///            two-link inverse kinematics).
template <chain Chain>
class planar_2r_solver
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = 2;
    static constexpr int max_solutions = 2;

    /// The only way to build one of these. Admits exactly the chains the
    /// derivation is written for: two revolute joints on parallel axes, with
    /// the home end-effector within the acceptance length of the plane through
    /// the first axis point, and neither link of zero length. A bent home
    /// (second link at a nonzero angle to the first)
    /// is admitted: the constant home-bend angle is carried into the joint-2
    /// solutions by the constructor, so the closed form reconstructs a
    /// bent-home arm and recovers the straight case when the bend is zero.
    static cartan::expected<planar_2r_solver, analytical_error<scalar_type>>
    make(const Chain& chain,
         verification_tolerance<scalar_type> tolerance
             = default_verification_tolerance_v<scalar_type>)
    {
        if (std::optional<analytical_error<scalar_type>> rejection
                = reject_chain(chain, tolerance.position()))
        {
            return cartan::unexpected(*rejection);
        }

        return planar_2r_solver(chain, tolerance);
    }

    cartan::expected<
        analytical_result<scalar_type, 2, 2>,
        analytical_error<scalar_type>>
    solve(const se3<scalar_type>& target) const
    {
        vector3<Scalar> p_target = target.translation() - m_base_point;

        // Project target onto the mechanism plane using the 2D basis
        Scalar u = p_target.dot(m_basis_u);
        Scalar v = p_target.dot(m_basis_v);

        Scalar out_of_plane = p_target.dot(m_plane_normal);
        Scalar dist = std::hypot(u, v);

        if (std::optional<analytical_error<Scalar>> rejection
                = reject_target(out_of_plane, dist))
        {
            return cartan::unexpected(*rejection);
        }

        Scalar L1 = m_link_length_1;
        Scalar L2 = m_link_length_2;

        // Law of cosines for elbow angle
        Scalar cos_beta = (L1 * L1 + L2 * L2 - dist * dist) / (Scalar(2) * L1 * L2);
        Scalar beta = detail::safe_acos(cos_beta);

        // Shoulder angle helper
        Scalar cos_alpha = (dist * dist + L1 * L1 - L2 * L2) / (Scalar(2) * L1 * dist);
        Scalar alpha = detail::safe_acos(cos_alpha);

        // Base angle to target in the projected plane
        Scalar gamma = std::atan2(v, u);

        analytical_result<Scalar, 2, 2> result;

        // The collinear closed form measures joint 2 from a straight home; a
        // bent home shifts that reference by m_home_bend, so subtract it.
        // Solution 1 (elbow "up" / righty)
        result.solutions[0] = Eigen::Vector<Scalar, 2>(
            gamma - alpha,
            std::numbers::pi_v<Scalar> - beta - m_home_bend);

        // Solution 2 (elbow "down" / lefty)
        result.solutions[1] = Eigen::Vector<Scalar, 2>(
            gamma + alpha,
            beta - std::numbers::pi_v<Scalar> - m_home_bend);

        // Boundary case: both solutions converge when alpha ~ 0
        if (alpha < detail::sqrt_epsilon_v<Scalar>)
        {
            result.count = 1;
        }
        else
        {
            result.count = 2;
        }

        // FK verification: filter out solutions that don't verify
        int verified_count = 0;
        analytical_result<Scalar, 2, 2> verified;
        for (std::size_t i = 0; i < static_cast<std::size_t>(result.count); ++i)
        {
            if (detail::verify_analytical_solution(
                    m_chain, result.solutions[i], target, false, m_tolerance))
            {
                verified.solutions[static_cast<std::size_t>(verified_count++)]
                    = result.solutions[i];
            }
        }
        verified.count = verified_count;

        if (verified_count == 0)
        {
            return cartan::unexpected(analytical_error<Scalar>{
                analytical_failure::verification_failed, std::nullopt});
        }

        return verified;
    }

    const chain_type& chain() const { return m_chain; }

private:
    /// Alias for the chain's floating-point type. Keeps the constructor and
    /// solve() body verbatim after the class template was re-shaped from
    /// <typename Scalar, joint_tag... Joints> to <chain Chain>.
    using Scalar = scalar_type;

    chain_type m_chain;
    Scalar m_link_length_1{};
    Scalar m_link_length_2{};
    Scalar m_home_bend{};
    verification_tolerance<Scalar> m_tolerance;
    vector3<Scalar> m_base_point{vector3<Scalar>::Zero()};
    vector3<Scalar> m_plane_normal{vector3<Scalar>::Zero()};
    vector3<Scalar> m_basis_u{vector3<Scalar>::Zero()};
    vector3<Scalar> m_basis_v{vector3<Scalar>::Zero()};

    planar_2r_solver(
        const Chain& chain,
        verification_tolerance<scalar_type> tolerance)
        : m_chain(chain)
        , m_tolerance(tolerance)
    {
        const auto& s0 = chain.axis(0);
        const auto& s1 = chain.axis(1);

        vector3<Scalar> q0 = s0.omega().cross(s0.v());
        vector3<Scalar> q1 = s1.omega().cross(s1.v());
        vector3<Scalar> p_ee = chain.home().translation();

        m_link_length_1 = (q1 - q0).norm();
        m_link_length_2 = (p_ee - q1).norm();
        m_base_point = q0;

        // The two axes are parallel, so their shared direction is the normal
        // of the plane both rotations move in.
        m_plane_normal = s0.omega();
        m_basis_u = (q1 - q0).normalized();
        m_basis_v = m_plane_normal.cross(m_basis_u);

        // Signed in-plane angle from the first-link direction to the second at
        // the home configuration; carried into the joint-2 solutions so a bent
        // home reconstructs. Zero when the home arm is straight.
        vector3<Scalar> link1 = q1 - q0;
        vector3<Scalar> link2 = p_ee - q1;
        m_home_bend = std::atan2(
            m_plane_normal.dot(link1.cross(link2)), link1.dot(link2));
    }

    /// The chains the planar derivation has no answer for. The two axes must be
    /// parallel, or there is no plane both rotations move in; and the home
    /// end-effector must lie in that plane, or the second link length the closed
    /// form reads off is a three-dimensional distance rather than the in-plane
    /// one it needs. The second joint needs no such test: a screw axis carries
    /// its line rather than a point on it, and the point recovered as omega x v
    /// is the foot of the perpendicular from the origin, so both recovered
    /// points are perpendicular to a shared axis direction and the first link
    /// lies in the plane identically. A separation is not a workspace excess, so
    /// no rejection carries a magnitude.
    static std::optional<analytical_error<Scalar>> reject_chain(
        const Chain& chain, Scalar accept)
    {
        const analytical_error<Scalar> degenerate{
            analytical_failure::degenerate_geometry, std::nullopt};

        if (chain.num_joints() != 2)
            return degenerate;
        for (int i = 0; i < 2; ++i)
            if (!chain.axis(i).is_revolute())
                return degenerate;

        const auto& s0 = chain.axis(0);
        const auto& s1 = chain.axis(1);
        vector3<Scalar> q0 = s0.omega().cross(s0.v());
        vector3<Scalar> q1 = s1.omega().cross(s1.v());
        vector3<Scalar> p_ee = chain.home().translation();

        // A sine between unit directions, so the threshold is machine
        // precision rather than a length.
        if (s0.omega().cross(s1.omega()).norm() > detail::sqrt_epsilon_v<Scalar>)
            return degenerate;
        if (std::abs((p_ee - q0).dot(s0.omega())) > accept)
            return degenerate;
        if ((q1 - q0).norm() < detail::sqrt_epsilon_v<Scalar>
            || (p_ee - q1).norm() < detail::sqrt_epsilon_v<Scalar>)
            return degenerate;

        return std::nullopt;
    }

    /// The targets the planar closed form has no answer for, decided ahead of
    /// the law of cosines that divides by the in-plane distance. Every gate
    /// compares a length against the acceptance length the back-check applies.
    /// An absent diagnostic means the target passed all four.
    std::optional<analytical_error<Scalar>> reject_target(
        Scalar out_of_plane, Scalar dist) const
    {
        Scalar accept = m_tolerance.position();
        Scalar max_reach = m_link_length_1 + m_link_length_2;
        Scalar min_reach = std::abs(m_link_length_1 - m_link_length_2);

        // The mechanism plane is the reachable set, so a target off it is out of
        // reach; the distance out of the plane is that reach violation, not a
        // shadow to be projected onto and solved silently.
        if (std::abs(out_of_plane) > accept)
            return analytical_error<Scalar>{
                analytical_failure::unreachable, std::abs(out_of_plane)};

        // Equal links reaching the base point: the shoulder angle is free and
        // the elbow folds back along the first link, so a continuum of
        // configurations attains the target. The solver does not enumerate that
        // continuum; a caller wanting one member of it fixes one joint angle and
        // solves for the other.
        if (dist <= accept && min_reach <= accept)
            return analytical_error<Scalar>{
                analytical_failure::singular_configuration, std::nullopt};

        if (dist > max_reach + accept)
            return analytical_error<Scalar>{
                analytical_failure::unreachable, dist - max_reach};
        if (dist < min_reach - accept)
            return analytical_error<Scalar>{
                analytical_failure::unreachable, min_reach - dist};

        return std::nullopt;
    }
};

static_assert(analytical_solver<planar_2r_solver<static_chain<double, revolute_z, revolute_z>>>,
    "planar_2r_solver must satisfy analytical_solver concept");

static_assert(analytical_solver<planar_2r_solver<kinematic_chain<double, dynamic>>>,
    "planar_2r_solver must also satisfy analytical_solver concept against dynamic chain");

/// Convenience wrapper around planar_2r_solver: validates the given
/// static_chain through the factory and immediately solves for the target pose.
/// A rejected chain travels out on the same diagnostic channel a failed solve
/// uses, so the two failures need no separate handling at the call site.
template <typename Scalar, joint_tag... Joints>
cartan::expected<analytical_result<Scalar, 2, 2>, analytical_error<Scalar>>
solve_2r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target)
{
    auto solver = planar_2r_solver<static_chain<Scalar, Joints...>>::make(chain);
    if (!solver)
        return cartan::unexpected(solver.error());
    return solver->solve(target);
}

}

#endif
