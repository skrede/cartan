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

    explicit planar_2r_solver(const Chain& chain)
        : m_chain(chain)
    {
        if (chain.num_joints() != 2)
        {
            m_valid = false;
            return;
        }
        for (int i = 0; i < 2; ++i)
        {
            if (!chain.axis(i).is_revolute())
            {
                m_valid = false;
                return;
            }
        }

        const auto& s0 = chain.axis(0);
        const auto& s1 = chain.axis(1);

        vector3<Scalar> q0 = s0.omega().cross(s0.v());
        vector3<Scalar> q1 = s1.omega().cross(s1.v());
        vector3<Scalar> p_ee = chain.home().translation();

        m_link_length_1 = (q1 - q0).norm();
        m_link_length_2 = (p_ee - q1).norm();
        m_base_point = q0;

        // Build an orthonormal basis for the mechanism plane.
        // For a planar 2R, both rotation axes are parallel. The mechanism
        // plane is perpendicular to the rotation axis. For non-parallel axes,
        // the plane normal is the cross product of the two axes.
        vector3<Scalar> omega0 = s0.omega();
        vector3<Scalar> omega1 = s1.omega();
        vector3<Scalar> axis_cross = omega0.cross(omega1);
        Scalar cross_norm = axis_cross.norm();

        if (cross_norm > detail::sqrt_epsilon_v<Scalar>)
        {
            // Non-parallel axes: normal is their cross product
            m_plane_normal = axis_cross / cross_norm;
        }
        else
        {
            // Parallel axes: the rotation axis IS the plane normal
            m_plane_normal = omega0;
        }

        // First in-plane basis: direction from base to second joint
        m_basis_u = (q1 - q0).normalized();
        // Second in-plane basis: complete the right-handed frame
        m_basis_v = m_plane_normal.cross(m_basis_u);

        // Signed in-plane angle from the first-link direction to the second at
        // the home configuration; carried into the joint-2 solutions so a bent
        // home reconstructs. Zero when the home arm is straight.
        vector3<Scalar> link1 = q1 - q0;
        vector3<Scalar> link2 = p_ee - q1;
        m_home_bend = std::atan2(
            m_plane_normal.dot(link1.cross(link2)), link1.dot(link2));
        m_valid = true;
    }

    /// Constructs a validated planar-2R solver. Rejects a chain that is not two
    /// revolute joints or that has a zero-length link. A bent home (second link
    /// at a nonzero angle to the first) is admitted: the constant home-bend
    /// angle is carried into the joint-2 solutions by the constructor, so the
    /// closed form reconstructs a bent-home arm and recovers the straight case
    /// when the bend is zero.
    static cartan::expected<planar_2r_solver, analytical_error<scalar_type>>
    make(const Chain& chain)
    {
        if (chain.num_joints() != 2)
        {
            return cartan::unexpected(analytical_error<scalar_type>{
                analytical_failure::degenerate_geometry, scalar_type(0)});
        }
        for (int i = 0; i < 2; ++i)
        {
            if (!chain.axis(i).is_revolute())
            {
                return cartan::unexpected(analytical_error<scalar_type>{
                    analytical_failure::degenerate_geometry, scalar_type(0)});
            }
        }

        const auto& s0 = chain.axis(0);
        const auto& s1 = chain.axis(1);
        vector3<scalar_type> q0 = s0.omega().cross(s0.v());
        vector3<scalar_type> q1 = s1.omega().cross(s1.v());
        vector3<scalar_type> p_ee = chain.home().translation();

        scalar_type n1 = (q1 - q0).norm();
        scalar_type n2 = (p_ee - q1).norm();
        if (n1 < detail::sqrt_epsilon_v<scalar_type>
            || n2 < detail::sqrt_epsilon_v<scalar_type>)
        {
            return cartan::unexpected(analytical_error<scalar_type>{
                analytical_failure::degenerate_geometry, scalar_type(0)});
        }

        return planar_2r_solver(chain);
    }

    cartan::expected<
        analytical_result<scalar_type, 2, 2>,
        analytical_error<scalar_type>>
    solve(const se3<scalar_type>& target) const
    {
        if (!m_valid)
        {
            return cartan::unexpected(analytical_error<scalar_type>{
                analytical_failure::degenerate_geometry, scalar_type(0)});
        }

        vector3<Scalar> p_target = target.translation() - m_base_point;

        // Project target onto the mechanism plane using the 2D basis
        Scalar u = p_target.dot(m_basis_u);
        Scalar v = p_target.dot(m_basis_v);

        Scalar dist_sq = u * u + v * v;
        Scalar L1 = m_link_length_1;
        Scalar L2 = m_link_length_2;
        Scalar max_reach = L1 + L2;
        Scalar min_reach = std::abs(L1 - L2);

        if (dist_sq > max_reach * max_reach + detail::sqrt_epsilon_v<Scalar>)
        {
            Scalar workspace_dist = std::sqrt(dist_sq) - max_reach;
            return cartan::unexpected(analytical_error<Scalar>{
                analytical_failure::unreachable, workspace_dist});
        }
        if (dist_sq < min_reach * min_reach - detail::sqrt_epsilon_v<Scalar>)
        {
            Scalar workspace_dist = min_reach - std::sqrt(dist_sq);
            return cartan::unexpected(analytical_error<Scalar>{
                analytical_failure::unreachable, workspace_dist});
        }

        // Law of cosines for elbow angle
        Scalar cos_beta = (L1 * L1 + L2 * L2 - dist_sq) / (Scalar(2) * L1 * L2);
        Scalar beta = detail::safe_acos(cos_beta);

        // Shoulder angle helper
        Scalar dist = std::sqrt(dist_sq);
        Scalar cos_alpha = (dist_sq + L1 * L1 - L2 * L2) / (Scalar(2) * L1 * dist);
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
                    m_chain, result.solutions[i], target, false))
            {
                verified.solutions[static_cast<std::size_t>(verified_count++)]
                    = result.solutions[i];
            }
        }
        verified.count = verified_count;

        if (verified_count == 0)
        {
            return cartan::unexpected(analytical_error<Scalar>{
                analytical_failure::verification_failed, Scalar(0)});
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
    vector3<Scalar> m_base_point{vector3<Scalar>::Zero()};
    vector3<Scalar> m_plane_normal{vector3<Scalar>::Zero()};
    vector3<Scalar> m_basis_u{vector3<Scalar>::Zero()};
    vector3<Scalar> m_basis_v{vector3<Scalar>::Zero()};
    bool m_valid{false};
};

template <chain Chain>
planar_2r_solver(const Chain&) -> planar_2r_solver<Chain>;

static_assert(analytical_solver<planar_2r_solver<static_chain<double, revolute_z, revolute_z>>>,
    "planar_2r_solver must satisfy analytical_solver concept");

static_assert(analytical_solver<planar_2r_solver<kinematic_chain<double, dynamic>>>,
    "planar_2r_solver must also satisfy analytical_solver concept against dynamic chain");

/// Convenience wrapper around planar_2r_solver: constructs a solver from the
/// given static_chain and immediately solves for the target pose.
template <typename Scalar, joint_tag... Joints>
auto solve_2r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target)
{
    planar_2r_solver solver(chain);
    return solver.solve(target);
}

}

#endif
