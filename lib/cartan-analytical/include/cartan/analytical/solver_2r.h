#ifndef HPP_GUARD_LIEPP_ANALYTICAL_SOLVER_2R_H
#define HPP_GUARD_LIEPP_ANALYTICAL_SOLVER_2R_H

#include "liepp/analytical/analytical_types.h"
#include "liepp/analytical/analytical_solver.h"

#include "liepp/analytical/detail/clamped_trig.h"
#include "liepp/analytical/detail/fk_verification.h"

#include "liepp/serial/chain/joint_tags.h"
#include "liepp/serial/chain/static_chain.h"

#include "liepp/lie/se3.h"

#include <cmath>
#include <cstddef>
#include <expected>
#include <numbers>

namespace liepp
{

template <typename Scalar, joint_tag... Joints>
class planar_2r_solver
{
    static_assert(sizeof...(Joints) == 2, "2R solver requires exactly 2 joints");
    static_assert((Joints::is_revolute && ...), "2R solver requires all revolute joints");

public:
    using chain_type = static_chain<Scalar, Joints...>;
    using scalar_type = Scalar;
    static constexpr int joints = 2;
    static constexpr int max_solutions = 2;

    explicit planar_2r_solver(const chain_type& chain)
        : m_chain(chain)
    {
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
    }

    [[nodiscard]] std::expected<
        analytical_result<Scalar, 2, 2>,
        analytical_error<Scalar>>
    solve(const se3<Scalar>& target) const
    {
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
            return std::unexpected(analytical_error<Scalar>{
                analytical_failure::unreachable, workspace_dist});
        }
        if (dist_sq < min_reach * min_reach - detail::sqrt_epsilon_v<Scalar>)
        {
            Scalar workspace_dist = min_reach - std::sqrt(dist_sq);
            return std::unexpected(analytical_error<Scalar>{
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

        // Solution 1 (elbow "up" / righty)
        result.solutions[0] = Eigen::Vector<Scalar, 2>(
            gamma - alpha,
            std::numbers::pi_v<Scalar> - beta);

        // Solution 2 (elbow "down" / lefty)
        result.solutions[1] = Eigen::Vector<Scalar, 2>(
            gamma + alpha,
            beta - std::numbers::pi_v<Scalar>);

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
            return std::unexpected(analytical_error<Scalar>{
                analytical_failure::verification_failed, Scalar(0)});
        }

        return verified;
    }

private:
    chain_type m_chain;
    Scalar m_link_length_1;
    Scalar m_link_length_2;
    vector3<Scalar> m_base_point;
    vector3<Scalar> m_plane_normal;
    vector3<Scalar> m_basis_u;
    vector3<Scalar> m_basis_v;
};

template <typename Scalar, joint_tag... Joints>
planar_2r_solver(const static_chain<Scalar, Joints...>&)
    -> planar_2r_solver<Scalar, Joints...>;

static_assert(analytical_solver<planar_2r_solver<double, revolute_z, revolute_z>>,
    "planar_2r_solver must satisfy analytical_solver concept");

template <typename Scalar, joint_tag... Joints>
[[nodiscard]] auto solve_2r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target)
{
    planar_2r_solver solver(chain);
    return solver.solve(target);
}

}

#endif
