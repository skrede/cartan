#ifndef HPP_GUARD_CARTAN_ANALYTICAL_SOLVER_3R_H
#define HPP_GUARD_CARTAN_ANALYTICAL_SOLVER_3R_H

#include "cartan/analytical/analytical_types.h"
#include "cartan/analytical/analytical_solver.h"
#include "cartan/analytical/paden_kahan.h"
#include "cartan/analytical/detail/fk_verification.h"

#include "cartan/serial/chain/static_chain.h"
#include "cartan/serial/chain/joint_tags.h"

#include "cartan/lie/se3.h"
#include "cartan/detail/epsilon.h"

#include <cmath>
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
template <typename Scalar, joint_tag... Joints>
class spatial_3r_solver
{
    static_assert(sizeof...(Joints) == 3, "3R solver requires exactly 3 joints");
    static_assert((Joints::is_revolute && ...), "3R solver requires all revolute joints");

public:
    using chain_type = static_chain<Scalar, Joints...>;
    using scalar_type = Scalar;
    static constexpr int joints = 3;
    static constexpr int max_solutions = 4;

    explicit spatial_3r_solver(const chain_type& chain)
        : m_chain(chain)
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            const auto& s = chain.axis(static_cast<int>(i));
            m_omega[i] = s.omega();
            m_q[i] = s.omega().cross(s.v());
        }
        m_p_ee = chain.home().translation();
    }

    [[nodiscard]] cartan::expected<
        analytical_result<Scalar, 3, 4>,
        analytical_error<Scalar>>
    solve(const se3<Scalar>& target) const
    {
        vector3<Scalar> p_target = target.translation();

        // Axes 1 and 2 intersect at m_q[0] (for chains where q0 = q1, the
        // typical case). Use the closest approach midpoint as the reference
        // point r for the SP3/SP2 decomposition.
        vector3<Scalar> r = find_axes_intersection(
            m_omega[0], m_q[0], m_omega[1], m_q[1]);

        // SP3: find theta3 such that ||rot(w3,q3,t3)*p_ee - r|| = ||p_d - r||.
        // Rotation about axes 1-2 through r preserves distance from r, so the
        // distance constraint decouples theta3 from theta1, theta2.
        Scalar delta = (p_target - r).norm();

        auto sp3_result = paden_kahan_3(m_omega[2], m_q[2], m_p_ee, r, delta);
        if (!sp3_result)
        {
            return cartan::unexpected(analytical_error<Scalar>{
                sp3_result.error(),
                (p_target - m_p_ee).norm()});
        }

        analytical_result<Scalar, 3, 4> result;

        for (int i = 0; i < sp3_result->count; ++i)
        {
            Scalar theta3 = sp3_result->solutions[static_cast<std::size_t>(i)];

            vector3<Scalar> p_prime = rotate_point_about_axis(
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

                if (detail::verify_analytical_solution(m_chain, q_candidate, target, false))
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
            analytical_failure::verification_failed,
            (p_target - m_p_ee).norm()});
    }

private:
    [[nodiscard]] static vector3<Scalar> rotate_point_about_axis(
        const vector3<Scalar>& omega,
        const vector3<Scalar>& q,
        const vector3<Scalar>& p,
        Scalar theta)
    {
        vector3<Scalar> v = p - q;
        Scalar ct = std::cos(theta);
        Scalar st = std::sin(theta);
        return q + ct * v
            + (Scalar(1) - ct) * omega.dot(v) * omega
            + st * omega.cross(v);
    }

    /// Find the intersection point of two lines (or closest approach midpoint).
    /// Line i: point q_i + t * omega_i.
    [[nodiscard]] static vector3<Scalar> find_axes_intersection(
        const vector3<Scalar>& omega1,
        const vector3<Scalar>& q1,
        const vector3<Scalar>& omega2,
        const vector3<Scalar>& q2)
    {
        vector3<Scalar> d = q2 - q1;
        Scalar a = omega1.dot(omega1);
        Scalar b = omega1.dot(omega2);
        Scalar c = omega2.dot(omega2);
        Scalar e = omega1.dot(d);
        Scalar f = omega2.dot(d);

        Scalar denom = a * c - b * b;
        if (std::abs(denom) < detail::epsilon_v<Scalar>)
            return q1;

        Scalar t = (b * f - c * e) / denom;
        Scalar s = (a * f - b * e) / denom;

        vector3<Scalar> p1 = q1 + t * omega1;
        vector3<Scalar> p2 = q2 + s * omega2;
        return (p1 + p2) / Scalar(2);
    }

    chain_type m_chain;
    std::array<vector3<Scalar>, 3> m_omega;
    std::array<vector3<Scalar>, 3> m_q;
    vector3<Scalar> m_p_ee;
};

template <typename Scalar, joint_tag... Joints>
spatial_3r_solver(const static_chain<Scalar, Joints...>&) -> spatial_3r_solver<Scalar, Joints...>;

static_assert(analytical_solver<spatial_3r_solver<double, revolute_z, revolute_y, revolute_z>>);

template <typename Scalar, joint_tag... Joints>
[[nodiscard]] auto solve_3r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target)
{
    return spatial_3r_solver(chain).solve(target);
}

}

#endif
