#ifndef HPP_GUARD_CARTAN_ANALYTICAL_SOLVER_6R_H
#define HPP_GUARD_CARTAN_ANALYTICAL_SOLVER_6R_H

#include "cartan/analytical/analytical_types.h"
#include "cartan/analytical/analytical_solver.h"
#include "cartan/analytical/paden_kahan.h"
#include "cartan/analytical/detail/clamped_trig.h"
#include "cartan/analytical/detail/fk_verification.h"
#include "cartan/analytical/detail/wrist_center.h"

#include "cartan/serial/chain/static_chain.h"
#include "cartan/serial/chain/joint_tags.h"

#include "cartan/lie/se3.h"
#include "cartan/lie/so3.h"
#include "cartan/detail/epsilon.h"

#include <array>
#include <cmath>
#include <expected>
#include <numbers>
#include <tuple>

namespace cartan
{

/// Closed-form IK solver for 6R mechanisms with Pieper geometry
/// (last three revolute axes intersecting at a common wrist center).
///
/// Decomposes IK into:
///   1. Inverse position: find joints 1-3 from wrist center position
///      using SP3 + SP2/SP1 (up to 4 solutions).
///   2. Inverse orientation: for each position solution, find joints 4-6
///      from wrist rotation matrix via Euler angle extraction
///      (up to 2 per position solution = 8 total).
///
/// All solutions are FK-verified with both position and orientation checks.
///
/// Reference: Lynch & Park, Modern Robotics, Section 6.1.1.
///            Murray, Li and Sastry (1994), Section 3.3.
template <typename Scalar, joint_tag... Joints>
class pieper_6r_solver
{
    static_assert(sizeof...(Joints) == 6, "6R solver requires exactly 6 joints");
    static_assert((Joints::is_revolute && ...), "6R solver requires all revolute joints");

public:
    using chain_type = static_chain<Scalar, Joints...>;
    using scalar_type = Scalar;
    static constexpr int joints = 6;
    static constexpr int max_solutions = 8;

    explicit pieper_6r_solver(const chain_type& chain)
        : m_chain(chain)
    {
        for (std::size_t i = 0; i < 6; ++i)
        {
            const auto& s = chain.axis(static_cast<int>(i));
            m_omega[i] = s.omega();
            m_q[i] = s.omega().cross(s.v());
        }

        auto wrist = detail::find_wrist_intersection(
            chain.axis(3), chain.axis(4), chain.axis(5));

        if (!wrist)
        {
            m_valid = false;
            return;
        }
        m_wrist_center_home = *wrist;
        m_valid = true;

        // Tool offset: vector from wrist center to EE in home config,
        // expressed in the home rotation frame (body frame).
        vector3<Scalar> tool_offset_world =
            chain.home().translation() - m_wrist_center_home;
        m_tool_offset = chain.home().rotation().inverse().act(tool_offset_world);

        m_p_ee = chain.home().translation();
    }

    [[nodiscard]] std::expected<
        analytical_result<Scalar, 6, 8>,
        analytical_error<Scalar>>
    solve(const se3<Scalar>& target) const
    {
        if (!m_valid)
        {
            return std::unexpected(analytical_error<Scalar>{
                analytical_failure::degenerate_geometry, Scalar(0)});
        }

        // Step 1: Compute wrist center position from target pose
        vector3<Scalar> p_wrist = detail::compute_wrist_center(target, m_tool_offset);

        // Step 2: Inverse position -- find joints 1-3 from wrist center.
        // Same SP3+SP2 decomposition as the 3R solver.

        // Reference point: intersection of axes 1 and 2
        vector3<Scalar> r = find_axes_reference(
            m_omega[0], m_q[0], m_omega[1], m_q[1]);

        // SP3: find theta3 such that rotating wrist_center_home about axis 3
        // gives a point at distance ||p_wrist - r|| from r.
        Scalar delta = (p_wrist - r).norm();

        auto sp3_result = paden_kahan_3(
            m_omega[2], m_q[2], m_wrist_center_home, r, delta);

        if (!sp3_result)
        {
            return std::unexpected(analytical_error<Scalar>{
                sp3_result.error(),
                (p_wrist - m_p_ee).norm()});
        }

        analytical_result<Scalar, 6, 8> result;

        // For each theta3 candidate, find theta1/theta2 and then wrist angles
        for (int i = 0; i < sp3_result->count; ++i)
        {
            Scalar theta3 = sp3_result->solutions[static_cast<std::size_t>(i)];

            vector3<Scalar> p_prime = rotate_point_about_axis(
                m_omega[2], m_q[2], m_wrist_center_home, theta3);

            // SP2: find (theta1, theta2) via two successive rotations
            auto sp2_result = paden_kahan_2(
                m_omega[0], m_omega[1], r, p_prime, p_wrist);
            if (!sp2_result)
                continue;

            for (int j = 0; j < sp2_result->count; ++j)
            {
                auto [theta1, theta2] =
                    sp2_result->solutions[static_cast<std::size_t>(j)];

                // Step 3: Inverse orientation -- extract joints 4-6
                // Compute R_03: rotation due to joints 1-3
                auto R_03 = compute_rotation_123(theta1, theta2, theta3);
                // Desired wrist rotation: R_wrist = R_03^(-1) * R_target
                auto R_wrist = R_03.inverse() * target.rotation();

                // Extract Euler angles from R_wrist for joints 4-6
                auto euler_solutions = extract_wrist_angles(R_wrist);

                for (int k = 0; k < euler_solutions.count; ++k)
                {
                    auto [theta4, theta5, theta6] =
                        euler_solutions.solutions[static_cast<std::size_t>(k)];

                    Eigen::Vector<Scalar, 6> q_candidate;
                    q_candidate << theta1, theta2, theta3, theta4, theta5, theta6;

                    if (detail::verify_analytical_solution(
                            m_chain, q_candidate, target, true))
                    {
                        result.solutions[static_cast<std::size_t>(result.count)]
                            = q_candidate;
                        ++result.count;
                        if (result.count >= max_solutions)
                            return result;
                    }
                }
            }
        }

        if (result.count > 0)
            return result;

        return std::unexpected(analytical_error<Scalar>{
            analytical_failure::unreachable,
            (p_wrist - m_wrist_center_home).norm()});
    }

private:
    /// Rotate a point about a screw axis by theta (Rodrigues).
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

    /// Find the reference point for axes 1-2 (closest approach midpoint).
    [[nodiscard]] static vector3<Scalar> find_axes_reference(
        const vector3<Scalar>& omega1,
        const vector3<Scalar>& q1,
        const vector3<Scalar>& omega2,
        const vector3<Scalar>& q2)
    {
        vector3<Scalar> w = q1 - q2;
        Scalar a = omega1.dot(omega1);
        Scalar b = omega1.dot(omega2);
        Scalar c = omega2.dot(omega2);
        Scalar e = omega1.dot(w);
        Scalar f = omega2.dot(w);

        Scalar denom = a * c - b * b;
        if (std::abs(denom) < detail::epsilon_v<Scalar>)
            return q1;

        Scalar t = (b * f - c * e) / denom;
        Scalar s = (a * f - b * e) / denom;

        vector3<Scalar> p1 = q1 + t * omega1;
        vector3<Scalar> p2 = q2 + s * omega2;
        return (p1 + p2) / Scalar(2);
    }

    /// Compute the rotation from base to joint 3 frame:
    /// R_03 = exp(omega1*t1) * exp(omega2*t2) * exp(omega3*t3) * R_home_03
    ///
    /// For the rotation contribution of joints 1-3, we compute
    /// the cumulative rotation from the home configuration.
    [[nodiscard]] so3<Scalar> compute_rotation_123(
        Scalar theta1, Scalar theta2, Scalar theta3) const
    {
        // Rotation contribution of each joint
        auto R1 = so3<Scalar>::exp(m_omega[0] * theta1);
        auto R2 = so3<Scalar>::exp(m_omega[1] * theta2);
        auto R3 = so3<Scalar>::exp(m_omega[2] * theta3);

        // R_03 = R1 * R2 * R3 * R_home
        // But for the PoE formulation, the home rotation already encodes
        // the reference orientation. We need the rotation at the wrist:
        // T(q) = exp(S1*q1) * ... * exp(S6*q6) * M
        // The rotation at the wrist (after joints 1-3) in space frame is:
        // R_03 = R1 * R2 * R3
        // And the target rotation = R_03 * R_wrist_home * R_456
        // So R_wrist = (R_03 * R_wrist_home)^(-1) * R_target
        // = R_wrist_home^(-1) * R_03^(-1) * R_target

        // Actually in PoE formulation:
        // R_target = R1 * R2 * R3 * R4 * R5 * R6 * R_home
        // So R4 * R5 * R6 = (R1*R2*R3)^(-1) * R_target * R_home^(-1)
        // We return R1*R2*R3 and the caller handles it.
        return R1 * R2 * R3;
    }

    /// Euler angle result for the wrist: up to 2 solutions.
    struct euler_result
    {
        struct angles { Scalar theta4, theta5, theta6; };
        std::array<angles, 2> solutions;
        int count{0};
    };

    /// Extract wrist joint angles (theta4, theta5, theta6) from the wrist
    /// rotation matrix. Uses the joint tag types at positions 3,4,5 to
    /// determine the Euler angle convention.
    ///
    /// R_wrist = R_03^(-1) * R_target * R_home^(-1)
    /// = exp(omega4*t4) * exp(omega5*t5) * exp(omega6*t6)
    [[nodiscard]] euler_result extract_wrist_angles(
        const so3<Scalar>& R_wrist_input) const
    {
        // The actual wrist rotation to decompose:
        // R_target = R_03 * R4 * R5 * R6 * R_home
        // R4*R5*R6 = R_03^(-1) * R_target * R_home^(-1)
        // The caller passes R_03^(-1) * R_target, so we need to
        // post-multiply by R_home^(-1)
        auto R = R_wrist_input * m_chain.home().rotation().inverse();
        matrix3<Scalar> M = R.matrix();

        using joint_tuple = std::tuple<Joints...>;
        using J3 = std::tuple_element_t<3, joint_tuple>;
        using J4 = std::tuple_element_t<4, joint_tuple>;
        using J5 = std::tuple_element_t<5, joint_tuple>;

        return extract_euler_dispatch<J3, J4, J5>(M);
    }

    /// Dispatch to the correct Euler angle extraction based on wrist axis types.
    template <typename J3, typename J4, typename J5>
    [[nodiscard]] euler_result extract_euler_dispatch(
        const matrix3<Scalar>& R) const
    {
        // Determine the wrist axis pattern from the joint tags.
        // We use the actual screw axis omega directions stored at construction.
        // Common patterns: ZYZ, ZYX, etc.
        // For general 3-axis Euler angles, use the stored omega directions.
        return extract_euler_general(R);
    }

    /// General Euler angle extraction using the stored wrist axis directions.
    ///
    /// Decompose R = exp(omega4*t4) * exp(omega5*t5) * exp(omega6*t6)
    /// using the approach from Murray, Li & Sastry.
    ///
    /// For the middle axis (joint 5), we can extract theta5 from the
    /// known rotation structure. Then theta4 and theta6 follow from
    /// single-axis subproblems.
    [[nodiscard]] euler_result extract_euler_general(
        const matrix3<Scalar>& R) const
    {
        euler_result result;

        // Use Paden-Kahan subproblem approach for Euler angle extraction.
        // Pick a reference vector not aligned with any wrist axis.
        // Apply R to it, then decompose using subproblems.
        //
        // R * v = exp(w4*t4) * exp(w5*t5) * exp(w6*t6) * v
        //
        // We choose a reference point p and target p' = R * p, then:
        //   1. SP3 for theta5: ||exp(w5*t5)*exp(w6*t6)*p - q|| = ||R^T * exp(w4*t4)^(-1) * p' - q||
        //   ... This gets complicated. Instead, use the closed-form ZYZ/ZYX approach
        //   when axes match standard patterns, and a general subproblem approach otherwise.

        // Detect the axis pattern from stored omega directions
        const auto& w4 = m_omega[3];
        const auto& w5 = m_omega[4];
        const auto& w6 = m_omega[5];

        // Check for ZYZ pattern (omega4 || omega6, both perpendicular to omega5)
        Scalar w4_dot_w6 = std::abs(w4.dot(w6));
        Scalar w4_dot_w5 = std::abs(w4.dot(w5));

        bool is_symmetric = w4_dot_w6 > Scalar(1) - detail::sqrt_epsilon_v<Scalar>
            && w4_dot_w5 < detail::sqrt_epsilon_v<Scalar>;

        if (is_symmetric)
        {
            // Symmetric Euler angles (like ZYZ, XYX, etc.)
            // theta5 from the diagonal element along the common axis direction
            return extract_symmetric_euler(R, w4, w5);
        }

        // Asymmetric Euler angles (like ZYX, XYZ, etc.)
        return extract_asymmetric_euler(R, w4, w5, w6);
    }

    /// Symmetric Euler angle extraction (e.g., ZYZ, XYX).
    /// R = Rot(w4, t4) * Rot(w5, t5) * Rot(w6, t6) where w4 || w6.
    [[nodiscard]] euler_result extract_symmetric_euler(
        const matrix3<Scalar>& R,
        const vector3<Scalar>& w_outer,
        const vector3<Scalar>& w_middle) const
    {
        euler_result result;

        // Build a frame: w_outer is one axis, w_middle is perpendicular
        // Express R in this frame to get standard Euler angles.
        // For a ZYZ-like wrist: the outer axis is w_outer, middle is w_middle.
        //
        // theta5 = acos(w_outer^T * R * w_outer)
        // When sin(theta5) != 0:
        //   theta4 = atan2(w_middle^T * R * w_outer, -(w_outer x w_middle)^T * R * w_outer)
        //   theta6 = atan2(w_outer^T * R * w_middle, w_outer^T * R * (w_outer x w_middle))

        vector3<Scalar> w_cross = w_outer.cross(w_middle);

        Scalar cos_theta5 = w_outer.dot(R * w_outer);
        Scalar theta5 = detail::safe_acos(cos_theta5);

        Scalar sin_theta5 = std::sin(theta5);

        if (std::abs(sin_theta5) < detail::sqrt_epsilon_v<Scalar>)
        {
            // Gimbal lock: only theta4 + theta6 (or theta4 - theta6) is determined.
            // Set theta6 = 0 and solve for theta4.
            Scalar theta4_plus_theta6;

            if (cos_theta5 > Scalar(0))
            {
                // theta5 ~ 0: R ~ Rot(w_outer, theta4 + theta6)
                // Extract combined angle from rotation in the plane perpendicular to w_outer
                Scalar c = w_middle.dot(R * w_middle);
                Scalar s = w_cross.dot(R * w_middle);
                theta4_plus_theta6 = std::atan2(s, c);
            }
            else
            {
                // theta5 ~ pi: R ~ Rot(w_outer, theta4 - theta6) * Rot(w_middle, pi)
                // The rotation flips the middle axis direction.
                // R * w_middle should be close to Rot(w_outer, theta4-theta6) * (-w_middle)
                // for ZYZ with theta5=pi.
                // Let R2 = R * Rot(w_middle, pi)^(-1) = R * Rot(w_middle, -pi)
                // Then R2 ~ Rot(w_outer, theta4 - theta6)
                matrix3<Scalar> R_mid_pi = (Scalar(2) * w_middle * w_middle.transpose()
                    - matrix3<Scalar>::Identity());
                matrix3<Scalar> R2 = R * R_mid_pi;
                Scalar c = w_middle.dot(R2 * w_middle);
                Scalar s = w_cross.dot(R2 * w_middle);
                theta4_plus_theta6 = std::atan2(s, c);
            }

            result.solutions[0] = {theta4_plus_theta6, theta5, Scalar(0)};
            result.count = 1;
            return result;
        }

        // Non-degenerate case: two solutions from theta5 sign ambiguity
        for (int sign = 0; sign < 2; ++sign)
        {
            Scalar t5 = (sign == 0) ? theta5 : -theta5;
            Scalar st5 = std::sin(t5);

            Scalar t4 = std::atan2(
                w_middle.dot(R * w_outer) / st5,
                -w_cross.dot(R * w_outer) / st5);

            Scalar t6 = std::atan2(
                w_outer.dot(R * w_middle) / st5,
                w_outer.dot(R * w_cross) / st5);

            result.solutions[static_cast<std::size_t>(result.count)] =
                {t4, t5, t6};
            ++result.count;
        }

        return result;
    }

    /// Asymmetric Euler angle extraction (e.g., ZYX, XYZ).
    /// R = Rot(w4, t4) * Rot(w5, t5) * Rot(w6, t6) where w4, w5, w6 are distinct.
    [[nodiscard]] euler_result extract_asymmetric_euler(
        const matrix3<Scalar>& R,
        const vector3<Scalar>& w4,
        const vector3<Scalar>& w5,
        const vector3<Scalar>& w6) const
    {
        euler_result result;

        // For asymmetric Euler: sin(theta5) comes from a specific element.
        // General approach: w4^T * R * w6 = w4^T * Rot(w4,t4) * Rot(w5,t5) * Rot(w6,t6) * w6
        // Since Rot(w4,t4) * w4 = w4 and w6^T * Rot(w6,t6) = w6^T:
        //   w4^T * R * w6 = w4^T * Rot(w5, t5) * w6
        //
        // For perpendicular axes: w4^T * Rot(w5,t5) * w6 involves sin/cos of t5.
        // Let's use the Paden-Kahan SP1 approach for the general case.

        // Use subproblem decomposition:
        // Choose reference vector p (not aligned with any axis).
        // p' = R * p
        // exp(w4*t4) * exp(w5*t5) * exp(w6*t6) * p = p'
        //
        // SP3 on w5: rotate p'' about w5 to match a distance constraint.
        // This is the most robust general approach.

        vector3<Scalar> origin = vector3<Scalar>::Zero();

        // SP3: find theta5 such that
        // || exp(w5*t5) * exp(w6*t6) * p - q || = || exp(-w4*t4) * p' - q ||
        // Since we don't know t4 yet, use the distance trick:
        // || rotated_p - origin || is preserved by rotation about w4 through origin,
        // so || p' - origin || = || exp(w4*t4)^(-1) * p' - origin ||.
        // Then: || exp(w5*t5) * (exp(w6*t6) * p) - origin || = || p' ||
        // But that's just ||p|| = ||p'|| = 1 (if p is unit), which is always true.
        //
        // Better approach: use a point NOT on the wrist center.
        // Actually for pure rotations through origin, all points preserve distance.
        //
        // Use the direct approach: compute w4^T * R * w6 to get theta5 info.

        // Direct element extraction for general 3-axis rotation:
        // R = Rot(w4, t4) * Rot(w5, t5) * Rot(w6, t6)
        // Key relation: w4^T * R * w6 depends only on t5 (not t4 or t6)
        // because w4 is eigenvector of Rot(w4,t4) and w6 is eigenvector of Rot(w6,t6).
        //
        // w4^T * Rot(w5, t5) * w6 = cos(t5)*(w4.w6) + (1-cos(t5))*(w4.w5)*(w5.w6) + sin(t5)*(w4.(w5 x w6))

        Scalar a = w4.dot(w6);
        Scalar b = w4.dot(w5) * w5.dot(w6);
        Scalar c_coeff = w4.dot(w5.cross(w6));
        Scalar rhs = w4.dot(R * w6);

        // rhs = cos(t5)*a + (1-cos(t5))*b + sin(t5)*c_coeff
        // rhs = cos(t5)*(a - b) + b + sin(t5)*c_coeff
        // A*cos(t5) + C*sin(t5) = rhs - b
        Scalar A = a - b;
        Scalar C = c_coeff;
        Scalar D = rhs - b;

        Scalar amplitude = std::sqrt(A * A + C * C);
        if (amplitude < detail::sqrt_epsilon_v<Scalar>)
        {
            // Degenerate: axes are aligned in a way that prevents decomposition.
            // Try a single solution with theta5 = 0.
            result.solutions[0] = {Scalar(0), Scalar(0), Scalar(0)};
            result.count = 0; // Will rely on FK verification to filter
            return result;
        }

        Scalar ratio = D / amplitude;
        if (std::abs(ratio) > Scalar(1) + detail::sqrt_epsilon_v<Scalar>)
            return result; // No solution

        Scalar base_angle = std::atan2(C, A);
        Scalar offset = detail::safe_acos(ratio);

        // Two theta5 candidates
        std::array<Scalar, 2> theta5_candidates = {
            base_angle + offset,
            base_angle - offset
        };
        int num_t5 = (offset < detail::sqrt_epsilon_v<Scalar>) ? 1 : 2;

        for (int idx = 0; idx < num_t5; ++idx)
        {
            Scalar t5 = theta5_candidates[static_cast<std::size_t>(idx)];

            // With theta5 known, compute R5 = Rot(w5, t5)
            // R = Rot(w4, t4) * R5 * Rot(w6, t6)
            // Rot(w4, t4) = R * Rot(w6, t6)^(-1) * R5^(-1)
            // Let R_left = R * R5^(-1)... no, order matters.
            // R * Rot(w6, -t6) = Rot(w4, t4) * R5
            // R5^(-1) * Rot(w4, -t4) * R = Rot(w6, t6)
            //
            // Use SP1 to find t4 and t6:
            // Rot(w4, t4) * (R5 * w6) = R * w6  (since Rot(w6,t6)*w6 = w6)
            // -> SP1 for t4
            // Rot(w6, t6) * w4 = R5^(-1) * Rot(w4, -t4) * R * w4... complex.
            //
            // Simpler: after finding t5,
            // R4 * R5 * R6 = R
            // R4 = R * R6^(-1) * R5^(-1)
            // Since R6 * w6 = w6: R * w6 = R4 * R5 * w6
            // SP1: find t4 such that Rot(w4, t4) rotates (R5*w6) to (R*w6)

            auto R5 = so3<Scalar>::exp(w5 * t5);
            vector3<Scalar> R5_w6 = R5.act(w6);
            vector3<Scalar> R_w6 = R * w6;

            auto t4_result = paden_kahan_1(w4, origin, R5_w6, R_w6);
            if (!t4_result)
                continue;
            Scalar t4 = *t4_result;

            // Now find t6:
            // R4^(-1) * R * R6 = R5 (where R6 rotates about w6)
            // R6 = R5^(-1) * R4^(-1) * R
            // Since R4^(-1) * w4 = w4: w4^T * R = w4^T * R4 * R5 * R6
            // w4 stays fixed under R4, so: R4^T * R = R5 * R6
            // SP1: Rot(w6, t6) rotates some reference to match
            // R6 * (R5^(-1) * w4) = (R4^(-1) * R)^(-1)... getting complicated.
            //
            // Direct: R6 = R5^(-1) * R4^(-1) * R
            // t6 via SP1: Rot(w6, t6) * (some_vec) = R6 * (some_vec)
            // where some_vec is not aligned with w6.

            auto R4 = so3<Scalar>::exp(w4 * t4);
            auto R6_computed = R5.inverse() * (R4.inverse() * so3<Scalar>(Eigen::Quaternion<Scalar>(R)));

            // Extract t6 from R6_computed using SP1 with a reference vector
            // perpendicular to w6
            vector3<Scalar> ref = w5; // w5 is not parallel to w6 (asymmetric case)
            if (std::abs(ref.dot(w6)) > Scalar(1) - detail::sqrt_epsilon_v<Scalar>)
            {
                ref = w4;
            }
            vector3<Scalar> ref_rotated = R6_computed.act(ref);

            auto t6_result = paden_kahan_1(w6, origin, ref, ref_rotated);
            if (!t6_result)
                continue;
            Scalar t6 = *t6_result;

            result.solutions[static_cast<std::size_t>(result.count)] =
                {t4, t5, t6};
            ++result.count;
        }

        return result;
    }

    chain_type m_chain;
    std::array<vector3<Scalar>, 6> m_omega;
    std::array<vector3<Scalar>, 6> m_q;
    vector3<Scalar> m_wrist_center_home;
    vector3<Scalar> m_tool_offset;
    vector3<Scalar> m_p_ee;
    bool m_valid{false};
};

template <typename Scalar, joint_tag... Joints>
pieper_6r_solver(const static_chain<Scalar, Joints...>&)
    -> pieper_6r_solver<Scalar, Joints...>;

static_assert(analytical_solver<pieper_6r_solver<double,
    revolute_z, revolute_y, revolute_y, revolute_z, revolute_y, revolute_z>>,
    "pieper_6r_solver must satisfy analytical_solver concept");

template <typename Scalar, joint_tag... Joints>
[[nodiscard]] auto solve_6r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target)
{
    pieper_6r_solver solver(chain);
    return solver.solve(target);
}

}

#endif
