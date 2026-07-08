#ifndef HPP_GUARD_CARTAN_ANALYTICAL_SOLVER_OPW_H
#define HPP_GUARD_CARTAN_ANALYTICAL_SOLVER_OPW_H

#include "cartan/analytical/analytical_types.h"
#include "cartan/analytical/analytical_solver.h"
#include "cartan/analytical/detail/clamped_trig.h"
#include "cartan/analytical/detail/fk_verification.h"
#include "cartan/analytical/detail/wrist_center.h"

#include "cartan/serial/chain/joint_tags.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/chain/static_chain.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include "cartan/lie/se3.h"
#include "cartan/lie/so3.h"
#include "cartan/detail/epsilon.h"

#include <array>
#include <cmath>
#include <limits>
#include "cartan/expected.h"
#include <numbers>
#include <algorithm>
#include <type_traits>

namespace cartan
{

/// Geometric parameters of an ortho-parallel-basis, spherical-wrist 6R arm,
/// following Brandstotter/Angerer/Hofbaur (2014). The seven a/b/c lengths and
/// the per-joint offset/sign convention mirror the community-standard
/// reference implementation verbatim so a hand-derived parameter set is
/// numerically interchangeable with it, with no unit or frame conversion.
///
///   a1  lateral offset of the shoulder from the base rotation axis
///   a2  elbow offset (upper-arm to forearm)
///   b   out-of-plane offset of the arm from the base plane
///   c1  base height (axis 1 to axis 2)
///   c2  upper-arm length (axis 2 to axis 3)
///   c3  forearm length (axis 3 to the wrist center)
///   c4  wrist length (wrist center to the flange along the approach axis)
///
/// The forward map applies the per-joint convention
/// `theta_internal[i] = q[i] * sign_corrections[i] - offsets[i]`; the inverse
/// map applies the exact reverse `q[i] = (theta_internal[i] + offsets[i]) *
/// sign_corrections[i]`. `sign_corrections` entries are constrained to
/// {-1, +1}. This is a plain aggregate.
template <typename Scalar>
struct opw_parameters
{
    Scalar a1;
    Scalar a2;
    Scalar b;
    Scalar c1;
    Scalar c2;
    Scalar c3;
    Scalar c4;
    std::array<Scalar, 6> offsets;
    std::array<signed char, 6> sign_corrections;
};

/// The eight distinct configuration branches an OPW arm can adopt to reach a
/// given pose, encoded as three independent binary choices.
///
/// Canonical bit assignment (MSB to LSB):
///   bit 2 (value 4): shoulder  -- front (0) vs back  (1)
///   bit 1 (value 2): elbow     -- up    (0) vs down  (1)
///   bit 0 (value 1): wrist     -- no-flip (0) vs flip (1)
///
/// The integer key of a branch is therefore
/// `shoulder * 4 + elbow * 2 + wrist`, and solve() emits solutions in
/// ascending key order. This ordering is the stable contract the deferred
/// cross-library branch-column comparison permutes against; it must not be
/// changed without updating that permutation.
enum class opw_branch : unsigned char
{
    front_up_no_flip   = 0,
    front_up_flip      = 1,
    front_down_no_flip = 2,
    front_down_flip    = 3,
    back_up_no_flip    = 4,
    back_up_flip       = 5,
    back_down_no_flip  = 6,
    back_down_flip     = 7
};

/// True when the branch places the shoulder in its back configuration (bit 2).
constexpr bool is_shoulder_back(opw_branch branch)
{
    return (static_cast<unsigned>(branch) & 0x4u) != 0u;
}

/// True when the branch places the elbow in its down configuration (bit 1).
constexpr bool is_elbow_down(opw_branch branch)
{
    return (static_cast<unsigned>(branch) & 0x2u) != 0u;
}

/// True when the branch flips the wrist (bit 0).
constexpr bool is_wrist_flip(opw_branch branch)
{
    return (static_cast<unsigned>(branch) & 0x1u) != 0u;
}

/// Forward kinematics of an OPW arm expressed directly through its
/// `opw_parameters`, independent of any screw model. Reproduces the reference
/// OPW forward map, applying the input convention
/// `theta_internal[i] = q[i] * sign_corrections[i] - offsets[i]`.
///
/// This is the always-available reconstruction gate: comparing it against the
/// chain's own forward kinematics at every configuration certifies that a
/// hand-derived parameter set and the screw model describe the same robot.
template <typename Scalar>
se3<Scalar> opw_forward(
    const opw_parameters<Scalar>& params,
    const Eigen::Vector<Scalar, 6>& q)
{
    std::array<Scalar, 6> theta;
    for (int i = 0; i < 6; ++i)
    {
        theta[static_cast<std::size_t>(i)] =
            q(i) * static_cast<Scalar>(params.sign_corrections[static_cast<std::size_t>(i)])
            - params.offsets[static_cast<std::size_t>(i)];
    }

    const Scalar psi3 = std::atan2(params.a2, params.c3);
    const Scalar k = std::sqrt(params.a2 * params.a2 + params.c3 * params.c3);

    const Scalar cx1 = params.c2 * std::sin(theta[1])
        + k * std::sin(theta[1] + theta[2] + psi3) + params.a1;
    const Scalar cy1 = params.b;
    const Scalar cz1 = params.c2 * std::cos(theta[1])
        + k * std::cos(theta[1] + theta[2] + psi3);

    const Scalar cx0 = cx1 * std::cos(theta[0]) - cy1 * std::sin(theta[0]);
    const Scalar cy0 = cx1 * std::sin(theta[0]) + cy1 * std::cos(theta[0]);
    const Scalar cz0 = cz1 + params.c1;

    const Scalar s1 = std::sin(theta[0]);
    const Scalar s2 = std::sin(theta[1]);
    const Scalar s3 = std::sin(theta[2]);
    const Scalar s4 = std::sin(theta[3]);
    const Scalar s5 = std::sin(theta[4]);
    const Scalar s6 = std::sin(theta[5]);

    const Scalar c1 = std::cos(theta[0]);
    const Scalar c2 = std::cos(theta[1]);
    const Scalar c3 = std::cos(theta[2]);
    const Scalar c4 = std::cos(theta[3]);
    const Scalar c5 = std::cos(theta[4]);
    const Scalar c6 = std::cos(theta[5]);

    matrix3<Scalar> r_0c;
    r_0c(0, 0) = c1 * c2 * c3 - c1 * s2 * s3;
    r_0c(0, 1) = -s1;
    r_0c(0, 2) = c1 * c2 * s3 + c1 * s2 * c3;
    r_0c(1, 0) = s1 * c2 * c3 - s1 * s2 * s3;
    r_0c(1, 1) = c1;
    r_0c(1, 2) = s1 * c2 * s3 + s1 * s2 * c3;
    r_0c(2, 0) = -s2 * c3 - c2 * s3;
    r_0c(2, 1) = Scalar(0);
    r_0c(2, 2) = -s2 * s3 + c2 * c3;

    matrix3<Scalar> r_ce;
    r_ce(0, 0) = c4 * c5 * c6 - s4 * s6;
    r_ce(0, 1) = -c4 * c5 * s6 - s4 * c6;
    r_ce(0, 2) = c4 * s5;
    r_ce(1, 0) = s4 * c5 * c6 + c4 * s6;
    r_ce(1, 1) = -s4 * c5 * s6 + c4 * c6;
    r_ce(1, 2) = s4 * s5;
    r_ce(2, 0) = -s5 * c6;
    r_ce(2, 1) = s5 * s6;
    r_ce(2, 2) = c5;

    matrix3<Scalar> r_oe = r_0c * r_ce;

    vector3<Scalar> u = vector3<Scalar>(cx0, cy0, cz0)
        + params.c4 * r_oe * vector3<Scalar>::UnitZ();

    return se3<Scalar>(so3<Scalar>(quaternion<Scalar>(r_oe)), u);
}

/// Recover the configuration branch of a solved joint vector, purely from the
/// joint values (the derived-identity contract: solutions carry no stored tag).
///
/// Each bit is read from a wrap-invariant geometric predicate on the internal
/// joint angles `theta_internal[i] = q[i] * sign_corrections[i] - offsets[i]`:
///   shoulder: the sign of the arm-plane horizontal wrist-center coordinate
///             `cx1 = c2*sin(theta2) + k*sin(theta2+theta3+psi3) + a1`; a
///             back-reaching arm projects this coordinate negative.
///   elbow:    `sin(theta3 + atan2(a2, c3))` -- non-negative for elbow-up
///             (the +acos branch), negative for elbow-down (the -acos branch).
///   wrist:    `sin(theta5)` -- non-negative for the no-flip branch
///             (theta5 in [0, pi]), negative for the flipped branch (-theta5).
template <typename Scalar>
opw_branch classify_branch(
    const Eigen::Vector<Scalar, 6>& q,
    const opw_parameters<Scalar>& params)
{
    std::array<Scalar, 6> theta;
    for (int i = 0; i < 6; ++i)
    {
        theta[static_cast<std::size_t>(i)] =
            q(i) * static_cast<Scalar>(params.sign_corrections[static_cast<std::size_t>(i)])
            - params.offsets[static_cast<std::size_t>(i)];
    }

    const Scalar psi3 = std::atan2(params.a2, params.c3);
    const Scalar k = std::sqrt(params.a2 * params.a2 + params.c3 * params.c3);

    const Scalar cx1 = params.c2 * std::sin(theta[1])
        + k * std::sin(theta[1] + theta[2] + psi3) + params.a1;

    const bool shoulder_back = cx1 < Scalar(0);
    const bool elbow_down = std::sin(theta[2] + psi3) < Scalar(0);
    const bool wrist_flip = std::sin(theta[4]) < Scalar(0);

    const unsigned key = (shoulder_back ? 0x4u : 0x0u)
        | (elbow_down ? 0x2u : 0x0u)
        | (wrist_flip ? 0x1u : 0x0u);
    return static_cast<opw_branch>(key);
}

/// Closed-form IK solver for ortho-parallel-basis, spherical-wrist 6R arms
/// (the industrial offset-shoulder class: KUKA, ABB, Fanuc, Motoman, Staubli).
///
/// This is the dual of pieper_6r_solver: where Pieper requires the shoulder
/// axes to intersect (a1 == 0), OPW accepts the lateral shoulder offset that
/// Pieper rejects, and instead requires the ortho-parallel basis (axis 1
/// perpendicular to axis 2, axis 2 parallel to axis 3) plus a spherical wrist.
///
/// The closed form is a faithful re-expression of the Brandstotter analytical
/// inverse: wrist center, then theta1 (two shoulder branches), theta2/theta3
/// (two elbow branches each via the law of cosines), and theta4/theta5/theta6
/// (two wrist branches via the R0c*Rce decomposition). Every one of the up to
/// eight candidates is FK-verified against the chain before it is reported;
/// candidates are emitted in the canonical branch-key order (see opw_branch).
///
/// Reference: Brandstotter, Angerer, Hofbaur (2014), "An Analytical Solution
/// of the Inverse Kinematics Problem of Industrial Serial Manipulators with an
/// Ortho-parallel Basis and a Spherical Wrist."

/// Verification policy for opw_6r_solver (a compile-time tag, no runtime cost).
///
/// opw_verified (the default) FK-verifies every candidate branch against the
/// chain and collapses coincident configurations, so solve() emits only genuine,
/// distinct solutions and reports unreachable/singular honestly -- the
/// load-bearing "verified ranked branches" contract that distinguishes cartan
/// from an unverified enumerator.
struct opw_verified
{
};

/// opw_raw skips the FK back-check and the duplicate collapse: it emits every
/// finite branch in canonical key order (up to eight, no dedup). It is faster,
/// but a branch that does not reach the target -- near a singularity or for an
/// unreachable pose -- is emitted unfiltered, exactly like an unverified
/// enumerator. Use only when reachability is guaranteed by the caller or checked
/// externally; opw_verified remains the default for every general-purpose use.
struct opw_raw
{
};

template <chain Chain, typename Verification = opw_verified>
class opw_6r_solver
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = 6;
    static constexpr int max_solutions = 8;

    /// Acceptance tolerance for the FK position/orientation back-check, matching
    /// detail::verify_analytical_solution. The construction-time spherical-wrist
    /// gate is anchored to the same value so a constructed solver is always
    /// solvable to the tolerance it verifies against.
    static constexpr scalar_type default_position_tolerance = scalar_type(1e-6);

    /// Threshold on |sin(theta5)| below which the wrist is treated as singular
    /// and the fold path (pin theta4 = 0, recover theta6 by projection) is
    /// taken. Pinned empirically, not copied from the reference implementation's
    /// hardcoded 1e-6. The near-singular wrist is a competition between two
    /// finite-precision failure modes: the naive theta4/theta6 atan2 decomposition
    /// amplifies rounding as ~eps/|sin(theta5)| (its worst-case FK error grows as
    /// the locus is approached), while pinning theta4 = 0 injects an O(|sin(theta5)|)
    /// residual because that pin is exact only at |sin(theta5)| = 0. A worst-case
    /// FK-error sweep of the two paths against a fixed spherical-wrist arm, over
    /// many shoulder/elbow configurations, locates their crossover in the band
    /// |sin(theta5)| ~= 2.2e-8 .. 3.1e-8: above it the naive decomposition wins,
    /// below it the fold wins. This default sits inside that measured band. The
    /// reference's 1e-6 folds a whole decade where the naive path is in fact more
    /// accurate, driving near-singular solutions to the solve tolerance edge; the
    /// swept value instead caps worst-case near-singular FK error two orders of
    /// magnitude lower.
    static constexpr scalar_type default_singularity_tolerance = scalar_type(2.5e-8);

    /// Construction-time geometry validation. Returns a ready solver only when
    /// the chain has OPW geometry; otherwise fails loudly with
    /// `degenerate_geometry` rather than deferring to a misleading per-pose
    /// `unreachable`. Validated:
    ///   1. exactly 6 revolute joints;
    ///   2. ortho basis: axis 1 is perpendicular to axis 2
    ///      (|omega0 . omega1| < sqrt_epsilon);
    ///   3. parallel basis: axis 2 is parallel to axis 3
    ///      (|omega1 . omega2| > 1 - sqrt_epsilon);
    ///   4. spherical wrist: axes 4, 5, 6 meet at a common center within the
    ///      acceptance tolerance (detail::find_wrist_intersection).
    ///
    /// The lateral shoulder offset (a1 != 0) is accepted -- this is precisely
    /// the geometry Pieper's shoulder-intersection gate rejects. Any
    /// ortho-parallel spherical-wrist arm is admitted, including the a1 == 0
    /// case that is also solvable by Pieper; selecting between solvers by
    /// geometry is left to a future dispatch layer above the solvers.
    static cartan::expected<opw_6r_solver, analytical_error<scalar_type>>
    make(const Chain& chain,
         const opw_parameters<scalar_type>& params,
         scalar_type position_tolerance = default_position_tolerance,
         scalar_type singularity_tolerance = default_singularity_tolerance)
    {
        if (chain.num_joints() != 6)
        {
            return cartan::unexpected(analytical_error<scalar_type>{
                analytical_failure::degenerate_geometry, scalar_type(0)});
        }
        for (int i = 0; i < 6; ++i)
        {
            if (!chain.axis(i).is_revolute())
            {
                return cartan::unexpected(analytical_error<scalar_type>{
                    analytical_failure::degenerate_geometry, scalar_type(0)});
            }
        }

        const vector3<scalar_type> w0 = chain.axis(0).omega();
        const vector3<scalar_type> w1 = chain.axis(1).omega();
        const vector3<scalar_type> w2 = chain.axis(2).omega();

        // Ortho basis: axis 1 perpendicular to axis 2.
        const scalar_type ortho = std::abs(w0.dot(w1));
        if (ortho >= detail::sqrt_epsilon_v<scalar_type>)
        {
            return cartan::unexpected(analytical_error<scalar_type>{
                analytical_failure::degenerate_geometry, ortho});
        }

        // Parallel basis: axis 2 parallel to axis 3.
        const scalar_type parallel = std::abs(w1.dot(w2));
        if (parallel <= scalar_type(1) - detail::sqrt_epsilon_v<scalar_type>)
        {
            return cartan::unexpected(analytical_error<scalar_type>{
                analytical_failure::degenerate_geometry,
                scalar_type(1) - parallel});
        }

        // Spherical wrist at the acceptance tolerance.
        auto wrist = detail::find_wrist_intersection(
            chain.axis(3), chain.axis(4), chain.axis(5), position_tolerance);
        if (!wrist)
        {
            return cartan::unexpected(analytical_error<scalar_type>{
                analytical_failure::degenerate_geometry, scalar_type(0)});
        }

        return opw_6r_solver(
            chain, params, position_tolerance, singularity_tolerance);
    }

    cartan::expected<
        analytical_result<scalar_type, 6, 8>,
        analytical_error<scalar_type>>
    solve(const se3<scalar_type>& target) const
    {
        if (!m_valid)
        {
            return cartan::unexpected(analytical_error<scalar_type>{
                analytical_failure::degenerate_geometry, scalar_type(0)});
        }

        const opw_parameters<Scalar>& p = m_params;
        const matrix3<Scalar> matrix = target.rotation().matrix();
        const vector3<Scalar> translation = target.translation();

        const Scalar pi = std::numbers::pi_v<Scalar>;
        const Scalar sqrt_eps = detail::sqrt_epsilon_v<Scalar>;

        // Step 1: wrist center = flange - c4 * approach axis.
        const vector3<Scalar> center = translation - p.c4 * matrix.col(2);

        // Domain and reach diagnostics: only genuine out-of-workspace targets
        // set domain_failed; near-singular loci are left to the FK back-check.
        bool domain_failed = false;
        Scalar workspace_distance = Scalar(0);

        // Step 2: theta1 -- guard the lateral-offset cylinder the wrist center
        // must lie outside of (c.x^2 + c.y^2 >= b^2).
        const Scalar nx1_arg =
            center.x() * center.x() + center.y() * center.y() - p.b * p.b;
        if (nx1_arg < -sqrt_eps)
        {
            domain_failed = true;
            workspace_distance = std::sqrt(-nx1_arg);
        }
        const Scalar nx1 = std::sqrt(std::clamp(
            nx1_arg, Scalar(0), std::numeric_limits<Scalar>::infinity()))
            - p.a1;

        const Scalar tmp1 = std::atan2(center.y(), center.x());
        const Scalar tmp2 = std::atan2(p.b, nx1 + p.a1);
        const Scalar theta1_i = tmp1 - tmp2;
        const Scalar theta1_ii = tmp1 + tmp2 - pi;

        // Step 3: theta2 / theta3 -- law of cosines over the shoulder-wrist
        // triangle, two elbow branches for each theta1 branch.
        const Scalar tmp3 = center.z() - p.c1;
        const Scalar s1_2 = nx1 * nx1 + tmp3 * tmp3;
        const Scalar tmp4 = nx1 + Scalar(2) * p.a1;
        const Scalar s2_2 = tmp4 * tmp4 + tmp3 * tmp3;
        const Scalar kappa_2 = p.a2 * p.a2 + p.c3 * p.c3;
        const Scalar c2_2 = p.c2 * p.c2;

        const Scalar s1 = std::sqrt(std::clamp(
            s1_2, Scalar(0), std::numeric_limits<Scalar>::infinity()));
        const Scalar s2 = std::sqrt(std::clamp(
            s2_2, Scalar(0), std::numeric_limits<Scalar>::infinity()));
        const Scalar k_reach = std::sqrt(kappa_2);

        // Triangle-inequality reach check (diagnostic; routes to unreachable).
        const Scalar reach_max = p.c2 + k_reach;
        const Scalar reach_min = std::abs(p.c2 - k_reach);
        const Scalar reach_deficit = std::max(
            {Scalar(0), s1 - reach_max, reach_min - s1});
        if (reach_deficit > sqrt_eps)
        {
            domain_failed = true;
            workspace_distance = std::max(workspace_distance, reach_deficit);
        }

        // acos with domain clamping; a genuine (beyond-rounding) out-of-range
        // ratio marks the target unreachable, while a vanishing denominator is
        // a singular locus (not a reach failure).
        auto acos_ratio = [&](Scalar num, Scalar den) -> Scalar
        {
            if (std::abs(den) < sqrt_eps)
                return detail::safe_acos(Scalar(0));
            const Scalar ratio = num / den;
            if (std::abs(ratio) > Scalar(1) + sqrt_eps)
                domain_failed = true;
            return detail::safe_acos(ratio);
        };

        const Scalar tmp5 = s1_2 + c2_2 - kappa_2;
        const Scalar tmp13 = acos_ratio(tmp5, Scalar(2) * s1 * p.c2);
        const Scalar tmp14 = std::atan2(nx1, tmp3);
        const Scalar theta2_i = -tmp13 + tmp14;
        const Scalar theta2_ii = tmp13 + tmp14;

        const Scalar tmp6 = s2_2 + c2_2 - kappa_2;
        const Scalar tmp15 = acos_ratio(tmp6, Scalar(2) * s2 * p.c2);
        const Scalar tmp16 = std::atan2(nx1 + Scalar(2) * p.a1, tmp3);
        const Scalar theta2_iii = -tmp15 - tmp16;
        const Scalar theta2_iv = tmp15 - tmp16;

        const Scalar tmp7 = s1_2 - c2_2 - kappa_2;
        const Scalar tmp8 = s2_2 - c2_2 - kappa_2;
        const Scalar tmp9 = Scalar(2) * p.c2 * k_reach;
        const Scalar tmp10 = std::atan2(p.a2, p.c3);
        const Scalar tmp11 = acos_ratio(tmp7, tmp9);
        const Scalar theta3_i = tmp11 - tmp10;
        const Scalar theta3_ii = -tmp11 - tmp10;
        const Scalar tmp12 = acos_ratio(tmp8, tmp9);
        const Scalar theta3_iii = tmp12 - tmp10;
        const Scalar theta3_iv = -tmp12 - tmp10;

        // Per (theta1, theta2, theta3) branch j = shoulder * 2 + elbow:
        //   j = 0 front/up, 1 front/down, 2 back/up, 3 back/down.
        const std::array<Scalar, 4> theta1_j{
            theta1_i, theta1_i, theta1_ii, theta1_ii};
        const std::array<Scalar, 4> theta2_j{
            theta2_i, theta2_ii, theta2_iii, theta2_iv};
        const std::array<Scalar, 4> theta3_j{
            theta3_i, theta3_ii, theta3_iii, theta3_iv};
        const std::array<Scalar, 4> sin1{
            std::sin(theta1_i), std::sin(theta1_i),
            std::sin(theta1_ii), std::sin(theta1_ii)};
        const std::array<Scalar, 4> cos1{
            std::cos(theta1_i), std::cos(theta1_i),
            std::cos(theta1_ii), std::cos(theta1_ii)};

        std::array<Scalar, 4> s23{};
        std::array<Scalar, 4> c23{};
        std::array<Scalar, 4> theta4_base{};
        std::array<Scalar, 4> theta5_base{};
        std::array<Scalar, 4> theta6_base{};

        // Step 4: theta4 / theta5 / theta6 -- decompose the residual wrist
        // rotation R0c * Rce. theta5 comes from the approach-axis projection m;
        // near |sin(theta5)| = 0 the wrist is singular, so pin theta4 = 0 and
        // fold the residual onto theta6 rather than dividing through zero.
        for (int j = 0; j < 4; ++j)
        {
            const std::size_t u = static_cast<std::size_t>(j);
            s23[u] = std::sin(theta2_j[u] + theta3_j[u]);
            c23[u] = std::cos(theta2_j[u] + theta3_j[u]);

            const Scalar m = matrix(0, 2) * s23[u] * cos1[u]
                + matrix(1, 2) * s23[u] * sin1[u]
                + matrix(2, 2) * c23[u];
            const Scalar sin5 = std::sqrt(
                std::clamp(Scalar(1) - m * m, Scalar(0), Scalar(1)));
            theta5_base[u] = std::atan2(sin5, m);

            if (sin5 < m_singularity_tolerance)
            {
                // Wrist singularity: joint 5 straight. theta4 and theta6 are
                // individually indeterminate; pin theta4 = 0 and recover
                // theta6 by projecting the flange x-axis onto the joint-5 frame
                // (whose z-axis coincides with the approach axis here).
                theta4_base[u] = Scalar(0);
                const vector3<Scalar> xe(
                    matrix(0, 0), matrix(1, 0), matrix(2, 0));
                matrix3<Scalar> r_c;
                r_c.col(1) = vector3<Scalar>(-sin1[u], cos1[u], Scalar(0));
                r_c.col(2) = matrix.col(2);
                r_c.col(0) = r_c.col(1).cross(r_c.col(2));
                const vector3<Scalar> xec = r_c.transpose() * xe;
                theta6_base[u] = std::atan2(xec(1), xec(0));
            }
            else
            {
                const Scalar t4y =
                    matrix(1, 2) * cos1[u] - matrix(0, 2) * sin1[u];
                const Scalar t4x = matrix(0, 2) * c23[u] * cos1[u]
                    + matrix(1, 2) * c23[u] * sin1[u]
                    - matrix(2, 2) * s23[u];
                theta4_base[u] = std::atan2(t4y, t4x);

                const Scalar t6y = matrix(0, 1) * s23[u] * cos1[u]
                    + matrix(1, 1) * s23[u] * sin1[u]
                    + matrix(2, 1) * c23[u];
                const Scalar t6x = -matrix(0, 0) * s23[u] * cos1[u]
                    - matrix(1, 0) * s23[u] * sin1[u]
                    - matrix(2, 0) * c23[u];
                theta6_base[u] = std::atan2(t6y, t6x);
            }
        }

        // Step 5/6: enumerate the eight branches in canonical key order and
        // FK-verify each before emission. Duplicate physical configurations
        // (which coincide at singularities) collapse via the wrap/dedup pass;
        // count therefore shrinks legally below eight.
        analytical_result<Scalar, 6, 8> result;
        for (int key = 0; key < 8; ++key)
        {
            const int shoulder = (key >> 2) & 1;
            const int elbow = (key >> 1) & 1;
            const int wrist = key & 1;
            const std::size_t j = static_cast<std::size_t>(shoulder * 2 + elbow);

            Scalar q4 = theta4_base[j];
            Scalar q5 = theta5_base[j];
            Scalar q6 = theta6_base[j];
            if (wrist != 0)
            {
                q4 = theta4_base[j] + pi;
                q5 = -theta5_base[j];
                q6 = theta6_base[j] - pi;
            }

            Eigen::Vector<Scalar, 6> q_internal;
            q_internal << theta1_j[j], theta2_j[j], theta3_j[j], q4, q5, q6;

            // Inverse convention: q_user = (theta_internal + offset) * sign.
            Eigen::Vector<Scalar, 6> q_user;
            bool finite = true;
            for (int i = 0; i < 6; ++i)
            {
                const std::size_t ui = static_cast<std::size_t>(i);
                const Scalar value = (q_internal(i) + p.offsets[ui])
                    * static_cast<Scalar>(p.sign_corrections[ui]);
                if (!std::isfinite(value))
                {
                    finite = false;
                    break;
                }
                q_user(i) = value;
            }
            if (!finite)
                continue;

            if constexpr (std::is_same_v<Verification, opw_verified>)
            {
                // Gate both position and orientation on the same acceptance
                // tolerance: `make()` exposes a single tolerance, so it must bind
                // the orientation check too (verify_analytical_solution otherwise
                // leaves orientation at its own 1e-6 default, silently ignoring a
                // tightened tolerance).
                if (detail::verify_analytical_solution(
                        m_chain, q_user, target, true,
                        m_position_tolerance, m_position_tolerance))
                {
                    const Eigen::Vector<Scalar, 6> q_wrapped = wrap_config(q_user);
                    if (is_duplicate_config(result, q_wrapped))
                        continue;
                    result.solutions[static_cast<std::size_t>(result.count)] =
                        q_wrapped;
                    ++result.count;
                    if (result.count >= max_solutions)
                        return result;
                }
            }
            else
            {
                // opw_raw: no FK back-check, no duplicate collapse -- emit every
                // finite branch in canonical key order.
                result.solutions[static_cast<std::size_t>(result.count)] =
                    wrap_config(q_user);
                ++result.count;
                if (result.count >= max_solutions)
                    return result;
            }
        }

        if (result.count > 0)
            return result;

        // No branch verified. Distinguish a genuine reach/domain failure from
        // a singular locus at which every branch degenerates.
        if (domain_failed)
        {
            return cartan::unexpected(analytical_error<scalar_type>{
                analytical_failure::unreachable, workspace_distance});
        }
        return cartan::unexpected(analytical_error<scalar_type>{
            analytical_failure::singular_configuration, scalar_type(0)});
    }

    const chain_type& chain() const { return m_chain; }

private:
    using Scalar = scalar_type;

    opw_6r_solver(
        const Chain& chain,
        const opw_parameters<scalar_type>& params,
        scalar_type position_tolerance,
        scalar_type singularity_tolerance)
        : m_chain(chain)
        , m_params(params)
        , m_position_tolerance(position_tolerance)
        , m_singularity_tolerance(singularity_tolerance)
    {
        if (chain.num_joints() != 6)
        {
            m_valid = false;
            return;
        }
        for (std::size_t i = 0; i < 6; ++i)
        {
            const auto& s = chain.axis(static_cast<int>(i));
            m_omega[i] = s.omega();
            m_q[i] = s.omega().cross(s.v());
        }
        m_valid = true;
    }

    /// Wrap a joint angle to the half-open interval (-pi, pi].
    static Scalar wrap_angle(Scalar theta)
    {
        Scalar w = std::remainder(theta, Scalar(2) * std::numbers::pi_v<Scalar>);
        if (w <= -std::numbers::pi_v<Scalar>)
            w += Scalar(2) * std::numbers::pi_v<Scalar>;
        return w;
    }

    /// Wrap every component of a joint configuration to (-pi, pi].
    static Eigen::Vector<Scalar, 6> wrap_config(
        const Eigen::Vector<Scalar, 6>& q)
    {
        Eigen::Vector<Scalar, 6> w;
        for (int i = 0; i < 6; ++i)
            w(i) = wrap_angle(q(i));
        return w;
    }

    /// Return true when q coincides, joint-by-joint modulo 2*pi, with a
    /// configuration already stored in result (collapsing pi/-pi aliases too).
    static bool is_duplicate_config(
        const analytical_result<Scalar, 6, 8>& result,
        const Eigen::Vector<Scalar, 6>& q)
    {
        for (int i = 0; i < result.count; ++i)
        {
            const auto& other = result.solutions[static_cast<std::size_t>(i)];
            bool same = true;
            for (int j = 0; j < 6; ++j)
            {
                if (std::abs(wrap_angle(q(j) - other(j)))
                    > detail::sqrt_epsilon_v<Scalar>)
                {
                    same = false;
                    break;
                }
            }
            if (same)
                return true;
        }
        return false;
    }

    chain_type m_chain;
    opw_parameters<scalar_type> m_params;
    std::array<vector3<Scalar>, 6> m_omega;
    std::array<vector3<Scalar>, 6> m_q;
    Scalar m_position_tolerance{default_position_tolerance};
    Scalar m_singularity_tolerance{default_singularity_tolerance};
    bool m_valid{false};
};

template <chain Chain>
opw_6r_solver(const Chain&) -> opw_6r_solver<Chain>;

static_assert(analytical_solver<opw_6r_solver<static_chain<double,
    revolute_z, revolute_y, revolute_y, revolute_z, revolute_y, revolute_z>>>,
    "opw_6r_solver must satisfy analytical_solver concept");

static_assert(analytical_solver<opw_6r_solver<kinematic_chain<double, dynamic>>>,
    "opw_6r_solver must also satisfy analytical_solver concept against dynamic chain");

static_assert(analytical_solver<opw_6r_solver<static_chain<double,
    revolute_z, revolute_y, revolute_y, revolute_z, revolute_y, revolute_z>,
    opw_raw>>,
    "opw_6r_solver<opw_raw> must satisfy analytical_solver concept");

}

#endif
