#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_LIMIT_ENFORCEMENT_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_LIMIT_ENFORCEMENT_H

/// Shared limit enforcement utility for IK solve policies.
///
/// Extracts the limit enforcement logic from ik_solver into a reusable
/// free function template. Policies call this in their step() method
/// instead of relying on post-hoc enforcement by the solver.

#include "cartan/serial/ik/policy/limits_policy.h"

#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include "cartan/detail/epsilon.h"

#include <Eigen/SVD>

#include <cmath>
#include <limits>
#include <numbers>
#include <concepts>
#include <cstddef>
#include <algorithm>

namespace cartan
{
namespace detail
{

/// Default tolerance for the feasibility check. Set to sqrt(machine epsilon):
/// the joint value only needs to sit within a half-precision-scale band of the
/// finite bound, so a configuration that is exactly at (or a rounding step past)
/// a limit still reads as feasible while one that has genuinely drifted out of
/// range does not. This is the boundary tolerance for within_limits and is a
/// candidate for an empirical sweep should a bound-hugging solver need slack.
template <typename Scalar>
inline Scalar default_feasibility_tol() noexcept
{
    return std::sqrt(std::numeric_limits<Scalar>::epsilon());
}

/// Feasibility predicate: true iff every finitely-bounded joint of q lies within
/// [position_min - tol, position_max + tol]. Joints whose bound is non-finite
/// (unbounded / continuous joints using +/-infinity) are skipped on that side,
/// so a joint with one finite and one infinite bound is still checked against
/// the finite side. This is a CHECK only -- it never mutates q. It is the gate a
/// no_limits trust-region solver consults before declaring convergence so that a
/// pose-converged but out-of-range configuration is reported as a joint-limit
/// failure rather than a trustworthy solution.
template <chain Chain>
bool within_limits(
    const typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type& q,
    const Chain& chain,
    typename Chain::scalar_type tol)
{
    const auto& limits = chain.limits();
    int n = chain.num_joints();
    for (int i = 0; i < n; ++i)
    {
        auto idx = static_cast<std::size_t>(i);
        auto lo = limits[idx].position_min;
        auto hi = limits[idx].position_max;
        if (std::isfinite(lo) && q(i) < lo - tol)
        {
            return false;
        }
        if (std::isfinite(hi) && q(i) > hi + tol)
        {
            return false;
        }
    }
    return true;
}

/// True iff the screw axis is a zero-pitch revolute joint: |omega| > 0 and the
/// screw pitch h = (omega . v) / |omega|^2 is ~ 0. Only a zero-pitch revolute
/// produces a pose that is 2*pi-periodic in its joint angle -- exp(2*pi*S) = I,
/// so adding a full turn to the angle leaves the end-effector pose unchanged and
/// the angle may be wrapped into the joint's limit arc without moving the tool.
///
/// A prismatic joint (omega = 0) is aperiodic. A helical / nonzero-pitch screw
/// (omega != 0, h != 0) satisfies exp(2*pi*S) != I -- it translates by 2*pi*h per
/// revolution -- so its angle is likewise not pose-periodic. Neither may be
/// wrapped; both are gated out here. The pitch test uses |omega . v| directly:
/// with |omega| = 1 for a normalized revolute axis, omega . v == h, and a pure
/// revolute (v = -omega x point) gives omega . v = 0 identically while a helical
/// axis carries omega . v = h. The threshold scales with |v| so the rounding of
/// the cross product on a large-reach chain is absorbed without admitting a
/// physically meaningful (millimeter-scale) pitch as "zero".
template <typename Scalar>
inline bool is_zero_pitch_revolute(const screw_axis<Scalar>& s) noexcept
{
    if (!s.is_revolute())
    {
        return false;
    }
    const Scalar wv = std::abs(s.omega().dot(s.v()));
    const Scalar scale = std::max(Scalar(1), s.v().norm());
    return wv <= sqrt_epsilon_v<Scalar> * scale;
}

/// Return the 2*pi-equivalent of theta that lands in the limit arc [lo, hi], or
/// the nearest representative on the lower (resp. upper) side when the arc is a
/// proper sub-interval of the circle that theta's equivalence class misses. The
/// caller must only pass angles of a zero-pitch revolute joint (see
/// is_zero_pitch_revolute), for which theta and theta + 2*pi*k describe the same
/// pose for every integer k. This is the general arc test -- NOT a naive wrap to
/// [-pi, pi]: it anchors to the actual bounds so an asymmetric or offset limit
/// range is handled correctly.
///
/// Bound handling mirrors within_limits: a non-finite bound (an unbounded /
/// continuous joint modeled with +/-infinity) imposes no constraint on that side.
///   - lo finite: return the smallest theta + 2*pi*k >= lo (within the tol band).
///     If hi is finite and the span hi - lo >= 2*pi this always lands in the arc;
///     if the span is a gap < 2*pi that the class misses, the result is >= lo but
///     > hi, so the subsequent box check correctly rejects it.
///   - lo = -inf, hi finite: return the largest theta + 2*pi*k <= hi.
///   - both non-finite: a fully continuous joint -- theta is already feasible and
///     is returned unchanged.
/// The tol band matches within_limits so a bound-hugging angle is not bumped a
/// full turn by a rounding step across the limit.
///
/// An angle already inside the arc is returned UNCHANGED (identity map): the
/// canonicalization only ever moves a representative the solver returned outside
/// the box, so a configuration already within limits is left bit-for-bit intact.
/// Without this guard an angle sitting exactly on the upper bound of a span-2*pi
/// range would be needlessly re-anchored to the lower bound (e.g. +pi -> -pi).
template <typename Scalar>
inline Scalar canonical_angle_in_limits(
    Scalar theta, Scalar lo, Scalar hi, Scalar tol) noexcept
{
    const bool above_lo = !std::isfinite(lo) || theta >= lo - tol;
    const bool below_hi = !std::isfinite(hi) || theta <= hi + tol;
    if (above_lo && below_hi)
    {
        return theta;
    }

    constexpr Scalar two_pi = Scalar(2) * std::numbers::pi_v<Scalar>;
    if (std::isfinite(lo))
    {
        const Scalar k = std::ceil((lo - theta - tol) / two_pi);
        return theta + two_pi * k;
    }
    if (std::isfinite(hi))
    {
        const Scalar k = std::floor((hi - theta + tol) / two_pi);
        return theta + two_pi * k;
    }
    return theta;
}

/// Pose-preserving canonicalization of q into the joint limits: every zero-pitch
/// revolute DOF with at least one finite bound is replaced by its in-arc 2*pi-
/// equivalent (see canonical_angle_in_limits); prismatic, helical / nonzero-pitch,
/// general-screw, and fully-continuous DOFs are left untouched. The end-effector
/// pose is invariant under this map because a full turn of a zero-pitch revolute
/// joint is the identity displacement.
///
/// This is the ONE-SHOT post-convergence canonicalization a no_limits trust-region
/// solver applies to its FINAL returned iterate. It must never be fed back into the
/// iteration: it is not a clamp, and clamping the working iterate would destroy the
/// trust-region geometry the no_limits family exists to preserve. Feasibility is a
/// property of the pose-equivalence class, not of the arbitrary R^n representative
/// the unconstrained solver happened to return -- an unwrapped 2*pi-equivalent
/// angle is recovered to its in-limits representative here.
template <chain Chain>
void canonicalize_into_limits(
    typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type& q,
    const Chain& chain,
    typename Chain::scalar_type tol)
{
    const auto& limits = chain.limits();
    int n = chain.num_joints();
    for (int i = 0; i < n; ++i)
    {
        if (!is_zero_pitch_revolute(chain.axis(i)))
        {
            continue;
        }
        auto idx = static_cast<std::size_t>(i);
        q(i) = canonical_angle_in_limits(
            q(i), limits[idx].position_min, limits[idx].position_max, tol);
    }
}

/// Canonicalize q into the joint limits (pose-preserving) and report whether the
/// result is feasible. This is the gate a no_limits trust-region solver consults
/// at convergence: a pose the unconstrained solver reached at an unwrapped
/// 2*pi-equivalent angle is recovered to its in-limits representative and accepted,
/// while a pose genuinely reachable only outside the box (a prismatic joint out of
/// range, or a revolute whose limit arc < 2*pi excludes the required angle) still
/// fails. On return q holds the canonical representative regardless of the verdict;
/// callers store it as the solution only on the feasible (converged) path.
template <chain Chain>
bool feasible_after_canonicalization(
    typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type& q,
    const Chain& chain,
    typename Chain::scalar_type tol)
{
    canonicalize_into_limits(q, chain, tol);
    return within_limits(q, chain, tol);
}

/// Apply limit enforcement to joint configuration q using the given LimitsPolicy.
///
/// If LimitsPolicy is no_limits, this is a no-op (optimized away via if constexpr).
/// If LimitsPolicy satisfies has_extended_enforce, computes FK, body Jacobian, and
/// SVD for null-space projection before calling enforce_extended.
/// Otherwise, calls the simple enforce method.
template <typename LimitsPolicy, chain Chain>
void enforce_limits(
    typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type& q,
    const Chain& chain)
{
    using Scalar = typename Chain::scalar_type;
    static constexpr int N = Chain::joints;

    if constexpr (std::same_as<LimitsPolicy, no_limits>)
    {
        return;
    }
    else if constexpr (has_extended_enforce<LimitsPolicy, Chain>)
    {
        auto fk = forward_kinematics(chain, q);
        auto J_b = body_jacobian(chain, fk);

        // V must be full: matrixV() is then n x n and V.rightCols(n - rank)
        // spans the true Jacobian kernel. A thin V is only n x min(m, n), so
        // for a wide (redundant) Jacobian it omits the kernel columns and hands
        // back row-space directions instead, corrupting the null-space step.
        // U may stay thin for a dynamic matrix (thin U is illegal for a
        // fixed-size one, hence the branch), and its 6 rows make thin and full
        // U identical in size regardless.
        constexpr unsigned int svd_options = (N == dynamic)
            ? (Eigen::ComputeThinU | Eigen::ComputeFullV)
            : (Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<jacobian_matrix<Scalar, N>> svd(J_b, svd_options);

        LimitsPolicy::template enforce_extended<Chain>(
            q, chain.limits(), J_b, svd);
    }
    else
    {
        LimitsPolicy::template enforce<Chain>(q, chain.limits());
    }
}

}
}

#endif
