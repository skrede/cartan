#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_VERDICT_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_VERDICT_H

/// @file verdict.h
/// @brief The harness's own adjudication of a returned joint vector.
///
/// Adjudication runs on every target, not only where the solver claimed
/// success: the re-verification this generalizes was nested inside the success
/// branch and was therefore structurally unable to see a solver that solved the
/// problem and did not notice. `accepted` is the only field a published success
/// figure may be built from; `self_reported` travels beside it as data.
///
/// This is the one place the periodic rule is applied, and it is applied by
/// which chain the canonicalization targets. A joint the rule treats as
/// unbounded has no arc to wrap into, so it passes through untouched while
/// every other joint is wrapped into its declared one -- the rule is a
/// statement about a joint, not a switch that turns wrapping off.

#include "feasible_set.h"

#include "../../tests/fixtures/chain_factories.h"

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/ik/detail/limit_enforcement.h>

#include <cmath>
#include <limits>
#include <cstddef>
#include <algorithm>

namespace cartan::bench
{

struct verdict
{
    bool self_reported;
    bool pose_ok;
    bool limits_ok;
    bool accepted;
    double pos_err;
    double ori_err;
    double worst_limit_violation;
};

namespace detail
{

/// Reported magnitude only. A diverged solve returns a joint value that is not
/// a number, and every ordered comparison against it is false, so a violation
/// assembled out of those comparisons alone reports the divergence as no
/// violation at all; the finiteness test is what makes it visible. The
/// feasibility decision itself is cartan::detail::within_limits, which carries
/// the same test and the boundary tolerance a bound-hugging solve needs.
template <int N>
double worst_violation(
    const cartan::kinematic_chain<double, N>& chain,
    const typename cartan::joint_state<double, N>::position_type& q)
{
    double worst = 0.0;
    const auto& limits = chain.limits();
    for (int i = 0; i < N; ++i)
    {
        if (!std::isfinite(q(i)))
        {
            return std::numeric_limits<double>::infinity();
        }
        const auto idx = static_cast<std::size_t>(i);
        worst = std::max(
            {worst, limits[idx].position_min() - q(i), q(i) - limits[idx].position_max()});
    }
    return worst;
}

}

template <int N>
verdict adjudicate(
    const feasible_set<N>& feasible,
    const typename cartan::joint_state<double, N>::position_type& q,
    const cartan::se3<double>& target,
    bool self_reported,
    double gate)
{
    const double tol = cartan::detail::default_feasibility_tol<double>();
    const auto& against = feasible.verification_chain();
    auto compared = q;
    cartan::detail::canonicalize_into_limits(compared, against, tol);

    const auto [pos_err, ori_err] =
        cartan::fixtures::compute_pose_errors(against, compared, target);
    const bool pose_ok = pos_err < gate && ori_err < gate;
    const bool limits_ok = cartan::detail::within_limits(compared, against, tol);

    return verdict{
        self_reported,
        pose_ok,
        limits_ok,
        pose_ok && limits_ok,
        pos_err,
        ori_err,
        detail::worst_violation<N>(against, compared)};
}

}

#endif
