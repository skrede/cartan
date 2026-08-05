#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_STRATA_SELFCHECK_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_STRATA_SELFCHECK_H

/// @file strata_selfcheck.h
/// @brief The harness's checks on the populations it generates.
///
/// A generator that quietly drew the wrong population would still produce
/// targets, and every figure computed over them would be about a stratum other
/// than the one named in the column. Each stratum is therefore regenerated,
/// checked against the property that defines it, and generated a second time
/// from the same seed and compared entry by entry -- a stratum that is not
/// reproducible is not evidence.

#include "exclusions.h"
#include "target_pool.h"
#include "strata_accepted.h"
#include "instrumentation_gate.h"

#include <cartan/serial/ik/detail/limit_enforcement.h>

#include <cmath>
#include <cstdio>
#include <cstddef>
#include <string_view>

namespace cartan::bench
{

namespace detail
{

template <int N>
bool same_entry(const target_entry<N>& left, const target_entry<N>& right)
{
    int joint = 0;
    const bool poses = left.pose.translation() == right.pose.translation()
        && left.pose.rotation().matrix() == right.pose.rotation().matrix();
    return poses && left.rng_seed == right.rng_seed && left.target_id == right.target_id
        && identical<N>(left.seed, right.seed, joint)
        && identical<N>(left.generated_from, right.generated_from, joint);
}

template <int N>
bool in_limits(const feasible_set<N>& feasible, const typename target_entry<N>::position_type& q)
{
    return cartan::detail::within_limits(
        q, feasible.declared(), cartan::detail::default_feasibility_tol<double>());
}

template <int N>
double bound_margin(
    const feasible_set<N>& feasible, const typename target_entry<N>::position_type& q)
{
    double closest = 1.0;
    for (int i = 0; i < N; ++i)
    {
        const auto& limit = feasible.declared().limits()[static_cast<std::size_t>(i)];
        const double range = limit.position_max() - limit.position_min();
        if (std::isfinite(range))
        {
            closest = std::min(closest,
                std::min(q(i) - limit.position_min(), limit.position_max() - q(i)) / range);
        }
    }
    return closest;
}

template <int N, typename Holds>
bool verify_stratum(
    const feasible_set<N>& feasible,
    stratum which,
    int count,
    std::uint64_t seed,
    const Holds& holds,
    std::string_view assertion)
{
    const auto drawn = generate_stratum<N>(feasible, which, count, seed);
    const auto again = generate_stratum<N>(feasible, which, count, seed);
    for (std::size_t i = 0; i < drawn.size(); ++i)
    {
        if (!same_entry<N>(drawn[i], again[i]))
        {
            std::printf("%s: target %zu differs between two draws from seed %llu\n",
                stratum_name(which).data(), i, static_cast<unsigned long long>(seed));
            return false;
        }
        if (!holds(drawn[i]))
        {
            std::printf("%s: target %zu does not satisfy: %s\n", stratum_name(which).data(), i,
                assertion.data());
            return false;
        }
    }
    std::printf("%s: %d targets, reproducible from seed %llu, every one satisfying: %s\n",
        stratum_name(which).data(), count, static_cast<unsigned long long>(seed),
        assertion.data());
    return true;
}

template <int N>
bool check_sampled_strata(const feasible_set<N>& feasible, int count, std::uint64_t seed)
{
    const double reach = reach_percentile<N>(
        feasible, k_boundary_quantile, k_boundary_calibration_draws, seed);
    bool held = verify_stratum<N>(feasible, stratum::reachable, count, seed,
        [&feasible](const target_entry<N>& entry)
        { return in_limits<N>(feasible, entry.generated_from); },
        "it is the forward kinematics of a configuration inside the declared bounds");
    held = verify_stratum<N>(feasible, stratum::boundary, count, seed,
               [reach](const target_entry<N>& entry)
               { return entry.pose.translation().norm() > reach; },
               "its distance from the base exceeds the reachable stratum's 95th percentile")
        && held;
    return verify_stratum<N>(feasible, stratum::near_singular, count, seed,
               [&feasible](const target_entry<N>& entry) {
                   return near_singular_at<N>(feasible, entry.generated_from).has_value();
               },
               "the library's own near-singularity predicate admits the configuration it came "
               "from")
        && held;
}

template <int N>
bool check_constructed_strata(const feasible_set<N>& feasible, int count, std::uint64_t seed)
{
    bool held = verify_stratum<N>(feasible, stratum::limit_adjacent, count, seed,
        [&feasible](const target_entry<N>& entry) {
            return bound_margin<N>(feasible, entry.generated_from) <= k_limit_adjacent_fraction;
        },
        "one joint of the configuration it came from lies within two per cent of its declared "
        "range of a bound");
    held = verify_stratum<N>(feasible, stratum::unreachable, count, seed,
               [](const target_entry<N>& entry) {
                   return entry.pose.rotation().matrix()
                       == Eigen::Matrix3d::Identity()
                       && !entry.from_configuration;
               },
               "it was drawn over the workspace box and carries the identity rotation")
        && held;
    const auto walk = generate_stratum<N>(feasible, stratum::warm_start_trajectory, count, seed);
    for (std::size_t i = 1; i < walk.size(); ++i)
    {
        int joint = 0;
        if (!identical<N>(walk[i].seed, walk[i - 1].generated_from, joint))
        {
            std::printf("warm_start_trajectory: target %zu is not seeded from its predecessor\n", i);
            return false;
        }
    }
    std::printf("warm_start_trajectory: %d targets, each carrying its predecessor's "
                "configuration as the seed the capture replaces with the accepted solution\n",
        count);
    return held;
}

inline void report_exclusions(limits_provenance provenance)
{
    for (int ordinal = 0; ordinal < k_stratum_count; ++ordinal)
    {
        for (const auto table : {"a", "b", "c"})
        {
            const auto which = static_cast<stratum>(ordinal);
            if (!cell_admissible(provenance, which, rule_for_table(table)))
            {
                std::printf("excluded by the tooling: %s stratum, table %s -- %s\n",
                    stratum_name(which).data(), table,
                    exclusion_for("this run's robot", which, rule_for_table(table)).reason.c_str());
            }
        }
    }
}

}

template <int N>
int run_strata_selfcheck(const feasible_set<N>& feasible, int count, std::uint64_t seed)
{
    const bool sampled = detail::check_sampled_strata<N>(feasible, count, seed);
    const bool constructed = detail::check_constructed_strata<N>(feasible, count, seed);
    detail::report_exclusions(limits_provenance::description);
    detail::report_exclusions(limits_provenance::synthetic);
    return sampled && constructed ? 0 : 1;
}

}

#endif
