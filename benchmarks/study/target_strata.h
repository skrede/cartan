#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_TARGET_STRATA_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_TARGET_STRATA_H

/// @file target_strata.h
/// @brief The six target populations a conclusion may be reported over.
///
/// A success rate is a statement about a population, and the previously
/// published study reported reachable-only figures with nothing in the record
/// saying which population they were about. Each stratum is defined here by the
/// predicate that admits a configuration into it; the loops that fill them are
/// written once each, next door, with the predicate as their parameter.
///
/// Near-singularity is the library's own measure at its own resolution floor
/// and the workspace box its own forward-kinematics sweep. A second definition
/// of either would be a second study wearing this one's column names.

#include "strata_entry.h"
#include "strata_accepted.h"
#include "strata_constructed.h"

#include "../closed_form_bench_utils.h"

#include <cartan/serial/fk/singular_spectrum.h>
#include <cartan/serial/fk/singularity_analysis.h>

#include <vector>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace cartan::bench
{

enum class stratum
{
    reachable,
    boundary,
    near_singular,
    limit_adjacent,
    unreachable,
    warm_start_trajectory
};

constexpr int k_stratum_count = 6;
constexpr double k_boundary_quantile = 0.95;
constexpr int k_boundary_calibration_draws = 2000;
constexpr double k_limit_adjacent_fraction = 0.02;
constexpr double k_walk_step_fraction = 0.02;
constexpr int k_box_probe_draws = 10000;

inline std::string_view stratum_name(stratum which)
{
    switch (which)
    {
        case stratum::reachable: return "reachable";
        case stratum::boundary: return "boundary";
        case stratum::near_singular: return "near_singular";
        case stratum::limit_adjacent: return "limit_adjacent";
        case stratum::unreachable: return "unreachable";
        case stratum::warm_start_trajectory: return "warm_start_trajectory";
    }
    throw std::runtime_error("the stratum enumeration gained a value with no name");
}

inline stratum stratum_from_name(std::string_view name)
{
    for (int ordinal = 0; ordinal < k_stratum_count; ++ordinal)
    {
        const auto which = static_cast<stratum>(ordinal);
        if (stratum_name(which) == name) { return which; }
    }
    throw std::runtime_error(std::string{name} + ": the study carries no such stratum");
}

/// What the fixture's classifier column holds, and what population the rows are
/// about. The second travels into the metadata: a reader who does not know the
/// unreachable stratum carries one orientation will over-read every figure.
struct stratum_note
{
    std::string_view classifier;
    std::string_view population;
};

inline stratum_note stratum_notes(stratum which)
{
    switch (which)
    {
        case stratum::reachable:
            return {"none", "the forward kinematics of configurations inside the declared bounds"};
        case stratum::boundary:
            return {"distance_from_base_m",
                "reachable, beyond the reachable stratum's own 95th percentile distance"};
        case stratum::near_singular:
            return {"condition_number",
                "generated from configurations the library's near-singularity predicate admits"};
        case stratum::limit_adjacent:
            return {"fraction_of_declared_range_from_the_bound",
                "at least one joint within two per cent of its declared range from a bound"};
        case stratum::unreachable:
            return {"none",
                "uniform over the forward-kinematics bounding box at identity rotation, so every "
                "target carries the same orientation and none is sampled"};
        case stratum::warm_start_trajectory:
            return {"joint_space_step_rad",
                "an ordered walk, each target seeded with the previously accepted solution"};
    }
    throw std::runtime_error("the stratum enumeration gained a value with no notes");
}

namespace detail
{

/// The condition number where the library's own predicate admits the
/// configuration, and nothing where it does not. A hand-rolled rank test calls
/// a structurally singular Jacobian merely ill-conditioned on one instruction
/// set and singular on another; the library answers both at its own floor.
template <int N>
std::optional<double> near_singular_at(
    const feasible_set<N>& feasible, const typename target_entry<N>::position_type& q)
{
    const auto sigma = cartan::singular_values(feasible.declared(), q);
    if (!sigma)
    {
        return std::nullopt;
    }
    const auto admitted = cartan::is_near_singular(*sigma);
    if (!admitted || !*admitted)
    {
        return std::nullopt;
    }
    return cartan::condition_number(*sigma).value_or(
        std::numeric_limits<double>::infinity());
}

template <int N>
std::vector<target_entry<N>> generate_near_singular(
    const feasible_set<N>& feasible, int count, std::uint64_t stream, int ordinal)
{
    return fill_by_acceptance<N>(feasible, count, stream, ordinal,
        [&feasible](const typename target_entry<N>::position_type& q)
        { return near_singular_at<N>(feasible, q); },
        "the near-singular stratum");
}

template <int N>
std::vector<target_entry<N>> generate_boundary(
    const feasible_set<N>& feasible, int count, std::uint64_t stream, int ordinal)
{
    const double threshold = reach_percentile<N>(
        feasible, k_boundary_quantile, k_boundary_calibration_draws, stream);
    return fill_by_acceptance<N>(feasible, count, stream, ordinal,
        [&feasible, threshold](const typename target_entry<N>::position_type& q)
        {
            const double distance = pose_of<N>(feasible, q).translation().norm();
            return distance > threshold ? std::optional<double>{distance} : std::nullopt;
        },
        "the boundary stratum");
}

}

/// Dispatch only. Every generator draws from the declared bounds and seeds
/// itself from a value each entry carries, so a target is derivable from its own
/// record rather than from a replayed stream.
template <int N>
std::vector<target_entry<N>> generate_stratum(
    const feasible_set<N>& feasible, stratum which, int count, std::uint64_t seed)
{
    const int ordinal = static_cast<int>(which);
    const auto anything = [](const typename target_entry<N>::position_type&)
    { return std::optional<double>{0.0}; };
    switch (which)
    {
        case stratum::reachable:
            return fill_by_acceptance<N>(
                feasible, count, seed, ordinal, anything, "the reachable stratum");
        case stratum::boundary:
            return detail::generate_boundary<N>(feasible, count, seed, ordinal);
        case stratum::near_singular:
            return detail::generate_near_singular<N>(feasible, count, seed, ordinal);
        case stratum::limit_adjacent:
            return fill_adjacent<N>(feasible, count, seed, ordinal, k_limit_adjacent_fraction);
        case stratum::unreachable:
            return fill_from_box<N>(feasible,
                cartan::fixtures::compute_bounding_box(
                    feasible.declared(), k_box_probe_draws, static_cast<unsigned>(seed)),
                count, seed, ordinal);
        case stratum::warm_start_trajectory:
            return fill_walk<N>(feasible, count, seed, ordinal, k_walk_step_fraction);
    }
    throw std::runtime_error("the stratum enumeration gained a value with no generator");
}

}

#endif
