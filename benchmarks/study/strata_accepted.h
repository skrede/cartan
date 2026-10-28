#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_STRATA_ACCEPTED_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_STRATA_ACCEPTED_H

/// @file strata_accepted.h
/// @brief Filling a stratum by accepting draws a predicate admits.
///
/// Three of the six strata differ only in which configurations they admit, so
/// the drawing is written once and the predicate is the parameter. A predicate
/// answers with the value it classified by, which is what puts the
/// classification on the record beside the target rather than leaving a reader
/// to trust it.

#include "strata_entry.h"

#include <string>
#include <vector>
#include <cstdint>
#include <optional>
#include <utility>
#include <algorithm>
#include <stdexcept>
#include <string_view>

namespace cartan::bench
{

/// A thin stratum needs many draws per accepted configuration, and a predicate
/// that admits nothing would otherwise spin forever. The cap is per target and
/// the refusal names the measured acceptance rate, so an empty stratum is a
/// reported number rather than a hang.
constexpr int k_acceptance_attempts = 200000;

namespace detail
{

template <int N, typename Accept>
std::optional<std::pair<typename target_entry<N>::position_type, double>> accepted_config(
    const feasible_set<N>& feasible, std::mt19937& rng, const Accept& accept, std::int64_t& draws)
{
    for (int attempt = 0; attempt < k_acceptance_attempts; ++attempt)
    {
        const auto candidate = draw_config<N>(feasible, rng);
        ++draws;
        if (const auto classified = accept(candidate); classified.has_value())
        {
            return std::pair{candidate, *classified};
        }
    }
    return std::nullopt;
}

}

template <int N, typename Accept>
std::vector<target_entry<N>> fill_by_acceptance(
    const feasible_set<N>& feasible,
    int count,
    std::uint64_t stream,
    int ordinal,
    const Accept& accept,
    std::string_view what)
{
    std::vector<target_entry<N>> entries;
    entries.reserve(static_cast<std::size_t>(count));
    std::int64_t draws = 0;
    for (int i = 0; i < count; ++i)
    {
        const auto seed = detail::entry_seed(stream, ordinal, i);
        auto rng = detail::rng_from(seed);
        const auto admitted = detail::accepted_config<N>(feasible, rng, accept, draws);
        if (!admitted.has_value())
        {
            throw std::runtime_error(std::string{what} + ": " + std::to_string(entries.size())
                + " of " + std::to_string(count) + " targets after " + std::to_string(draws)
                + " draws, so the stratum is thinner than the harness can fill");
        }
        entries.push_back(detail::entry_from_config<N>(
            feasible, admitted->first, rng, i, seed, admitted->second));
    }
    return entries;
}

/// The distance a boundary target has to beat, measured over the reachable
/// stratum for this robot rather than over a hand-picked radius: the boundary
/// is defined relative to the population it is the boundary of.
template <int N>
double reach_percentile(
    const feasible_set<N>& feasible, double quantile, int samples, std::uint64_t stream)
{
    auto rng = detail::rng_from(stream);
    std::vector<double> distances;
    distances.reserve(static_cast<std::size_t>(samples));
    for (int i = 0; i < samples; ++i)
    {
        const auto q = detail::draw_config<N>(feasible, rng);
        distances.push_back(detail::pose_of<N>(feasible, q).translation().norm());
    }
    std::sort(distances.begin(), distances.end());
    const auto rank = static_cast<std::size_t>(quantile * static_cast<double>(distances.size()));
    return distances[std::min(rank, distances.size() - 1)];
}

}

#endif
