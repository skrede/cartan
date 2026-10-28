#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_STRATA_CONSTRUCTED_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_STRATA_CONSTRUCTED_H

/// @file strata_constructed.h
/// @brief Filling a stratum by constructing its draws rather than accepting them.
///
/// A limit-adjacent configuration and a walk along a path are vanishingly rare
/// under a uniform draw, so these three are built rather than sampled. The
/// bounds they are built against are the description's own, reached through the
/// library's unbounded-joint interval where a joint declares no bound.

#include "strata_entry.h"

#include <cartan/serial/chain/joint_limits.h>

#include <cmath>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <utility>
#include <algorithm>
#include <stdexcept>

namespace cartan::bench
{

namespace detail
{

template <int N>
std::pair<double, double> usable_interval(const feasible_set<N>& feasible, int joint)
{
    const auto& limit = feasible.declared().limits()[static_cast<std::size_t>(joint)];
    const auto anchored = ::cartan::detail::anchor_bounds(limit.position_min(),
        limit.position_max(), ::cartan::detail::k_unbounded_angular_range_v<double>, 0.0);
    return {anchored.lower, anchored.upper};
}

/// Only a joint with two finite bounds has a declared range to be adjacent to.
/// An unbounded joint's substituted interval is the library's step scale, not a
/// manufacturer limit, and placing a target against it would report adjacency
/// to a number this study invented.
template <int N>
std::vector<int> bounded_joints(const feasible_set<N>& feasible)
{
    std::vector<int> bounded;
    for (int i = 0; i < N; ++i)
    {
        const auto& limit = feasible.declared().limits()[static_cast<std::size_t>(i)];
        if (std::isfinite(limit.position_max() - limit.position_min()))
        {
            bounded.push_back(i);
        }
    }
    return bounded;
}

template <int N>
double place_against_bound(
    const feasible_set<N>& feasible,
    typename target_entry<N>::position_type& q,
    const std::vector<int>& bounded,
    std::mt19937& rng,
    double fraction)
{
    const int joint = bounded[std::uniform_int_distribution<std::size_t>(
        0, bounded.size() - 1)(rng)];
    const auto& limit = feasible.declared().limits()[static_cast<std::size_t>(joint)];
    const double band = fraction * (limit.position_max() - limit.position_min());
    const bool low = std::bernoulli_distribution(0.5)(rng);
    const double anchor = low ? limit.position_min() : limit.position_max() - band;
    q(joint) = anchor + std::uniform_real_distribution<double>(0.0, band)(rng);
    return std::min(q(joint) - limit.position_min(), limit.position_max() - q(joint)) / band;
}

}

template <int N>
std::vector<target_entry<N>> fill_adjacent(
    const feasible_set<N>& feasible, int count, std::uint64_t stream, int ordinal, double fraction)
{
    const auto bounded = detail::bounded_joints<N>(feasible);
    if (bounded.empty())
    {
        throw std::runtime_error(
            "every joint of this robot declares an unbounded range, so no target can be "
            "adjacent to a declared limit");
    }
    std::vector<target_entry<N>> entries;
    entries.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        const auto seed = detail::entry_seed(stream, ordinal, i);
        auto rng = detail::rng_from(seed);
        auto source = detail::draw_config<N>(feasible, rng);
        const double margin =
            detail::place_against_bound<N>(feasible, source, bounded, rng, fraction);
        entries.push_back(
            detail::entry_from_config<N>(feasible, source, rng, i, seed, margin));
    }
    return entries;
}

/// The rotation is identity for every target here, inherited from the
/// bounding-box probe this stratum reuses. A population whose orientations are
/// all one pose is a different problem from one whose orientations are uniform,
/// and the note travels with the stratum into the record.
template <int N, typename Box>
std::vector<target_entry<N>> fill_from_box(
    const feasible_set<N>& feasible, const Box& box, int count, std::uint64_t stream, int ordinal)
{
    std::vector<target_entry<N>> entries;
    entries.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        const auto seed = detail::entry_seed(stream, ordinal, i);
        auto rng = detail::rng_from(seed);
        Eigen::Vector3d point;
        for (int axis = 0; axis < 3; ++axis)
        {
            point(axis) =
                std::uniform_real_distribution<double>(box.tmin(axis), box.tmax(axis))(rng);
        }
        const cartan::se3<double> pose(cartan::so3<double>::identity(), point);
        const double nothing = std::numeric_limits<double>::quiet_NaN();
        entries.push_back(target_entry<N>{pose, detail::draw_config<N>(feasible, rng),
            target_entry<N>::position_type::Constant(nothing), i, seed, nothing, false});
    }
    return entries;
}

/// The ordering is what this stratum measures, so a waypoint follows from the
/// one before it: replaying the walk means replaying it from its first index,
/// and each entry's own seed is the step that produced it.
template <int N>
std::vector<target_entry<N>> fill_walk(
    const feasible_set<N>& feasible, int count, std::uint64_t stream, int ordinal, double step)
{
    auto opening = detail::rng_from(detail::entry_seed(stream, ordinal, -1));
    auto walker = detail::draw_config<N>(feasible, opening);
    std::vector<target_entry<N>> entries;
    entries.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        const auto seed = detail::entry_seed(stream, ordinal, i);
        auto rng = detail::rng_from(seed);
        const auto previous = walker;
        for (int joint = 0; joint < N; ++joint)
        {
            const auto [lower, upper] = detail::usable_interval<N>(feasible, joint);
            const double reach = step * (upper - lower);
            walker(joint) = std::clamp(
                walker(joint) + std::uniform_real_distribution<double>(-reach, reach)(rng),
                lower, upper);
        }
        entries.push_back(target_entry<N>{detail::pose_of<N>(feasible, walker),
            i == 0 ? detail::draw_config<N>(feasible, rng) : previous, walker, i, seed,
            (walker - previous).norm(), true});
    }
    return entries;
}

}

#endif
