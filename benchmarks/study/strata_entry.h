#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_STRATA_ENTRY_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_STRATA_ENTRY_H

/// @file strata_entry.h
/// @brief One generated target, and the draw it is re-derivable from.
///
/// Every entry carries the sixty-four-bit value its own draw was seeded from,
/// so a row in the record names the stream that produced it rather than an
/// index into a stream a reader would have to replay from the beginning.

#include "feasible_set.h"

#include "../../tests/fixtures/chain_factories.h"

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/joint_state.h>

#include <random>
#include <cstdint>

namespace cartan::bench
{

template <int N>
struct target_entry
{
    using position_type = typename cartan::joint_state<double, N>::position_type;

    cartan::se3<double> pose;
    position_type seed;
    position_type generated_from;
    int target_id;
    std::uint64_t rng_seed;
    double classifier_value;
    bool from_configuration;
};

namespace detail
{

/// SplitMix64's finalizer over the stream, the stratum and the index, so the
/// three strata a robot is drawn for do not share a sequence and a target's
/// seed follows from its identity alone.
///
/// Steele, Lea and Flood, Fast Splittable Pseudorandom Number Generators,
/// OOPSLA 2014.
inline std::uint64_t entry_seed(std::uint64_t stream, int ordinal, int index)
{
    constexpr std::uint64_t stratum_gamma = 0x9E3779B97F4A7C15ULL;
    constexpr std::uint64_t index_gamma = 0xD1B54A32D192ED03ULL;
    constexpr std::uint64_t first_mix = 0xBF58476D1CE4E5B9ULL;
    constexpr std::uint64_t second_mix = 0x94D049BB133111EBULL;
    std::uint64_t mixed = stream
        + stratum_gamma * (static_cast<std::uint64_t>(ordinal) + std::uint64_t{1})
        + index_gamma * static_cast<std::uint64_t>(index);
    mixed = (mixed ^ (mixed >> 30)) * first_mix;
    mixed = (mixed ^ (mixed >> 27)) * second_mix;
    return mixed ^ (mixed >> 31);
}

inline std::mt19937 rng_from(std::uint64_t seed)
{
    std::seed_seq sequence{static_cast<std::uint32_t>(seed & 0xFFFFFFFFULL),
        static_cast<std::uint32_t>(seed >> 32)};
    return std::mt19937(sequence);
}

/// The draw reads the declared chain rather than the rule-applied one: a target
/// is the same problem under all three periodic rules, and drawing from the
/// rule-applied bounds would give each table its own population.
template <int N>
typename target_entry<N>::position_type draw_config(
    const feasible_set<N>& feasible, std::mt19937& rng)
{
    return cartan::fixtures::random_joint_config(feasible.declared(), rng);
}

template <int N>
cartan::se3<double> pose_of(
    const feasible_set<N>& feasible, const typename target_entry<N>::position_type& q)
{
    return cartan::forward_kinematics_unchecked(feasible.declared(), q).end_effector;
}

template <int N>
target_entry<N> entry_from_config(
    const feasible_set<N>& feasible,
    const typename target_entry<N>::position_type& source,
    std::mt19937& rng,
    int index,
    std::uint64_t seed,
    double classifier)
{
    return target_entry<N>{pose_of<N>(feasible, source), draw_config<N>(feasible, rng), source,
        index, seed, classifier, true};
}

}

}

#endif
