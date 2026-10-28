#ifndef HPP_GUARD_CARTAN_BENCHMARKS_CHECKED_CACHE_H
#define HPP_GUARD_CARTAN_BENCHMARKS_CHECKED_CACHE_H

/// @file checked_cache.h
/// @brief Precomputation of the kinematics caches a timed loop indexes into.

#include "../tests/fixtures/chain_factories.h"

#include <cartan/serial/chain/chain_failure.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <benchmark/benchmark.h>

#include <random>
#include <utility>

namespace cartan::bench
{

/// Fill cache with the values produce answers, or mark this cell in error and
/// answer false so the caller can return before entering its timed loop.
///
/// A cache is built outside the timed region, so it takes the checked entry
/// point while the timed loop that reads it names the unchecked sibling. A
/// refusal marks one cell and leaves every measurement the binary has already
/// accumulated intact; aborting would discard all of them.
template <typename Cache, typename Produce>
bool fill_cache(Cache& cache, benchmark::State& state, Produce produce)
{
    for (auto& entry : cache)
    {
        auto held = produce();
        if (!held)
        {
            state.SkipWithError(message(held.error()));
            return false;
        }
        entry = std::move(*held);
    }
    return true;
}

/// fill_cache over forward kinematics at configurations drawn from the chain.
template <typename Cache, typename Chain>
bool fill_fk_cache(
    Cache& cache,
    const Chain& chain,
    std::mt19937& rng,
    benchmark::State& state)
{
    return fill_cache(cache, state,
        [&] { return forward_kinematics(chain, fixtures::random_joint_config(chain, rng)); });
}

}

#endif
