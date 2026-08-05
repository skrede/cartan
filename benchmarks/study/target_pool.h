#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_TARGET_POOL_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_TARGET_POOL_H

/// @file target_pool.h
/// @brief One stratum's targets, drawn once and shared by both passes.
///
/// The untimed pass and the timed pass solve the same entries in the same order
/// from the same draw, so a figure from one is about the same problem as a
/// figure from the other. The pool also knows whether its stratum is ordered:
/// on the warm-start stratum the ordering is the thing being measured, and a
/// pass that reordered it would measure something else.

#include "target_strata.h"

#include <cartan/lie/se3.h>

#include <vector>
#include <cstdint>
#include <cstddef>

namespace cartan::bench
{

template <int N>
class target_pool
{
public:
    using position_type = typename target_entry<N>::position_type;

    target_pool(const feasible_set<N>& feasible, stratum which, int count, std::uint64_t seed)
        : m_stratum(which)
        , m_entries(generate_stratum<N>(feasible, which, count, seed))
    {
    }

    int size() const { return static_cast<int>(m_entries.size()); }

    stratum which() const { return m_stratum; }

    bool ordered() const { return m_stratum == stratum::warm_start_trajectory; }

    const target_entry<N>& entry(int i) const
    {
        return m_entries.at(static_cast<std::size_t>(i));
    }

    const cartan::se3<double>& target(int i) const { return entry(i).pose; }

    const position_type& seed(int i) const { return entry(i).seed; }

private:
    stratum m_stratum;
    std::vector<target_entry<N>> m_entries;
};

}

#endif
