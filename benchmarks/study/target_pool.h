#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_TARGET_POOL_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_TARGET_POOL_H

/// @file target_pool.h
/// @brief One stratum's targets, drawn once and shared by both passes.
///
/// The untimed pass and the timed pass solve the same pairs in the same order
/// from the same stream, so a figure from one is about the same problem as a
/// figure from the other.

#include "feasible_set.h"

#include "../../tests/fixtures/chain_factories.h"

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/joint_state.h>

#include <random>
#include <vector>
#include <cstddef>
#include <stdexcept>
#include <string_view>

namespace cartan::bench
{

class target_pool
{
public:
    using position_type = typename cartan::joint_state<double, study_joints>::position_type;

    target_pool(
        const feasible_set<study_joints>& feasible,
        std::string_view stratum,
        int count,
        unsigned int seed)
        : m_seeds()
        , m_targets()
    {
        if (stratum != "reachable")
        {
            throw std::runtime_error(
                std::string{stratum} + ": only the reachable stratum is drawn here");
        }
        std::mt19937 rng(seed);
        m_targets.reserve(static_cast<std::size_t>(count));
        m_seeds.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i)
        {
            m_targets.push_back(cartan::fixtures::random_reachable_target(feasible.chain(), rng));
            m_seeds.push_back(cartan::fixtures::random_joint_config(feasible.chain(), rng));
        }
    }

    int size() const { return static_cast<int>(m_targets.size()); }

    const cartan::se3<double>& target(int i) const
    {
        return m_targets.at(static_cast<std::size_t>(i));
    }

    const position_type& seed(int i) const
    {
        return m_seeds.at(static_cast<std::size_t>(i));
    }

private:
    std::vector<position_type> m_seeds;
    std::vector<cartan::se3<double>> m_targets;
};

}

#endif
