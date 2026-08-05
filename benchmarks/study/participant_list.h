#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_PARTICIPANT_LIST_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_PARTICIPANT_LIST_H

/// @file participant_list.h
/// @brief What the run expected to solve with, and what it actually resolved.
///
/// A configuration that produced zero cells and read exactly like a disabled
/// one has already happened twice in this suite, and neither occurrence warned.
/// The declared list is written down here so the difference between it and the
/// resolved list can be named rather than inferred from an empty table.

#include "target_pool.h"
#include "solve_outcome.h"

#include <cartan/lie/se3.h>

#include <array>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <functional>
#include <string_view>

namespace cartan::bench
{

using solve_call = std::function<solve_outcome<study_joints>(
    const cartan::se3<double>&, const typename target_pool::position_type&)>;

struct participant_entry
{
    std::string name;
    solve_call solve;
};

inline const std::array<std::string_view, 2>& declared_participants()
{
    static const std::array<std::string_view, 2> declared{"cartan_lm", "pinocchio_lm"};
    return declared;
}

inline void print_participants(const std::vector<participant_entry>& resolved)
{
    std::printf("declared participants: %zu, resolved: %zu\n",
        declared_participants().size(), resolved.size());
    for (const auto& entry : resolved)
    {
        std::printf("  resolved participant: %s\n", entry.name.c_str());
    }
}

inline void refuse_incomplete_participants(const std::vector<participant_entry>& resolved)
{
    std::string absent;
    for (const auto declared : declared_participants())
    {
        const bool present = std::any_of(resolved.begin(), resolved.end(),
            [declared](const participant_entry& entry) { return entry.name == declared; });
        if (!present)
        {
            absent += absent.empty() ? "" : ", ";
            absent += declared;
        }
    }
    if (!absent.empty())
    {
        throw std::runtime_error(
            "the participant group resolved to " + std::to_string(resolved.size()) + " of "
            + std::to_string(declared_participants().size())
            + " declared solvers; absent from this build: " + absent);
    }
}

}

#endif
