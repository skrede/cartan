#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_PARTICIPANT_LIST_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_PARTICIPANT_LIST_H

/// @file participant_list.h
/// @brief What the run expected to solve with, and what it actually resolved.
///
/// A configuration that produced zero cells and read exactly like a disabled one
/// has already happened twice in this suite, and neither occurrence warned.
/// Under system-provided comparators a partial participant set is the ordinary
/// path rather than an error path, so the declared list is written down here and
/// the difference between it and the resolved list is carried into the manifest
/// by name instead of being inferred from a smaller table.

#include "target_pool.h"
#include "solve_outcome.h"
#include "build_manifest.h"

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
    bool kernel_countable;
    solve_call solve;
};

/// `supplier` names the dependency the participant needs, so an absence can
/// carry the reason the configure already recorded rather than a second
/// explanation written here.
struct declared_participant
{
    std::string_view name;
    std::string_view supplier;
    bool kernel_countable;
};

inline const std::array<declared_participant, 3>& declared_participants()
{
    static const std::array<declared_participant, 3> declared{
        declared_participant{"cartan_lm", "", true},
        declared_participant{"pinocchio_lm", "pinocchio", true},
        declared_participant{"trac_ik", "trac_ik", false}};
    return declared;
}

inline void print_participants(const std::vector<participant_entry>& resolved)
{
    std::printf("declared participants: %zu, resolved: %zu\n",
        declared_participants().size(), resolved.size());
    for (const auto& entry : resolved)
    {
        std::printf("  resolved participant: %s (kernel evaluations %s)\n", entry.name.c_str(),
            entry.kernel_countable ? "counted by the harness" : "not countable");
    }
}

inline std::vector<absent_participant> absent_participants(
    const std::vector<participant_entry>& resolved)
{
    std::vector<absent_participant> absent;
    const auto reasons = build_absences();
    for (const auto& declared : declared_participants())
    {
        const bool present = std::any_of(resolved.begin(), resolved.end(),
            [declared](const participant_entry& entry) { return entry.name == declared.name; });
        if (present)
        {
            continue;
        }
        std::string reason{"absent from this build"};
        for (const auto& recorded : reasons)
        {
            if (recorded.name == declared.supplier)
            {
                reason = recorded.reason;
            }
        }
        absent.push_back(absent_participant{std::string{declared.name}, reason});
    }
    return absent;
}

inline void report_absent_participants(const std::vector<absent_participant>& absent)
{
    for (const auto& entry : absent)
    {
        std::printf("  declared but unresolved: %s -- %s\n", entry.name.c_str(),
            entry.reason.c_str());
    }
}

inline void refuse_empty_participants(const std::vector<participant_entry>& resolved)
{
    if (resolved.empty())
    {
        throw std::runtime_error(
            "the participant group resolved to no solvers at all, so there is nothing to measure");
    }
}

}

#endif
