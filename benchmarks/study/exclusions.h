#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_EXCLUSIONS_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_EXCLUSIONS_H

/// @file exclusions.h
/// @brief Which cells a run refuses to emit, and what it records instead.
///
/// A row that is present is a claim. A row that is absent with a recorded
/// reason is a different and honest claim. A row that is absent with no reason
/// is the defect this study exists to remove -- and a qualification written
/// only in prose beside a table detaches from it the first time the table is
/// quoted, which is exactly how the previous study's caveats came unstuck.

#include "feasible_set.h"
#include "target_strata.h"

#include <string>
#include <vector>

namespace cartan::bench
{

struct excluded_cell
{
    std::string robot;
    std::string stratum;
    std::string table;
    std::string reason;
};

/// A stratum defined by adjacency to a declared bound says nothing when the
/// bound was invented here, and says less than nothing under a rule that then
/// relaxes it: the row would be about two different bounds at once.
inline bool cell_admissible(limits_provenance provenance, stratum which, periodic_rule rule)
{
    const bool invented = provenance == limits_provenance::synthetic;
    return !(invented && which == stratum::limit_adjacent
        && !rule_preserves_declared_bounds(rule));
}

inline excluded_cell exclusion_for(
    const std::string& robot, stratum which, periodic_rule rule)
{
    return excluded_cell{robot, std::string{stratum_name(which)},
        std::string{table_key(rule)},
        "the bounds this run used were not read from a description, and this stratum is "
        "defined by adjacency to a bound that this table's rule also relaxes"};
}

}

#endif
