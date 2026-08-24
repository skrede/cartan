#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_REACHABILITY_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_REACHABILITY_H

/// @file reachability.h
/// @brief The unreachable stratum's own quantity, which is not a success rate.
///
/// Success on poses that can be reached and correctness about which poses can
/// be reached are different questions, and one rate over both answers neither.
/// A solver reporting failure on a target out of reach is right, not
/// unsuccessful, so this stratum reports how often a return code tells the
/// truth instead.
///
/// Bounding-box sampling cannot guarantee that a target is out of reach, and a
/// target any participant reached is reachable whoever else missed it. Such a
/// target leaves the stratum for every participant rather than being scored
/// against the ones that failed it, and the count of them is published: without
/// it, a solver that simply could not reach a reachable pose would be credited
/// with correctly recognizing an unreachable one.
///
/// The ledger reports the largest budget the run reached. A reachability claim
/// made under a censoring budget is a claim about the budget.

#include "target_record.h"

#include <set>
#include <map>
#include <string>
#include <format>
#include <vector>
#include <fstream>
#include <utility>
#include <filesystem>
#include <string_view>

namespace cartan::bench
{

struct reachability_claim
{
    int target_id;
    bool accepted;
    bool self_reported;
};

struct reachability_counts
{
    int correctly_reported;
    int falsely_claimed;
    int reclassified;
};

inline reachability_counts count_claims(
    const std::vector<reachability_claim>& claims, const std::set<int>& reached)
{
    reachability_counts counts{0, 0, 0};
    for (const auto& claim : claims)
    {
        if (claim.accepted || (!claim.self_reported && reached.count(claim.target_id) > 0))
        {
            ++counts.reclassified;
        }
        else if (claim.self_reported)
        {
            ++counts.falsely_claimed;
        }
        else
        {
            ++counts.correctly_reported;
        }
    }
    return counts;
}

inline std::string_view reachability_header()
{
    return "table,robot,limits_provenance,solver,n_unreachable,"
           "n_correctly_reported_unreachable,n_falsely_claimed_reached,n_reclassified_reached";
}

class reachability_ledger
{
public:
    reachability_ledger()
        : m_rung(-1)
        , m_reached()
        , m_claims()
    {
    }

    void add(const target_record& row)
    {
        if (row.budget_index < m_rung)
        {
            return;
        }
        if (row.budget_index > m_rung)
        {
            m_rung = row.budget_index;
            m_reached.clear();
            m_claims.clear();
        }
        if (row.adjudication.accepted)
        {
            m_reached.insert(row.target_id);
        }
        m_claims[identity(row)].push_back(
            {row.target_id, row.adjudication.accepted, row.adjudication.self_reported});
    }

    bool empty() const { return m_claims.empty(); }

    void write(std::ostream& out) const
    {
        for (const auto& [who, claims] : m_claims)
        {
            const auto counts = count_claims(claims, m_reached);
            out << std::format("{},{},{},{},{}\n", who, claims.size(), counts.correctly_reported,
                counts.falsely_claimed, counts.reclassified);
        }
    }

private:
    int m_rung;
    std::set<int> m_reached;
    std::map<std::string, std::vector<reachability_claim>> m_claims;

    static std::string identity(const target_record& row)
    {
        return std::format("{},{},{},{}", csv_field(row.table), csv_field(row.robot),
            csv_field(row.provenance), csv_field(row.solver));
    }
};

}

#endif
