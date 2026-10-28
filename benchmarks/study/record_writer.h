#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_RECORD_WRITER_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_RECORD_WRITER_H

/// @file record_writer.h
/// @brief Three output tiers for one run, written from one stream of rows.
///
/// The per-target rows are the full-resolution evidence and go to a sidecar
/// directory; the aggregates go beside the report; the reachability breakdown
/// is its own artifact because it is not a success rate and must not be read as
/// one. All three come from the same rows in the untimed pass, so the two
/// smaller tiers can be cross-checked against the largest rather than merely
/// being different sizes of the same claim.
///
/// Every filename carries its table's key. A statistic spanning two periodic
/// rules is not a comparison -- the solves themselves differ -- and one file per
/// table is what makes constructing one a deliberate act rather than a column
/// read off an artifact that already holds both.

#include "reachability.h"
#include "target_record.h"
#include "cell_aggregate.h"

#include <map>
#include <string>
#include <fstream>
#include <stdexcept>
#include <filesystem>
#include <string_view>

namespace cartan::bench
{

namespace detail
{

inline std::ofstream open_with_header(
    const std::filesystem::path& path, std::string_view header)
{
    if (path.has_parent_path())
    {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream out(path);
    if (!out)
    {
        throw std::runtime_error(path.string() + ": cannot be opened for writing");
    }
    out << header << '\n';
    return out;
}

inline std::ofstream open_tier(
    const std::filesystem::path& directory,
    std::string_view table,
    std::string_view tier,
    std::string_view header)
{
    return open_with_header(
        directory / ("table_" + std::string{table} + "_" + std::string{tier} + ".csv"), header);
}

}

class record_writer
{
public:
    record_writer(
        const std::filesystem::path& cells,
        const std::filesystem::path& sidecar,
        std::string_view table)
        : m_table(table)
        , m_targets(detail::open_tier(sidecar, table, "targets", target_record_header()))
        , m_cell_dir(cells)
        , m_cells()
        , m_reach()
    {
    }

    void write(const target_record& row)
    {
        m_targets << csv_row(row) << '\n';
        auto identity = cell_identity(row);
        m_cells
            .try_emplace(identity, identity, std::string{row.accuracy_target},
                std::string{row.accuracy_target_met}, row.solver_tolerance, row.kernel_countable,
                row.success_rate_reportable)
            .first->second.add(row);
        if (!row.success_rate_reportable)
        {
            m_reach.add(row);
        }
    }

    void finish()
    {
        m_targets.flush();
        auto out = detail::open_tier(m_cell_dir, m_table, "cells", cell_header());
        for (const auto& [identity, cell] : m_cells)
        {
            out << cell_row(cell) << '\n';
        }
        if (m_reach.empty())
        {
            return;
        }
        auto claims = detail::open_tier(m_cell_dir, m_table, "reachability", reachability_header());
        m_reach.write(claims);
    }

private:
    std::string m_table;
    std::ofstream m_targets;
    std::filesystem::path m_cell_dir;
    std::map<std::string, cell_accumulator> m_cells;
    reachability_ledger m_reach;
};

}

#endif
