#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_RECORD_WRITER_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_RECORD_WRITER_H

/// @file record_writer.h
/// @brief Two output tiers for one run, written from one stream of rows.
///
/// The per-target rows are the full-resolution evidence and go to a sidecar
/// directory; the aggregates go beside the report. Both come from the same rows
/// in the untimed pass, so the two can be cross-checked against each other
/// rather than merely being different sizes of the same claim.

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

inline std::ofstream open_table(
    const std::filesystem::path& directory, std::string_view table, std::string_view header)
{
    std::filesystem::create_directories(directory);
    const auto path = directory / ("table_" + std::string{table} + ".csv");
    std::ofstream out(path);
    if (!out)
    {
        throw std::runtime_error(path.string() + ": cannot be opened for writing");
    }
    out << header << '\n';
    return out;
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
        , m_targets(detail::open_table(sidecar, table, target_record_header()))
        , m_cell_dir(cells)
        , m_cells()
    {
    }

    void write(const target_record& row)
    {
        m_targets << csv_row(row) << '\n';
        auto identity = cell_identity(row);
        m_cells.try_emplace(identity, identity, row.solver_tolerance, row.kernel_countable)
            .first->second.add(row);
    }

    void finish()
    {
        m_targets.flush();
        auto out = detail::open_table(m_cell_dir, m_table, cell_header());
        for (const auto& [identity, cell] : m_cells)
        {
            out << cell_row(cell) << '\n';
        }
    }

private:
    std::string m_table;
    std::ofstream m_targets;
    std::filesystem::path m_cell_dir;
    std::map<std::string, cell_accumulator> m_cells;
};

}

#endif
