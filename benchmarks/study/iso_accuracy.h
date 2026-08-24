#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_ISO_ACCURACY_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_ISO_ACCURACY_H

/// @file iso_accuracy.h
/// @brief The calibrated per-solver tolerance table the accuracy mode reads.
///
/// Requested tolerance and achieved accuracy are different numbers, and they do
/// not differ by the same factor from one solver to the next. Handing every
/// participant the same requested gate therefore compares them at different
/// accuracies and calls it a controlled comparison. A row of this table says
/// what one solver had to be asked for to deliver one measured accuracy on one
/// robot, and a capture reads its tolerance from the row rather than from a
/// value shared across participants.
///
/// The lookup refuses rather than substitutes. A participant this table does not
/// cover cannot be run at a default tolerance in a mode whose whole claim is
/// matched accuracy: the resulting table would say the opposite of what happened.
/// The bounds a row was searched under are part of its identity for the same
/// reason, so a run under one provenance cannot silently read a row measured
/// under the other.

#include "calibration_row.h"
#include "calibration_parse.h"

#include <array>
#include <format>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <filesystem>
#include <string_view>

namespace cartan::bench
{

constexpr std::array<double, 3> k_accuracy_targets{1e-5, 1e-6, 1e-7};
constexpr int k_bisection_steps = 16;

/// What a record row says about the accuracy target it ran under, preformatted:
/// both fields empty under the budget mode, and on an accuracy-mode row the
/// target beside whether the calibration behind it reached the target.
struct accuracy_claim
{
    std::string target;
    std::string met;
};

inline bool is_declared_target(double target)
{
    for (const double declared : k_accuracy_targets)
    {
        if (target == declared)
        {
            return true;
        }
    }
    return false;
}

class calibration_table
{
public:
    static calibration_table load(const std::filesystem::path& path)
    {
        std::ifstream in(path);
        if (!in)
        {
            throw std::runtime_error(path.string() + ": cannot be opened for reading");
        }
        std::string line;
        std::getline(in, line);
        refuse_foreign_header(path, detail::without_return(line));
        calibration_table loaded;
        for (int number = 2; std::getline(in, line); ++number)
        {
            line = detail::without_return(std::move(line));
            if (!line.empty())
            {
                loaded.m_rows.push_back(detail::parse_calibration_row(line, number));
            }
        }
        return loaded;
    }

    const calibration_row& row_for(std::string_view table, std::string_view robot,
        std::string_view provenance, std::string_view solver, double target) const
    {
        for (const auto& row : m_rows)
        {
            if (row.table == table && row.robot == robot && row.limits_provenance == provenance
                && row.solver == solver && detail::same_target(row.accuracy_target, target))
            {
                return row;
            }
        }
        throw std::runtime_error(std::string{solver} + ": the calibration table carries no row for "
            + std::string{robot} + " on table " + std::string{table} + " under "
            + std::string{provenance} + " bounds at accuracy target "
            + std::format("{:g}", target)
            + ", and running it at a default tolerance in a matched-accuracy mode would produce a "
              "table saying the opposite of what happened");
    }

    double tolerance_for(std::string_view table, std::string_view robot,
        std::string_view provenance, std::string_view solver, double target) const
    {
        return row_for(table, robot, provenance, solver, target).calibrated_tolerance;
    }

private:
    std::vector<calibration_row> m_rows;

    calibration_table()
        : m_rows()
    {
    }

    static void refuse_foreign_header(const std::filesystem::path& path, const std::string& header)
    {
        if (header != calibration_header())
        {
            throw std::runtime_error(path.string() + ": its header is '" + header
                + "' against the '" + std::string{calibration_header()}
                + "' a calibration table declares, so it is refused rather than partially read");
        }
    }
};

}

#endif
