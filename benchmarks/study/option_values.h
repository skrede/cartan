#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_OPTION_VALUES_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_OPTION_VALUES_H

/// @file option_values.h
/// @brief The values a study program's flags may carry, and their refusals.
///
/// Each of these turns a command line's text into one of the study's own
/// vocabularies, and refuses anything outside it with the reason rather than
/// with a usage line. A request the harness cannot honor exactly is turned away
/// here, before anything is drawn or solved, because the alternative is a run
/// that quietly measures the nearest thing it could do.

#include "budget.h"
#include "iso_accuracy.h"

#include <string>
#include <charconv>
#include <stdexcept>
#include <string_view>

namespace cartan::bench::detail
{

inline int to_count(std::string_view text, int least)
{
    int value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value < least)
    {
        throw std::runtime_error(std::string{text} + ": not a count of at least "
            + std::to_string(least));
    }
    return value;
}

inline std::string one_table(std::string_view value)
{
    if (value.size() != 1 || value.find_first_not_of("abc") != std::string_view::npos)
    {
        throw std::runtime_error(std::string{value}
            + ": a statistic spanning tables is not a comparison. The three periodic rules "
              "produce three experiments whose solves differ, so a run captures one table and "
              "the tables are published as separate artifacts");
    }
    return std::string{value};
}

inline std::string one_provenance(std::string_view value)
{
    if (value != "description" && value != "synthetic")
    {
        throw std::runtime_error(std::string{value}
            + ": limits are read from a description or invented here, and a row says which");
    }
    return std::string{value};
}

inline int one_budget_index(std::string_view value)
{
    const int index = to_count(value, 0);
    if (index >= k_budget_points)
    {
        throw std::runtime_error(
            std::string{value} + ": the ladder has " + std::to_string(k_budget_points) + " rungs");
    }
    return index;
}

inline double a_distance(std::string_view value)
{
    const double target = std::stod(std::string{value});
    if (!(target > 0.0))
    {
        throw std::runtime_error(std::string{value} + ": an accuracy target is a distance");
    }
    return target;
}

/// Only the three declared targets are accepted by a capture. A run at a fourth
/// would report an accuracy no calibration established.
inline double one_accuracy_target(std::string_view value)
{
    const double target = a_distance(value);
    if (!is_declared_target(target))
    {
        throw std::runtime_error(std::string{value}
            + ": the study calibrates at 1e-5, 1e-6 and 1e-7 metres, and a target with no "
              "calibrated tolerance behind it is a requested accuracy rather than an achieved one");
    }
    return target;
}

}

#endif
