#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_CALIBRATION_PARSE_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_CALIBRATION_PARSE_H

/// @file calibration_parse.h
/// @brief Reading one row of a calibration table back off disk.
///
/// The file this reads determines the tolerance every participant of an
/// accuracy-mode run is asked for, so a line short of its nine fields is
/// refused by number rather than filled in: a row read as something other than
/// what was written would drive a solver at a tolerance nobody measured.

#include "calibration_row.h"

#include <cmath>
#include <string>
#include <vector>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace cartan::bench::detail
{

inline bool same_target(double left, double right)
{
    return std::abs(left - right) <= 1e-9 * std::abs(right);
}

inline std::vector<std::string> split_fields(const std::string& line)
{
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ','))
    {
        fields.push_back(field);
    }
    return fields;
}

/// A line written on one platform and read on another arrives with the carriage
/// return still attached, and a header compared against it would never match.
inline std::string without_return(std::string line)
{
    if (!line.empty() && line.back() == '\r')
    {
        line.pop_back();
    }
    return line;
}

inline double to_achieved(const std::string& text)
{
    return text.empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(text);
}

inline calibration_row parse_calibration_row(const std::string& line, int number)
{
    const auto fields = split_fields(line);
    if (fields.size() != 9)
    {
        throw std::runtime_error("line " + std::to_string(number) + " carries "
            + std::to_string(fields.size()) + " fields against the nine a calibration row has");
    }
    return calibration_row{fields[0], fields[1], fields[2], std::stod(fields[3]),
        std::stod(fields[4]), to_achieved(fields[5]), to_achieved(fields[6]),
        std::stoi(fields[7]), fields[8] == "1"};
}

}

#endif
