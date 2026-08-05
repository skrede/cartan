#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_TOLERANCE_POLICY_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_TOLERANCE_POLICY_H

/// @file tolerance_policy.h
/// @brief What each participant in one run is asked for, and on whose authority.
///
/// Under the budget mode every participant is asked for the same tolerance, and
/// the accuracy each one achieves is whatever that produces. Under the accuracy
/// mode each participant is asked for its own calibrated tolerance, so that the
/// accuracies achieved match instead. Those are the only two authorities: a
/// participant the calibration does not cover has no third one to fall back to,
/// and a run that gave it a default would publish a matched-accuracy table over
/// a solver whose accuracy was never matched.

#include "iso_accuracy.h"

#include <cmath>
#include <format>
#include <limits>
#include <string>
#include <vector>
#include <utility>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace cartan::bench
{

namespace detail
{

inline std::string closest_accuracy(const calibration_row& row)
{
    return row.n == 0 ? std::string{"nothing: no solve met its requested tolerance anywhere in "
                                    "the search interval"}
                      : achieved_field(row.achieved_median, row.n);
}

}

class tolerance_policy
{
public:
    explicit tolerance_policy(double shared)
        : m_shared(shared)
        , m_target(std::numeric_limits<double>::quiet_NaN())
        , m_include_unconverged(false)
        , m_table()
        , m_key()
        , m_robot()
    {
    }

    tolerance_policy(calibration_table table, std::string key, std::string robot, double target,
        bool include_unconverged)
        : m_shared(std::numeric_limits<double>::quiet_NaN())
        , m_target(target)
        , m_include_unconverged(include_unconverged)
        , m_table(std::move(table))
        , m_key(std::move(key))
        , m_robot(std::move(robot))
    {
    }

    bool iso_accuracy() const { return m_table.has_value(); }

    double tolerance_for(std::string_view solver) const
    {
        return iso_accuracy() ? calibration_for(solver).calibrated_tolerance : m_shared;
    }

    /// The target travels onto every row it produced, and beside it whether the
    /// calibration that produced the tolerance reached the target at all. A row
    /// carrying the target alone would read as a met one.
    accuracy_claim claim_for(std::string_view solver) const
    {
        if (!iso_accuracy())
        {
            return accuracy_claim{std::string{}, std::string{}};
        }
        return accuracy_claim{
            std::format("{:.17g}", m_target), calibration_for(solver).converged ? "1" : "0"};
    }

    std::vector<double> declared_targets() const
    {
        return iso_accuracy() ? std::vector<double>{m_target} : std::vector<double>{};
    }

private:
    double m_shared;
    double m_target;
    bool m_include_unconverged;
    std::optional<calibration_table> m_table;
    std::string m_key;
    std::string m_robot;

    const calibration_row& calibration_for(std::string_view solver) const
    {
        const auto& row = m_table->row_for(m_key, m_robot, solver, m_target);
        if (!row.converged && !m_include_unconverged)
        {
            throw std::runtime_error(std::string{solver}
                + ": its calibration at accuracy target " + std::format("{:g}", m_target)
                + " did not converge -- the closest accuracy it reached was "
                + detail::closest_accuracy(row)
                + " -- so this run would publish it at an accuracy it was never driven to. Pass "
                  "--include-unconverged to record its rows as not meeting the target");
        }
        return row;
    }
};

}

#endif
