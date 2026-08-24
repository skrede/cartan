#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_ARGMIN_CONVERGENCE_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_ARGMIN_CONVERGENCE_H

/// The width of the per-criterion telemetry a convergence policy reports, for
/// the solve policies that expose that telemetry to their callers.

#include <argmin/solver/convergence.h>

#include <array>
#include <utility>
#include <optional>
#include <type_traits>
#include <cstddef>

namespace cartan::detail
{

/// argmin sizes the telemetry array by criteria count -- four for the default
/// policy, three for the SLSQP-compatible one -- so an accessor that returns it
/// must follow the policy rather than fix a width. A policy predating the
/// telemetry reports none, and the default width stands in for the empty answer.
template <typename Convergence>
constexpr std::size_t convergence_check_count()
{
    if constexpr (requires(const Convergence& c) { c.last_check_results(); })
    {
        return std::tuple_size_v<std::remove_cvref_t<
            decltype(std::declval<const Convergence&>().last_check_results())>>;
    }
    else
    {
        return 4;
    }
}

template <typename Convergence>
using convergence_check_results
    = std::array<std::optional<argmin::solver_status>, convergence_check_count<Convergence>()>;

}

#endif
