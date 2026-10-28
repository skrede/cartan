#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_RULE_CHAIN_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_RULE_CHAIN_H

/// @file rule_chain.h
/// @brief The two rewrites a run may apply to a description's declared bounds.
///
/// A joint whose declared range spans a full turn reaches every orientation its
/// axis can reach, so treating it as unbounded is a defensible modelling choice
/// -- and the previously published study made that choice for every joint of
/// every robot without recording it. Here the rewrite is named, the joints it
/// applies to are listed, and the run says which of the two it performed.

#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <cmath>
#include <vector>
#include <limits>
#include <string>
#include <cstddef>
#include <numbers>
#include <stdexcept>

namespace cartan::bench
{

constexpr double k_full_turn = 2.0 * std::numbers::pi;

/// The joints on which the three periodic rules can disagree: a joint that
/// already declares no bound is unbounded under every rule, so it is not one of
/// them however continuous it is.
template <int N>
std::vector<int> full_turn_joints(const cartan::kinematic_chain<double, N>& chain)
{
    std::vector<int> qualifying;
    for (int i = 0; i < N; ++i)
    {
        const auto& limit = chain.limits()[static_cast<std::size_t>(i)];
        const double range = limit.position_max() - limit.position_min();
        if (std::isfinite(range) && range >= k_full_turn)
        {
            qualifying.push_back(i);
        }
    }
    return qualifying;
}

namespace detail
{

inline cartan::joint_limits<double> rebound(
    const cartan::joint_limits<double>& limit, double lower, double upper)
{
    auto made = cartan::joint_limits<double>::make(
        lower, upper, limit.velocity_max(), limit.effort_max(), limit.acceleration_max());
    if (!made)
    {
        throw std::runtime_error("the substituted joint bounds were refused by the library");
    }
    return *made;
}

}

template <int N>
cartan::kinematic_chain<double, N> relax_joints(
    const cartan::kinematic_chain<double, N>& chain, const std::vector<int>& joints)
{
    auto limits = chain.limits();
    const double unbounded = std::numeric_limits<double>::infinity();
    for (const int joint : joints)
    {
        const auto index = static_cast<std::size_t>(joint);
        limits[index] = detail::rebound(limits[index], -unbounded, unbounded);
    }
    return cartan::kinematic_chain<double, N>(chain.home(), chain.axes(), limits);
}

/// The symmetric box the previously published comparison ran every cartan cell
/// against. It is kept as an opt-in arm rather than deleted, because how much
/// an invented bound changed the answer is a measurable quantity and deleting
/// the arm makes it unmeasurable. Every row it produces is labeled synthetic.
template <int N>
cartan::kinematic_chain<double, N> symmetric_box(
    const cartan::kinematic_chain<double, N>& chain)
{
    auto limits = chain.limits();
    for (std::size_t i = 0; i < limits.size(); ++i)
    {
        limits[i] = detail::rebound(limits[i], -std::numbers::pi, std::numbers::pi);
    }
    return cartan::kinematic_chain<double, N>(chain.home(), chain.axes(), limits);
}

}

#endif
