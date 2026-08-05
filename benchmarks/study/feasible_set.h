#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_FEASIBLE_SET_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_FEASIBLE_SET_H

/// @file feasible_set.h
/// @brief The one bounded problem a run constructs and hands to every solver.
///
/// The comparator takes its bounds as arrays rather than reading them off a
/// chain, so a harness that writes those arrays beside the chain is describing
/// one robot twice. The previous one did, and the two descriptions agreed only
/// because both were hard-coded to the same symmetric box. Here they are
/// derived from the chain in the member initialization list, and no
/// constructor, setter or factory accepts bounds.

#include "kdl_chain.h"
#include "description_chain.h"

#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <string>
#include <utility>
#include <stdexcept>
#include <string_view>

namespace cartan::bench
{

enum class periodic_rule
{
    unbounded,
    unbounded_solve_canonical_compare,
    canonical
};

enum class limits_provenance
{
    description,
    synthetic
};

struct comparator_bounds
{
    KDL::JntArray lower;
    KDL::JntArray upper;
};

namespace detail
{

template <int N>
comparator_bounds derive_comparator_bounds(const cartan::kinematic_chain<double, N>& chain)
{
    const auto width = ::cartan::detail::k_unbounded_angular_range_v<double>;
    comparator_bounds derived{KDL::JntArray(N), KDL::JntArray(N)};
    const auto& limits = chain.limits();
    for (unsigned int i = 0; i < static_cast<unsigned int>(N); ++i)
    {
        const auto anchored = ::cartan::detail::anchor_bounds(
            limits[i].position_min(), limits[i].position_max(), width, 0.0);
        derived.lower(i) = anchored.lower;
        derived.upper(i) = anchored.upper;
    }
    return derived;
}

}

template <int N>
class feasible_set
{
public:
    using chain_type = cartan::kinematic_chain<double, N>;
    using position_type = typename cartan::joint_state<double, N>::position_type;

    feasible_set(
        chain_type chain,
        KDL::Chain comparator,
        periodic_rule rule,
        limits_provenance provenance,
        std::string source)
        : m_chain(std::move(chain))
        , m_rule(rule)
        , m_source(std::move(source))
        , m_comparator(std::move(comparator))
        , m_bounds(detail::derive_comparator_bounds<N>(m_chain))
        , m_provenance(provenance)
    {
    }

    const chain_type& chain() const { return m_chain; }

    periodic_rule rule() const { return m_rule; }

    const std::string& source() const { return m_source; }

    const KDL::Chain& comparator() const { return m_comparator; }

    const comparator_bounds& bounds() const { return m_bounds; }

    limits_provenance provenance() const { return m_provenance; }

private:
    chain_type m_chain;
    periodic_rule m_rule;
    std::string m_source;
    KDL::Chain m_comparator;
    comparator_bounds m_bounds;
    limits_provenance m_provenance;
};

inline const description_spec& description_for(std::string_view robot)
{
    const description_spec* found = nullptr;
    for (const auto& spec : description_specs())
    {
        if (spec.robot_key == robot || spec.robot_key.ends_with(robot))
        {
            if (found != nullptr)
            {
                throw std::runtime_error(std::string{robot} + ": names more than one description");
            }
            found = &spec;
        }
    }
    if (found == nullptr)
    {
        throw std::runtime_error(std::string{robot} + ": the study carries no such description");
    }
    return *found;
}

inline periodic_rule rule_for_table(std::string_view table)
{
    if (table == "c")
    {
        return periodic_rule::canonical;
    }
    throw std::runtime_error(
        std::string{table} + ": only the canonical-throughout table is measured here");
}

/// The one joint count this spine measures.
constexpr int study_joints = 6;

/// Both the comparator's geometry and its bounds come from the chain the
/// description produced. Nothing here names a second description of the robot.
template <int N>
feasible_set<N> load_feasible_set(const description_spec& spec, periodic_rule rule)
{
    auto loaded = chain_from_description<N>(spec);
    auto comparator = build_kdl_chain<N>(loaded);
    return feasible_set<N>(
        std::move(loaded),
        std::move(comparator),
        rule,
        limits_provenance::description,
        std::string{spec.relative_path});
}

}

#endif
