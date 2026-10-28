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
///
/// The periodic rule is applied in exactly two places: the solve chain built
/// here, and the canonicalization step inside the adjudication. Two application
/// sites would be two rules.

#include "kdl_chain.h"
#include "rule_chain.h"
#include "description_chain.h"

#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <string>
#include <vector>
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

namespace detail
{

template <int N>
cartan::kinematic_chain<double, N> solve_chain_for(
    const cartan::kinematic_chain<double, N>& declared,
    periodic_rule rule,
    const std::vector<int>& qualifying)
{
    return rule == periodic_rule::canonical ? declared : relax_joints<N>(declared, qualifying);
}

}

template <int N>
class feasible_set
{
public:
    using chain_type = cartan::kinematic_chain<double, N>;
    using position_type = typename cartan::joint_state<double, N>::position_type;

    feasible_set(
        chain_type declared,
        KDL::Chain comparator,
        periodic_rule rule,
        limits_provenance provenance,
        std::string source)
        : m_declared(std::move(declared))
        , m_qualifying(full_turn_joints<N>(m_declared))
        , m_chain(detail::solve_chain_for<N>(m_declared, rule, m_qualifying))
        , m_rule(rule)
        , m_source(std::move(source))
        , m_comparator(std::move(comparator))
        , m_bounds(derive_comparator_bounds<N>(m_chain))
        , m_provenance(provenance)
    {
    }

    /// What every participant solves against.
    const chain_type& chain() const { return m_chain; }

    /// The description's own bounds, which every target is drawn from and which
    /// two of the three rules verify against.
    const chain_type& declared() const { return m_declared; }

    const chain_type& verification_chain() const
    {
        return m_rule == periodic_rule::unbounded ? m_chain : m_declared;
    }

    periodic_rule rule() const { return m_rule; }

    const std::string& source() const { return m_source; }

    const KDL::Chain& comparator() const { return m_comparator; }

    const comparator_bounds& bounds() const { return m_bounds; }

    limits_provenance provenance() const { return m_provenance; }

    /// Empty where the three rules coincide, which is a publishable finding
    /// about the robot rather than a defect in the tables that agree.
    const std::vector<int>& qualifying_joints() const { return m_qualifying; }

private:
    chain_type m_declared;
    std::vector<int> m_qualifying;
    chain_type m_chain;
    periodic_rule m_rule;
    std::string m_source;
    KDL::Chain m_comparator;
    comparator_bounds m_bounds;
    limits_provenance m_provenance;
};

inline std::string_view table_key(periodic_rule rule)
{
    switch (rule)
    {
        case periodic_rule::unbounded: return "a";
        case periodic_rule::unbounded_solve_canonical_compare: return "b";
        case periodic_rule::canonical: return "c";
    }
    throw std::runtime_error("the periodic rule enumeration gained a value with no table");
}

inline std::string_view rule_name(periodic_rule rule)
{
    switch (rule)
    {
        case periodic_rule::unbounded: return "unbounded";
        case periodic_rule::unbounded_solve_canonical_compare:
            return "unbounded_solve_canonical_compare";
        case periodic_rule::canonical: return "canonical";
    }
    throw std::runtime_error("the periodic rule enumeration gained a value with no name");
}

inline periodic_rule rule_from_name(std::string_view name)
{
    for (const auto rule : {periodic_rule::unbounded,
             periodic_rule::unbounded_solve_canonical_compare, periodic_rule::canonical})
    {
        if (rule_name(rule) == name)
        {
            return rule;
        }
    }
    throw std::runtime_error(std::string{name} + ": the study measures no such periodic rule");
}

/// Whether the rule leaves the description's declared bounds standing. A row
/// about adjacency to a bound the rule then relaxes is a claim about two
/// different bounds at once.
inline bool rule_preserves_declared_bounds(periodic_rule rule)
{
    return rule == periodic_rule::canonical;
}

inline periodic_rule rule_for_table(std::string_view table)
{
    for (const auto rule : {periodic_rule::unbounded,
             periodic_rule::unbounded_solve_canonical_compare, periodic_rule::canonical})
    {
        if (table_key(rule) == table)
        {
            return rule;
        }
    }
    throw std::runtime_error(std::string{table} + ": the study publishes tables a, b and c");
}

/// Both the comparator's geometry and its bounds come from the chain the
/// description produced. Nothing here names a second description of the robot.
template <int N>
feasible_set<N> load_feasible_set(
    const description_spec& spec, periodic_rule rule, limits_provenance provenance)
{
    auto loaded = chain_from_description<N>(spec);
    auto declared = provenance == limits_provenance::synthetic
        ? symmetric_box<N>(loaded)
        : std::move(loaded);
    auto comparator = build_kdl_chain<N>(declared);
    return feasible_set<N>(std::move(declared), std::move(comparator), rule, provenance,
        std::string{spec.relative_path});
}

}

#endif
