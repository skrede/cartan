#ifndef HPP_GUARD_CARTAN_URDF_DETAIL_TOPOLOGY_H
#define HPP_GUARD_CARTAN_URDF_DETAIL_TOPOLOGY_H

/// The index maps a parsed model's joints induce over its links, and the
/// refusal that reports a shape those maps show is not one serial chain.
///
/// The maps are built once and consumed in sequence by the walk; they are
/// separate from it because they describe the document's graph, which is
/// settled before the first step is taken.

#include "cartan/urdf/error.h"
#include "cartan/urdf/schema.h"

#include <string>
#include <vector>
#include <cstddef>
#include <unordered_map>

namespace cartan::detail
{

/// Build the parent-link -> outgoing-joint-indices adjacency over parsed_model.
/// A link with multiple non-fixed outgoing joints is a branch point; the chain
/// extractor surfaces that via urdf_failure::branched_kinematic_tree.
template <typename Scalar>
inline std::unordered_map<std::string, std::vector<std::size_t>>
build_outgoing(const parsed_model<Scalar>& model)
{
    std::unordered_map<std::string, std::vector<std::size_t>> out;
    for (std::size_t i = 0; i < model.joints.size(); ++i)
    {
        out[model.joints[i].parent_link].push_back(i);
    }
    return out;
}

/// Build the child-link -> parent-joint-index reverse map. Used to identify
/// root links (those with no incoming joint).
template <typename Scalar>
inline std::unordered_map<std::string, std::size_t>
build_incoming(const parsed_model<Scalar>& model)
{
    std::unordered_map<std::string, std::size_t> in;
    for (std::size_t i = 0; i < model.joints.size(); ++i)
    {
        in.emplace(model.joints[i].child_link, i);
    }
    return in;
}

/// Identify the root links (no incoming joint) in source order.
template <typename Scalar>
inline std::vector<std::string>
collect_roots(const parsed_model<Scalar>& model,
              const std::unordered_map<std::string, std::size_t>& incoming)
{
    std::vector<std::string> roots;
    for (const auto& link : model.links)
    {
        if (!incoming.contains(link.name))
        {
            roots.push_back(link.name);
        }
    }
    return roots;
}

/// A branched refusal names the links the caller has to choose between; one
/// that only reports that the tree branches costs the caller the same search
/// as no refusal at all.
inline urdf_error branched_failure(const std::string& what,
                                   const std::vector<std::string>& names)
{
    std::string detail_msg = "branched tree; " + what + ": [";
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        if (i > 0) { detail_msg += ", "; }
        detail_msg += names[i];
    }
    detail_msg += "]";
    return urdf_error{
        .kind = urdf_failure::branched_kinematic_tree,
        .detail = std::move(detail_msg),
        .location = std::nullopt};
}

}

#endif
