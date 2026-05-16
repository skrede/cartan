#ifndef HPP_GUARD_CARTAN_URDF_BUILD_H
#define HPP_GUARD_CARTAN_URDF_BUILD_H

/// Chain extractor: build a strictly-serial kinematic_chain from a parsed URDF.
///
/// build_chain walks the parsed_model from a unique root link (no incoming
/// joint after fixed-joint folding) to a unique leaf link (no outgoing
/// non-fixed joint) and emits one kinematic_chain entry per mobile joint.
/// Fixed joints between mobile joints are composed into the world-to-parent
/// accumulator and naturally fold into the next downstream screw_axis origin;
/// fixed joints trailing the last mobile joint fold into the home pose M.
/// Branched trees (more than one outgoing non-fixed joint from any link, or
/// more than one root) produce urdf_failure::branched_kinematic_tree.

#include "cartan/urdf/error.h"
#include "cartan/urdf/schema.h"
#include "cartan/urdf/metadata.h"

#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include "cartan/lie/se3.h"

#include "cartan/types.h"

#include <string>
#include <vector>
#include <limits>
#include <utility>
#include <expected>
#include <algorithm>
#include <unordered_map>

namespace cartan::urdf
{

namespace detail
{

/// Build the parent-link -> outgoing-joint-indices adjacency over parsed_model.
/// A link with multiple non-fixed outgoing joints is a branch point; the chain
/// extractor surfaces that via urdf_failure::branched_kinematic_tree.
template <typename Scalar>
[[nodiscard]] inline std::unordered_map<std::string, std::vector<std::size_t>>
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
[[nodiscard]] inline std::unordered_map<std::string, std::size_t>
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
[[nodiscard]] inline std::vector<std::string>
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

}

/// Build a strictly-serial kinematic_chain from a parsed_model.
///
/// Auto-detect path (opts.base_link and opts.tool_link both empty): the
/// extractor walks from the unique root link along the unique chain of
/// non-fixed outgoing joints (fixed joints fold into the accumulator). If
/// any walked link has more than one non-fixed outgoing joint, or if the
/// model has more than one root, the result is
/// urdf_failure::branched_kinematic_tree.
///
/// Override path: when opts.base_link or opts.tool_link is non-empty, each
/// named link must appear in parsed_model.links; missing names produce
/// urdf_failure::link_not_found. When base_link is set, the walk starts at
/// that link instead of the auto-detected root; when tool_link is set, the
/// walk stops once it reaches that link.
template <typename Scalar = double>
[[nodiscard]] std::expected<urdf_load_result<Scalar>, urdf_error>
build_chain(const parsed_model<Scalar>& model, const load_options& opts = {})
{
    // Validate overrides against the link set up front so the caller gets the
    // most specific error rather than a downstream branched_kinematic_tree.
    auto link_exists = [&](const std::string& name) {
        for (const auto& l : model.links) { if (l.name == name) { return true; } }
        return false;
    };
    if (!opts.base_link.empty() && !link_exists(opts.base_link))
    {
        return std::unexpected(urdf_error{
            .kind = urdf_failure::link_not_found,
            .detail = opts.base_link,
            .location = std::nullopt});
    }
    if (!opts.tool_link.empty() && !link_exists(opts.tool_link))
    {
        return std::unexpected(urdf_error{
            .kind = urdf_failure::link_not_found,
            .detail = opts.tool_link,
            .location = std::nullopt});
    }

    const auto outgoing = detail::build_outgoing(model);
    const auto incoming = detail::build_incoming(model);
    const auto roots = detail::collect_roots(model, incoming);

    std::string current;
    if (!opts.base_link.empty())
    {
        current = opts.base_link;
    }
    else if (roots.size() == 1)
    {
        current = roots[0];
    }
    else
    {
        std::string detail_msg = "branched tree; roots: [";
        for (std::size_t i = 0; i < roots.size(); ++i)
        {
            if (i > 0) { detail_msg += ", "; }
            detail_msg += roots[i];
        }
        detail_msg += "]";
        return std::unexpected(urdf_error{
            .kind = urdf_failure::branched_kinematic_tree,
            .detail = std::move(detail_msg),
            .location = std::nullopt});
    }

    // Walk root -> leaf accumulating screw axes and the world-to-current-link
    // transform. Fixed joints advance the accumulator without emitting a
    // chain entry; mobile joints emit a screw_axis whose origin is the
    // world-frame image of the joint axis at zero joint configuration.
    se3<Scalar> T_acc = se3<Scalar>::identity();
    std::vector<screw_axis<Scalar>> axes;
    std::vector<joint_limits<Scalar>> limits;
    std::vector<std::string> joint_names;
    std::vector<std::optional<Scalar>> velocity_max;
    std::vector<std::optional<Scalar>> effort_max;

    while (true)
    {
        if (!opts.tool_link.empty() && current == opts.tool_link)
        {
            break;
        }
        auto it = outgoing.find(current);
        if (it == outgoing.end())
        {
            // Leaf reached on the auto-detect path.
            break;
        }

        // Single-outgoing-joint paths and a single non-fixed outgoing joint
        // with any number of fixed siblings are extractable; multiple
        // non-fixed siblings make the tree branched.
        const auto& outs = it->second;
        std::size_t mobile_count = 0;
        std::size_t chosen = outs.size();
        for (std::size_t idx : outs)
        {
            if (model.joints[idx].kind != parsed_joint_kind::fixed)
            {
                ++mobile_count;
                chosen = idx;
            }
        }
        if (mobile_count > 1)
        {
            // Identify leaf links of the branch for the diagnostic.
            std::string detail_msg = "branched tree; non-fixed branches at link '"
                + current + "': [";
            bool first = true;
            for (std::size_t idx : outs)
            {
                if (model.joints[idx].kind == parsed_joint_kind::fixed) { continue; }
                if (!first) { detail_msg += ", "; }
                detail_msg += model.joints[idx].child_link;
                first = false;
            }
            detail_msg += "]";
            return std::unexpected(urdf_error{
                .kind = urdf_failure::branched_kinematic_tree,
                .detail = std::move(detail_msg),
                .location = std::nullopt});
        }
        if (mobile_count == 0)
        {
            // Only fixed children. If there is exactly one, fold it; otherwise
            // the branching is purely cosmetic (multiple tool plates on the
            // same parent link) and we treat it as branched as well, since
            // there is no canonical choice of which fixed branch to follow.
            if (outs.size() == 1)
            {
                chosen = outs[0];
                const auto& fixed_joint = model.joints[chosen];
                T_acc = T_acc * fixed_joint.origin;
                current = fixed_joint.child_link;
                continue;
            }
            std::string detail_msg = "branched tree; fixed-only branches at link '"
                + current + "': [";
            bool first = true;
            for (std::size_t idx : outs)
            {
                if (!first) { detail_msg += ", "; }
                detail_msg += model.joints[idx].child_link;
                first = false;
            }
            detail_msg += "]";
            return std::unexpected(urdf_error{
                .kind = urdf_failure::branched_kinematic_tree,
                .detail = std::move(detail_msg),
                .location = std::nullopt});
        }

        // mobile_count == 1: fold any fixed siblings into the accumulator
        // would mean ignoring tool branches, which is not what the user
        // expects. We require non-fixed-mobile uniqueness as the only
        // extractable path. Fixed siblings of a mobile joint are also a
        // branch; surface as branched_kinematic_tree.
        if (outs.size() > 1)
        {
            std::string detail_msg = "branched tree; mobile joint shares link '"
                + current + "' with siblings: [";
            bool first = true;
            for (std::size_t idx : outs)
            {
                if (idx == chosen) { continue; }
                if (!first) { detail_msg += ", "; }
                detail_msg += model.joints[idx].child_link;
                first = false;
            }
            detail_msg += "]";
            return std::unexpected(urdf_error{
                .kind = urdf_failure::branched_kinematic_tree,
                .detail = std::move(detail_msg),
                .location = std::nullopt});
        }

        const auto& j = model.joints[chosen];

        // World-frame joint pose at zero joint configuration:
        //   T_acc_after_joint = T_acc * j.origin
        // The joint axis is expressed in the joint frame; its world image is
        // R(T_acc * j.origin).act(axis). The point on the axis (origin of the
        // joint frame in world coords) is (T_acc * j.origin).translation().
        se3<Scalar> T_acc_after_joint = T_acc * j.origin;
        const vector3<Scalar> axis_world =
            T_acc_after_joint.rotation().act(j.axis.normalized());
        const vector3<Scalar> point_world = T_acc_after_joint.translation();

        if (j.kind == parsed_joint_kind::prismatic)
        {
            axes.push_back(screw_axis<Scalar>::prismatic(axis_world));
        }
        else
        {
            axes.push_back(screw_axis<Scalar>::revolute(axis_world, point_world));
        }

        joint_limits<Scalar> jl{};
        if (j.kind == parsed_joint_kind::continuous)
        {
            jl.position_min = -std::numeric_limits<Scalar>::infinity();
            jl.position_max = +std::numeric_limits<Scalar>::infinity();
        }
        else
        {
            jl.position_min = j.position_min.value_or(Scalar(0));
            jl.position_max = j.position_max.value_or(Scalar(0));
        }
        jl.velocity_max = j.velocity_max;
        jl.effort_max = j.effort_max;
        limits.push_back(jl);

        joint_names.push_back(j.name);
        velocity_max.push_back(j.velocity_max);
        effort_max.push_back(j.effort_max);

        T_acc = T_acc_after_joint;
        current = j.child_link;
    }

    // Populate metadata. base_link_name and tool_link_name reflect the walk's
    // endpoints (which echo the override values when supplied).
    urdf_metadata<Scalar> meta{};
    if (!opts.base_link.empty())
    {
        meta.base_link_name = opts.base_link;
    }
    else
    {
        meta.base_link_name = roots[0];
    }
    meta.tool_link_name = current;
    meta.joint_names = std::move(joint_names);
    meta.velocity_max = std::move(velocity_max);
    meta.effort_max = std::move(effort_max);

    for (const auto& link : model.links)
    {
        if (!link.inertial.has_value()) { continue; }
        link_inertial<Scalar> li{};
        li.link_name = link.name;
        li.mass = link.inertial->mass;
        li.com = link.inertial->com;
        li.inertia = link.inertial->inertia;
        meta.link_inertials.push_back(std::move(li));
    }

    urdf_load_result<Scalar> result{
        kinematic_chain<Scalar, dynamic>(T_acc, std::move(axes), std::move(limits)),
        std::move(meta)};
    return result;
}

}

#endif
