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
#include "cartan/expected.h"
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
[[nodiscard]] cartan::expected<urdf_load_result<Scalar>, urdf_error>
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
        return cartan::unexpected(urdf_error{
            .kind = urdf_failure::link_not_found,
            .detail = opts.base_link,
            .location = std::nullopt});
    }
    if (!opts.tool_link.empty() && !link_exists(opts.tool_link))
    {
        return cartan::unexpected(urdf_error{
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
        return cartan::unexpected(urdf_error{
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

    // Helper: a "leaf" outgoing joint is a fixed joint whose child link has
    // no further outgoing joints. ROS-Industrial URDFs commonly attach such
    // joints to the chain root as world-frame attachment markers (the `base`
    // link sibling of `base_link_inertia` on Universal Robots arms is a
    // canonical example). They carry no kinematic content for a serial chain
    // and can be elided from the walk.
    auto is_leaf_outgoing = [&](std::size_t idx) {
        if (model.joints[idx].kind != parsed_joint_kind::fixed) { return false; }
        auto child_it = outgoing.find(model.joints[idx].child_link);
        return child_it == outgoing.end() || child_it->second.empty();
    };

    // A "self-collision wrapper" leaf is a fixed leaf whose child link's name
    // matches the convention `{parent_link_name}_sc` (the parent link is the
    // current walk node). The Franka Panda URDF attaches such wrappers to
    // every chain link to carry separate self-collision sphere/capsule meshes;
    // they are kinematic dead weight and must not compete with the genuine
    // tool-offset leaf when the walk reaches the final mobile joint. The rule
    // is structural rather than name-pattern-only: it triggers only when the
    // suffix `_sc` is appended verbatim to the parent link name, so unrelated
    // links whose names happen to contain `_sc` are unaffected.
    auto is_self_collision_wrapper_leaf =
        [&](std::size_t idx, const std::string& parent_link) {
            if (!is_leaf_outgoing(idx)) { return false; }
            const std::string& child = model.joints[idx].child_link;
            const std::string expected = parent_link + "_sc";
            return child == expected;
        };

    // Bound the walk against a malicious or malformed tree. A strictly-serial
    // walk visits each joint at most once, so it can take no more than
    // model.joints.size() productive steps; one extra step lets a legitimate
    // trailing tool-offset fold terminate cleanly. Exceeding the bound means
    // the walk is revisiting a link, i.e. the input encodes a cycle.
    const std::size_t max_steps = model.joints.size() + 1;
    std::size_t steps = 0;

    while (true)
    {
        if (steps++ > max_steps)
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::cyclic_kinematic_tree,
                .detail = "kinematic cycle detected while walking through link '"
                    + current + "'",
                .location = std::nullopt});
        }
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

        // Classify outgoing joints. A "chain-continuing" child is either a
        // mobile joint or a fixed joint whose child link itself has further
        // outgoing joints (a "pass-through" fixed joint). The walk is
        // well-defined when there is exactly one chain-continuing child;
        // fixed-leaf siblings attach world-side reference frames and are
        // silently skipped. When no chain-continuing child exists and there
        // is exactly one fixed-leaf child, fold it into T_acc as the trailing
        // tool-offset pattern (e.g. wrist_3 -> flange -> tool0 in the ROS
        // canonical UR URDFs).
        const auto& outs = it->second;
        std::size_t chain_continuing_count = 0;
        std::size_t chosen = outs.size();
        for (std::size_t idx : outs)
        {
            if (is_leaf_outgoing(idx)) { continue; }
            ++chain_continuing_count;
            chosen = idx;
        }
        if (chain_continuing_count == 0)
        {
            // All outgoing joints are fixed leaves. A single leaf is treated
            // as a trailing tool offset and folded into the accumulator;
            // when multiple leaves are present, partition them into
            // self-collision wrappers and "other" leaves and fold an unique
            // "other" leaf as the trailing tool offset (the Franka Panda
            // attaches up to seven `_sc` wrappers alongside the actual
            // `panda_joint8` tool offset at link7). A genuine ambiguity (zero
            // or several non-wrapper leaves) terminates the walk at the
            // current link without folding any of them, leaving the caller's
            // explicit `tool_link` override as the disambiguation path.
            if (outs.size() == 1)
            {
                const auto& fixed_joint = model.joints[outs[0]];
                T_acc = T_acc * fixed_joint.origin;
                current = fixed_joint.child_link;
                continue;
            }
            std::size_t other_leaf_count = 0;
            std::size_t other_leaf_idx = outs.size();
            for (std::size_t idx : outs)
            {
                if (is_self_collision_wrapper_leaf(idx, current)) { continue; }
                ++other_leaf_count;
                other_leaf_idx = idx;
            }
            if (other_leaf_count == 1)
            {
                const auto& fixed_joint = model.joints[other_leaf_idx];
                T_acc = T_acc * fixed_joint.origin;
                current = fixed_joint.child_link;
                continue;
            }
            break;
        }
        if (chain_continuing_count > 1)
        {
            // Genuine branching: more than one outgoing joint leads to a
            // sub-tree that needs further traversal.
            std::string detail_msg = "branched tree; multiple chain-continuing"
                " branches at link '" + current + "': [";
            bool first = true;
            for (std::size_t idx : outs)
            {
                if (is_leaf_outgoing(idx)) { continue; }
                if (!first) { detail_msg += ", "; }
                detail_msg += model.joints[idx].child_link;
                first = false;
            }
            detail_msg += "]";
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::branched_kinematic_tree,
                .detail = std::move(detail_msg),
                .location = std::nullopt});
        }

        if (model.joints[chosen].kind == parsed_joint_kind::fixed)
        {
            const auto& fixed_joint = model.joints[chosen];
            T_acc = T_acc * fixed_joint.origin;
            current = fixed_joint.child_link;
            continue;
        }

        const auto& j = model.joints[chosen];

        // World-frame joint pose at zero joint configuration:
        //   T_acc_after_joint = T_acc * j.origin
        // The joint axis is expressed in the joint frame; its world image is
        // R(T_acc * j.origin).act(axis). The point on the axis (origin of the
        // joint frame in world coords) is (T_acc * j.origin).translation().
        se3<Scalar> T_acc_after_joint = T_acc * j.origin;
        // A zero-magnitude axis normalizes to NaN and would silently poison the
        // kinematics; reject it before normalization.
        if (!(j.axis.norm() > Scalar(0)))
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::zero_axis,
                .detail = "joint '" + j.name + "' has a zero-magnitude <axis>",
                .location = std::nullopt});
        }
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
            // Revolute and prismatic joints are bounded; a missing <limit
            // lower upper> is malformed (freezing it to [0,0] would silently
            // pin the joint). A continuous joint is the only unbounded case.
            if (!j.position_min.has_value() || !j.position_max.has_value())
            {
                return cartan::unexpected(urdf_error{
                    .kind = urdf_failure::missing_joint_limit,
                    .detail = "joint '" + j.name + "' is "
                        + (j.kind == parsed_joint_kind::prismatic
                               ? std::string("prismatic")
                               : std::string("revolute"))
                        + " but omits the required <limit lower upper>",
                    .location = std::nullopt});
            }
            jl.position_min = *j.position_min;
            jl.position_max = *j.position_max;
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

    // When the caller pinned a tool link, the walk must have terminated on it.
    // A leaf reached before the requested tool link means the tool link is off
    // the serial walk path (e.g. on a sibling branch) and cannot be honored.
    if (!opts.tool_link.empty() && current != opts.tool_link)
    {
        return cartan::unexpected(urdf_error{
            .kind = urdf_failure::tool_link_unreachable,
            .detail = "tool link '" + opts.tool_link
                + "' is not on the serial walk path (walk ended at '" + current + "')",
            .location = std::nullopt});
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
