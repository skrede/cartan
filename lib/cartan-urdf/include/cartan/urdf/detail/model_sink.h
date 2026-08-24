#ifndef HPP_GUARD_CARTAN_URDF_DETAIL_MODEL_SINK_H
#define HPP_GUARD_CARTAN_URDF_DETAIL_MODEL_SINK_H

/// Staging consumer for the description reader's model push.
///
/// The reader's sink protocol leaves the five methods' return types
/// unconstrained, so a refusal cannot be returned from one: the first is
/// latched and reported from finish(). finish() is also what builds the chain,
/// so after a push the sink holds either a chain or a typed error and no
/// half-built state is representable.
///
/// The refusals here are the ones only this library can make. The reader
/// accepts a wider set of descriptions than a serial chain can carry, and
/// topology, duplicate names, limit ordering and axis magnitude are each
/// already answered elsewhere; a second answer to any of them is an answer that
/// can disagree with the first.

#include "cartan/urdf/detail/code_map.h"
#include "cartan/urdf/detail/model_stage.h"

#include "cartan/urdf/build.h"
#include "cartan/urdf/error.h"
#include "cartan/urdf/schema.h"
#include "cartan/urdf/metadata.h"

#include "cartan/expected.h"

#include <meios/records/link.h>
#include <meios/records/joint.h>
#include <meios/records/material.h>
#include <meios/records/robot_info.h>

#include <meios/diagnostic/source_location.h>

#include <string>
#include <utility>
#include <optional>
#include <string_view>

namespace cartan::detail
{

template <typename Scalar = double>
class model_sink
{
public:
    explicit model_sink(load_options opts)
        : m_opts(std::move(opts))
        , m_model()
        , m_failure()
        , m_result(cartan::unexpected(urdf_error{
              .kind = urdf_failure::unknown_error,
              .detail = "the robot description was never pushed into this sink",
              .location = std::nullopt}))
    {
    }

    void on_robot(const meios::robot_info& robot)
    {
        if (m_failure) { return; }
        m_model.robot_name = robot.name;
    }

    /// Materials carry no kinematics; the method exists because the reader's
    /// sink protocol names it.
    void on_material(const meios::material<double>&)
    {
    }

    void on_link(const meios::link<double>& node)
    {
        if (m_failure) { return; }
        parsed_link<Scalar> staged{};
        if (const auto field = stage_link(node, staged))
        {
            m_failure = overflow_failure("link", node.name, *field, node.origin_loc);
            return;
        }
        m_model.links.push_back(std::move(staged));
    }

    void on_joint(const meios::joint<double>& edge)
    {
        if (m_failure) { return; }
        const std::optional<parsed_joint_kind> kind = staged_kind(edge.kind);
        m_failure = unsupported_construct(edge, kind);
        if (m_failure) { return; }
        parsed_joint<Scalar> staged{};
        if (const auto field = stage_joint(edge, *kind, staged))
        {
            m_failure = overflow_failure("joint", edge.name, *field, edge.origin_loc);
            return;
        }
        m_model.joints.push_back(std::move(staged));
    }

    void finish()
    {
        if (m_failure)
        {
            m_result = cartan::unexpected(*m_failure);
            return;
        }
        m_result = build_chain<Scalar>(m_model, m_opts);
    }

    const parsed_model<Scalar>& staged() const
    {
        return m_model;
    }

    /// The latched staging refusal, separate from result(), which also carries
    /// the extractor's. A caller comparing what reaches the extractor needs to
    /// tell a document this sink refused from one the extractor refused.
    const std::optional<urdf_error>& failure() const
    {
        return m_failure;
    }

    const cartan::expected<urdf_load_result<Scalar>, urdf_error>& result() const
    {
        return m_result;
    }

private:
    load_options m_opts;
    parsed_model<Scalar> m_model;
    std::optional<urdf_error> m_failure;
    cartan::expected<urdf_load_result<Scalar>, urdf_error> m_result;

    /// The order is the contract: a joint that both declares a mimic relation
    /// and carries a kind no serial chain has is reported as the mimic.
    static std::optional<urdf_error> unsupported_construct(
        const meios::joint<double>& edge, const std::optional<parsed_joint_kind>& kind)
    {
        if (edge.couple)
        {
            return construct_failure(edge, urdf_failure::mimic_joint_unsupported,
                                     "declares a mimic relation");
        }
        if (!kind)
        {
            return construct_failure(edge, urdf_failure::unsupported_joint_type,
                                     "moves in a way a serial chain cannot carry");
        }
        return std::nullopt;
    }

    static urdf_error construct_failure(const meios::joint<double>& edge, urdf_failure kind,
                                        std::string_view what)
    {
        return urdf_error{
            .kind = kind,
            .detail = "joint '" + edge.name + "' " + std::string(what),
            .location = located(edge.origin_loc, "joint")};
    }

    static urdf_error overflow_failure(std::string_view element, const std::string& name,
                                       std::string_view field,
                                       const std::optional<meios::source_location>& at)
    {
        return urdf_error{
            .kind = urdf_failure::non_finite_value,
            .detail = std::string(element) + " '" + name + "' declares a " + std::string(field)
                + " the chain's scalar type cannot represent",
            .location = located(at, element)};
    }

    static std::optional<urdf_source_location> located(
        const std::optional<meios::source_location>& at, std::string_view element)
    {
        if (!at) { return std::nullopt; }
        return location_of(*at, element);
    }
};

}

#endif
