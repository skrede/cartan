#ifndef HPP_GUARD_CARTAN_URDF_DETAIL_MODEL_SINK_H
#define HPP_GUARD_CARTAN_URDF_DETAIL_MODEL_SINK_H

/// Staging consumer for the description reader's model push.
///
/// The reader's sink protocol leaves the five methods' return types
/// unconstrained, so a refusal cannot be returned from one: the first is
/// latched and reported from finish(). finish() is also what builds the chain,
/// so after a push the sink holds either a chain or a typed error and no
/// half-built state is representable.

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

#include <string>
#include <optional>

namespace cartan::detail
{

template <typename Scalar = double>
class model_sink
{
public:
    explicit model_sink(const load_options& opts)
        : m_opts(opts)
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
        m_model.links.push_back(staged_link<Scalar>(node));
    }

    void on_joint(const meios::joint<double>& edge)
    {
        if (m_failure) { return; }
        const std::optional<parsed_joint_kind> kind = staged_kind(edge.kind);
        if (!kind)
        {
            m_failure = unsupported_kind_failure(edge);
            return;
        }
        m_model.joints.push_back(staged_joint<Scalar>(edge, *kind));
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

    const cartan::expected<urdf_load_result<Scalar>, urdf_error>& result() const
    {
        return m_result;
    }

private:
    load_options m_opts;
    parsed_model<Scalar> m_model;
    std::optional<urdf_error> m_failure;
    cartan::expected<urdf_load_result<Scalar>, urdf_error> m_result;

    static urdf_error unsupported_kind_failure(const meios::joint<double>& edge)
    {
        std::optional<urdf_source_location> at;
        if (edge.origin_loc)
        {
            at = location_of(*edge.origin_loc, "joint");
        }
        return urdf_error{
            .kind = urdf_failure::unsupported_joint_type,
            .detail = "joint '" + edge.name + "' moves in a way a serial chain cannot carry",
            .location = at};
    }
};

}

#endif
