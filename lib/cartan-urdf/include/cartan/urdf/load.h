#ifndef HPP_GUARD_CARTAN_URDF_LOAD_H
#define HPP_GUARD_CARTAN_URDF_LOAD_H

/// Entry point for a robot description that has already been read, expanded
/// and resolved. It performs no I/O; it pushes the model through the staging
/// sink and hands the result to the chain extractor.

#include "cartan/urdf/error.h"
#include "cartan/urdf/metadata.h"

#include "cartan/urdf/detail/model_sink.h"

#include "cartan/expected.h"

#include <meios/model/model.h>

#include <meios/sink/model_sink.h>

#include <meios/records/link.h>
#include <meios/records/joint.h>
#include <meios/records/material.h>
#include <meios/records/robot_info.h>

namespace cartan
{

static_assert(meios::model_sink<detail::model_sink<>>);

/// Build a kinematic chain from an already-evaluated robot description. The
/// push order reproduces the reader's own, which is the order its sink
/// protocol is specified in; the reader's emitter is not called, because it
/// is a private name of that library.
///
/// The result's diagnostics stay empty and its claims stay none: this entry
/// point performs no read, so it has nothing to report on and inventing
/// values would put the caller's own model behind assertions this function
/// never checked.
template <typename Scalar = double>
inline cartan::expected<urdf_load_result<Scalar>, urdf_error>
chain_from_model(const meios::model<>& robot, const load_options& opts = {})
{
    detail::model_sink<Scalar> sink(opts);
    meios::robot_info info;
    info.name = robot.name;
    sink.on_robot(info);
    for (const meios::material<double>& mat : robot.materials) { sink.on_material(mat); }
    for (const meios::link<double>& node : robot.links) { sink.on_link(node); }
    for (const meios::joint<double>& edge : robot.joints) { sink.on_joint(edge); }
    sink.finish();
    return sink.result();
}

}

#endif
