#include "harness.h"

#include <cartan/urdf/build.h>
#include <cartan/urdf/schema.h>
#include <cartan/urdf/metadata.h>

#include <span>
#include <string>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace
{

using cartan::fuzzing::field_reader;

constexpr std::size_t k_link_budget = 12;

std::string link_name(std::size_t index)
{
    return "l" + std::to_string(index);
}

cartan::parsed_joint_kind decode_kind(std::uint8_t selector)
{
    switch (selector % 4)
    {
    case 0: return cartan::parsed_joint_kind::fixed;
    case 1: return cartan::parsed_joint_kind::revolute;
    case 2: return cartan::parsed_joint_kind::continuous;
    default: return cartan::parsed_joint_kind::prismatic;
    }
}

/// Origins stay at the identity. The builder's contract is a parser-validated
/// model, and the parser refuses a non-finite origin, so a synthesized
/// non-finite origin would only manufacture a finiteness failure inside the
/// chain constructor that no document can reach. The axis and the limit pair
/// are decoded raw because the builder guards those itself.
cartan::parsed_joint<double> decode_joint(
    field_reader& fields, std::size_t index, std::size_t links)
{
    cartan::parsed_joint<double> joint;
    joint.name = "j" + std::to_string(index);
    joint.kind = decode_kind(fields.byte());
    joint.parent_link = link_name(fields.byte() % links);
    joint.child_link = link_name(fields.byte() % links);
    joint.axis = cartan::vector3<double>(
        fields.scalar(), fields.scalar(), fields.scalar());
    joint.position_min = fields.scalar();
    joint.position_max = fields.scalar();
    const std::uint8_t engaged = fields.byte();
    const double velocity = fields.scalar();
    const double effort = fields.scalar();
    if ((engaged & 1) != 0) { joint.velocity_max = velocity; }
    if ((engaged & 2) != 0) { joint.effort_max = effort; }
    return joint;
}

cartan::parsed_model<double> decode_model(field_reader& fields)
{
    cartan::parsed_model<double> model;
    model.robot_name = "r";
    const std::size_t links = 1 + fields.byte() % k_link_budget;
    for (std::size_t i = 0; i < links; ++i)
    {
        model.links.push_back(cartan::parsed_link<double>{link_name(i), std::nullopt});
    }
    const std::size_t joints = fields.byte() % (links + 2);
    for (std::size_t i = 0; i < joints; ++i)
    {
        model.joints.push_back(decode_joint(fields, i, links));
    }
    return model;
}

/// One index past the link set, so an override naming a link that does not
/// exist is reachable and the not-found path is fuzzed alongside the walk.
cartan::load_options decode_options(field_reader& fields, std::size_t links)
{
    cartan::load_options options;
    const std::uint8_t selector = fields.byte();
    const std::size_t base = fields.byte() % (links + 1);
    const std::size_t tool = fields.byte() % (links + 1);
    if ((selector & 1) != 0) { options.base_link = link_name(base); }
    if ((selector & 2) != 0) { options.tool_link = link_name(tool); }
    return options;
}

}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    field_reader fields{std::span<const std::uint8_t>(data, size)};
    const cartan::parsed_model<double> model = decode_model(fields);
    const cartan::load_options options = decode_options(fields, model.links.size());
    auto built = cartan::build_chain<double>(model, options);
    if (!built.has_value())
    {
        return 0;
    }
    cartan::fuzzing::consume(built.value().chain);
    return 0;
}
