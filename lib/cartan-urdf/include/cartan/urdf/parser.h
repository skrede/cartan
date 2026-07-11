#ifndef HPP_GUARD_CARTAN_URDF_PARSER_H
#define HPP_GUARD_CARTAN_URDF_PARSER_H

/// Top-level URDF parser entry point.
///
/// parse_urdf_file reads a URDF document from disk, validates well-formedness
/// of every link and joint it encounters, and produces a parsed_model that
/// preserves the full link and joint tree. Connectivity (root and leaf
/// detection, fixed-joint merging, chain extraction) is performed by a
/// separate step downstream; the parser's responsibility ends at structurally
/// valid per-element parsing. The parser is a one-shot startup path; cost of
/// the cartan::expected return channel and the diagnostic strings is acceptable
/// because URDF loading is not on any hot path.

#include "cartan/urdf/error.h"
#include "cartan/urdf/schema.h"

#include "cartan/lie/se3.h"
#include "cartan/lie/so3.h"

#include "cartan/types.h"
#include "cartan/expected.h"

#include <pugixml.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <string_view>
#include <system_error>
#include <unordered_set>

namespace cartan
{

namespace detail
{

/// Convert a zero-based byte offset within the file text to a one-based line
/// number. Used to translate pugixml's xml_parse_result::offset into the
/// urdf_source_location reported on parse failures.
inline int offset_to_line(const std::string& text, std::ptrdiff_t offset)
{
    int line = 1;
    const auto cap = static_cast<std::ptrdiff_t>(text.size());
    const auto stop = offset < cap ? offset : cap;
    for (std::ptrdiff_t i = 0; i < stop; ++i)
    {
        if (text[static_cast<std::size_t>(i)] == '\n')
        {
            ++line;
        }
    }
    return line;
}

/// Read the entire file into a string. Returns an empty string on failure;
/// callers should not rely on this value being meaningful beyond line-number
/// tracking on a separate pugixml load_file call.
inline std::string slurp_file(const std::filesystem::path& path)
{
    std::error_code ec;
    auto size = std::filesystem::file_size(path, ec);
    if (ec)
    {
        return {};
    }
    // std::ifstream takes the path directly, so wide-char paths on Windows are
    // handled without the narrowing that std::fopen(path.c_str()) would force.
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        return {};
    }
    std::string text;
    text.resize(static_cast<std::size_t>(size));
    in.read(text.data(), static_cast<std::streamsize>(text.size()));
    text.resize(static_cast<std::size_t>(in.gcount()));
    return text;
}

/// Read a numeric XML attribute, rejecting non-finite (NaN or infinity)
/// values. The strtod-backed pugixml reader accepts "nan"/"inf" tokens, so
/// this is the single choke point every numeric attribute passes through; an
/// empty optional signals the caller to raise urdf_failure::non_finite_value
/// naming the offending element.
inline std::optional<double> as_finite_double(const pugi::xml_attribute& attr)
{
    const double raw = attr.as_double();
    if (!std::isfinite(raw))
    {
        return std::nullopt;
    }
    return raw;
}

/// Consume and return the next whitespace-delimited token from s, advancing s
/// past it. Returns an empty view when only whitespace remains.
inline std::string_view next_field(std::string_view& s)
{
    constexpr std::string_view ws = " \t\r\n\f\v";
    const std::size_t begin = s.find_first_not_of(ws);
    if (begin == std::string_view::npos)
    {
        s = {};
        return {};
    }
    s.remove_prefix(begin);
    const std::size_t end = s.find_first_of(ws);
    const std::string_view token = s.substr(0, end);
    s.remove_prefix(end == std::string_view::npos ? s.size() : end);
    return token;
}

/// Parse a whitespace-separated triple ("x y z") into a vector3. strtod keeps
/// parsing off the std::num_get facet, which crashes when a static-libstdc++
/// wheel and numpy pull in two libstdc++ runtimes, and matches the strtod-backed
/// pugixml attribute reader the rest of this parser relies on (both require a
/// "C" numeric locale, as ROS does). std::from_chars would be locale-free but
/// its floating-point overload is unavailable below a very recent macOS SDK.
/// Returns false on any field that fails to parse, is non-finite, or leaves
/// trailing characters, and on a count other than three; out is written only on
/// full success.
template <typename Scalar>
bool parse_triple(std::string_view s, vector3<Scalar>& out)
{
    Scalar values[3];
    for (std::size_t i = 0; i < 3; ++i)
    {
        const std::string_view field = next_field(s);
        if (field.empty())
        {
            return false;
        }
        const std::string token(field);
        char* end = nullptr;
        const double parsed = std::strtod(token.c_str(), &end);
        if (end != token.c_str() + token.size() || !std::isfinite(parsed))
        {
            return false;
        }
        values[i] = static_cast<Scalar>(parsed);
    }
    if (!next_field(s).empty())
    {
        return false;
    }
    out << values[0], values[1], values[2];
    return true;
}

/// Build an SO(3) rotation from URDF roll-pitch-yaw angles. URDF convention:
/// R = Rz(yaw) * Ry(pitch) * Rx(roll). Each axis rotation is built via
/// so3::exp so the implementation reuses the validated cartan exponential map.
template <typename Scalar>
so3<Scalar> rotation_from_rpy(Scalar roll, Scalar pitch, Scalar yaw)
{
    auto rx = so3<Scalar>::exp(vector3<Scalar>(roll, Scalar(0), Scalar(0)));
    auto ry = so3<Scalar>::exp(vector3<Scalar>(Scalar(0), pitch, Scalar(0)));
    auto rz = so3<Scalar>::exp(vector3<Scalar>(Scalar(0), Scalar(0), yaw));
    return rz * ry * rx;
}

/// Parse a URDF <origin xyz="..." rpy="..."/> sub-element into an se3 pose.
/// Missing attributes default to zero (URDF spec). Returns std::nullopt only
/// on malformed numeric content; absence of the <origin> element entirely is
/// the caller's concern.
template <typename Scalar>
std::optional<se3<Scalar>> parse_origin(const pugi::xml_node& origin)
{
    vector3<Scalar> xyz = vector3<Scalar>::Zero();
    vector3<Scalar> rpy = vector3<Scalar>::Zero();
    if (auto attr = origin.attribute("xyz"); attr)
    {
        if (!parse_triple<Scalar>(attr.value(), xyz))
        {
            return std::nullopt;
        }
    }
    if (auto attr = origin.attribute("rpy"); attr)
    {
        if (!parse_triple<Scalar>(attr.value(), rpy))
        {
            return std::nullopt;
        }
    }
    return se3<Scalar>(rotation_from_rpy<Scalar>(rpy(0), rpy(1), rpy(2)), xyz);
}

/// Map a URDF joint type token to the parser's enumeration. Returns
/// std::nullopt when the token is not one of the supported kinds; callers
/// translate that into urdf_failure::unsupported_joint_type.
inline std::optional<parsed_joint_kind> joint_kind_from_token(std::string_view token)
{
    if (token == "fixed") { return parsed_joint_kind::fixed; }
    if (token == "revolute") { return parsed_joint_kind::revolute; }
    if (token == "continuous") { return parsed_joint_kind::continuous; }
    if (token == "prismatic") { return parsed_joint_kind::prismatic; }
    return std::nullopt;
}

/// Parse an <inertial> sub-element into a parsed_inertial. Rejects
/// non-positive mass and negative diagonal entries in the inertia matrix
/// (cheap necessary condition for positive semidefiniteness).
template <typename Scalar>
cartan::expected<parsed_inertial<Scalar>, urdf_error>
parse_inertial(const pugi::xml_node& inertial_node,
               const std::string& link_name,
               const std::string& file_path)
{
    parsed_inertial<Scalar> out{};
    out.mass = Scalar(0);
    out.com = vector3<Scalar>::Zero();
    out.inertia = matrix3<Scalar>::Zero();

    auto mass_node = inertial_node.child("mass");
    if (!mass_node)
    {
        return cartan::unexpected(urdf_error{
            .kind = urdf_failure::inertial_singular,
            .detail = "link '" + link_name + "': <inertial> missing <mass> child",
            .location = urdf_source_location{file_path, 0, "inertial"}});
    }
    auto mass_val = as_finite_double(mass_node.attribute("value"));
    if (!mass_val.has_value())
    {
        return cartan::unexpected(urdf_error{
            .kind = urdf_failure::non_finite_value,
            .detail = "link '" + link_name + "': inertial mass is not finite",
            .location = urdf_source_location{file_path, 0, "inertial"}});
    }
    out.mass = static_cast<Scalar>(*mass_val);
    if (!(out.mass > Scalar(0)))
    {
        return cartan::unexpected(urdf_error{
            .kind = urdf_failure::inertial_singular,
            .detail = "link '" + link_name + "': inertial mass must be positive, got "
                + std::to_string(static_cast<double>(out.mass)),
            .location = urdf_source_location{file_path, 0, "inertial"}});
    }

    if (auto origin = inertial_node.child("origin"); origin)
    {
        if (auto attr = origin.attribute("xyz"); attr)
        {
            if (!parse_triple<Scalar>(attr.value(), out.com))
            {
                return cartan::unexpected(urdf_error{
                    .kind = urdf_failure::inertial_singular,
                    .detail = "link '" + link_name + "': malformed inertial origin xyz",
                    .location = urdf_source_location{file_path, 0, "inertial"}});
            }
        }
    }

    auto inertia = inertial_node.child("inertia");
    if (!inertia)
    {
        return cartan::unexpected(urdf_error{
            .kind = urdf_failure::inertial_singular,
            .detail = "link '" + link_name + "': <inertial> missing <inertia> child",
            .location = urdf_source_location{file_path, 0, "inertial"}});
    }
    Scalar ixx{}, ixy{}, ixz{}, iyy{}, iyz{}, izz{};
    for (const auto& [name, slot] : {
             std::pair<const char*, Scalar*>{"ixx", &ixx},
             std::pair<const char*, Scalar*>{"ixy", &ixy},
             std::pair<const char*, Scalar*>{"ixz", &ixz},
             std::pair<const char*, Scalar*>{"iyy", &iyy},
             std::pair<const char*, Scalar*>{"iyz", &iyz},
             std::pair<const char*, Scalar*>{"izz", &izz}})
    {
        auto v = as_finite_double(inertia.attribute(name));
        if (!v.has_value())
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::non_finite_value,
                .detail = "link '" + link_name + "': inertia entry '" + name + "' is not finite",
                .location = urdf_source_location{file_path, 0, "inertial"}});
        }
        *slot = static_cast<Scalar>(*v);
    }
    if (ixx < Scalar(0) || iyy < Scalar(0) || izz < Scalar(0))
    {
        return cartan::unexpected(urdf_error{
            .kind = urdf_failure::inertial_singular,
            .detail = "link '" + link_name + "': inertia diagonal must be non-negative",
            .location = urdf_source_location{file_path, 0, "inertial"}});
    }
    out.inertia(0, 0) = ixx;
    out.inertia(0, 1) = ixy;
    out.inertia(0, 2) = ixz;
    out.inertia(1, 0) = ixy;
    out.inertia(1, 1) = iyy;
    out.inertia(1, 2) = iyz;
    out.inertia(2, 0) = ixz;
    out.inertia(2, 1) = iyz;
    out.inertia(2, 2) = izz;
    return out;
}

}

/// Parse a URDF document from disk into a parsed_model. The model is the
/// intermediate representation the chain extractor walks; this entry point
/// does not enforce connectivity or single-leaf invariants. Failure modes:
///   - urdf_failure::malformed_xml when pugixml rejects the document or the
///     <robot> root is missing;
///   - urdf_failure::unsupported_joint_type when a joint's type attribute is
///     not one of fixed, revolute, continuous, prismatic;
///   - urdf_failure::unknown_parent_link when a joint's parent link is not in
///     the link set;
///   - urdf_failure::mimic_joint_unsupported when a joint declares <mimic>;
///   - urdf_failure::inertial_singular when an <inertial> element declares a
///     non-positive mass or a negative diagonal inertia entry.
template <typename Scalar = double>
cartan::expected<parsed_model<Scalar>, urdf_error>
parse_urdf_file(const std::filesystem::path& path)
{
    const std::string path_str = path.string();
    const std::string file_text = detail::slurp_file(path);

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path.c_str(), pugi::parse_default, pugi::encoding_auto);
    if (!result)
    {
        return cartan::unexpected(urdf_error{
            .kind = urdf_failure::malformed_xml,
            .detail = std::string{result.description()},
            .location = urdf_source_location{
                path_str,
                detail::offset_to_line(file_text, static_cast<std::ptrdiff_t>(result.offset)),
                "robot"}});
    }

    pugi::xml_node robot = doc.child("robot");
    if (!robot)
    {
        return cartan::unexpected(urdf_error{
            .kind = urdf_failure::malformed_xml,
            .detail = "missing <robot> element",
            .location = urdf_source_location{path_str, 1, "robot"}});
    }

    parsed_model<Scalar> model{};
    model.robot_name = robot.attribute("name").as_string();

    // Pass 1: links (and inertials).
    std::unordered_set<std::string> link_names;
    for (pugi::xml_node link_node : robot.children("link"))
    {
        parsed_link<Scalar> link{};
        link.name = link_node.attribute("name").as_string();
        if (link.name.empty())
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::malformed_xml,
                .detail = "<link> missing required name attribute",
                .location = urdf_source_location{path_str, 0, "link"}});
        }
        if (!link_names.insert(link.name).second)
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::duplicate_name,
                .detail = "link '" + link.name + "' is declared more than once",
                .location = urdf_source_location{path_str, 0, "link"}});
        }
        if (auto inertial = link_node.child("inertial"); inertial)
        {
            auto parsed = detail::parse_inertial<Scalar>(inertial, link.name, path_str);
            if (!parsed.has_value())
            {
                return cartan::unexpected(std::move(parsed.error()));
            }
            link.inertial = std::move(parsed.value());
        }
        model.links.push_back(std::move(link));
    }

    // Pass 2: joints. Cross-references against link_names collected in pass 1.
    // joint_names guards against duplicate joint declarations; child_links
    // guards against a link that is the child of more than one joint (a
    // non-tree, multi-parent topology).
    std::unordered_set<std::string> joint_names;
    std::unordered_set<std::string> child_links;
    for (pugi::xml_node joint_node : robot.children("joint"))
    {
        parsed_joint<Scalar> joint{};
        joint.name = joint_node.attribute("name").as_string();
        if (joint.name.empty())
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::malformed_xml,
                .detail = "<joint> missing required name attribute",
                .location = urdf_source_location{path_str, 0, "joint"}});
        }
        if (!joint_names.insert(joint.name).second)
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::duplicate_name,
                .detail = "joint '" + joint.name + "' is declared more than once",
                .location = urdf_source_location{path_str, 0, "joint"}});
        }
        const std::string type_token = joint_node.attribute("type").as_string();
        auto kind_opt = detail::joint_kind_from_token(type_token);
        if (!kind_opt.has_value())
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::unsupported_joint_type,
                .detail = "joint '" + joint.name + "' has unsupported type '" + type_token + "'",
                .location = urdf_source_location{path_str, 0, "joint"}});
        }
        joint.kind = *kind_opt;

        if (joint_node.child("mimic"))
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::mimic_joint_unsupported,
                .detail = "joint '" + joint.name + "' declares <mimic>; coupled-joint kinematics is not supported",
                .location = urdf_source_location{path_str, 0, "joint"}});
        }

        pugi::xml_node parent = joint_node.child("parent");
        pugi::xml_node child = joint_node.child("child");
        if (!parent || !child)
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::malformed_xml,
                .detail = "joint '" + joint.name + "' missing <parent> or <child>",
                .location = urdf_source_location{path_str, 0, "joint"}});
        }
        joint.parent_link = parent.attribute("link").as_string();
        joint.child_link = child.attribute("link").as_string();
        if (!link_names.contains(joint.parent_link))
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::unknown_parent_link,
                .detail = "joint '" + joint.name + "' references unknown parent link '" + joint.parent_link + "'",
                .location = urdf_source_location{path_str, 0, "joint"}});
        }
        if (!link_names.contains(joint.child_link))
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::unknown_link_reference,
                .detail = "joint '" + joint.name + "' references unknown child link '" + joint.child_link + "'",
                .location = urdf_source_location{path_str, 0, "joint"}});
        }
        if (joint.parent_link == joint.child_link)
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::cyclic_kinematic_tree,
                .detail = "joint '" + joint.name + "' forms a self-loop: parent and child link are both '"
                    + joint.parent_link + "'",
                .location = urdf_source_location{path_str, 0, "joint"}});
        }
        if (!child_links.insert(joint.child_link).second)
        {
            return cartan::unexpected(urdf_error{
                .kind = urdf_failure::multi_parent_link,
                .detail = "link '" + joint.child_link + "' is the child of more than one joint",
                .location = urdf_source_location{path_str, 0, "joint"}});
        }

        if (auto origin = joint_node.child("origin"); origin)
        {
            auto pose = detail::parse_origin<Scalar>(origin);
            if (!pose.has_value())
            {
                return cartan::unexpected(urdf_error{
                    .kind = urdf_failure::malformed_xml,
                    .detail = "joint '" + joint.name + "' has malformed <origin>",
                    .location = urdf_source_location{path_str, 0, "origin"}});
            }
            joint.origin = std::move(pose.value());
        }

        if (auto axis_node = joint_node.child("axis"); axis_node)
        {
            vector3<Scalar> axis;
            if (auto attr = axis_node.attribute("xyz"); attr)
            {
                if (!detail::parse_triple<Scalar>(attr.value(), axis))
                {
                    return cartan::unexpected(urdf_error{
                        .kind = urdf_failure::malformed_xml,
                        .detail = "joint '" + joint.name + "' has malformed <axis>",
                        .location = urdf_source_location{path_str, 0, "axis"}});
                }
                joint.axis = axis;
            }
        }

        if (auto limit = joint_node.child("limit"); limit)
        {
            auto read_limit_attr = [&](const char* attr_name,
                                       std::optional<Scalar>& slot)
                -> std::optional<urdf_error> {
                auto attr = limit.attribute(attr_name);
                if (!attr) { return std::nullopt; }
                auto v = detail::as_finite_double(attr);
                if (!v.has_value())
                {
                    return urdf_error{
                        .kind = urdf_failure::non_finite_value,
                        .detail = "joint '" + joint.name + "' has a non-finite <limit "
                            + attr_name + ">",
                        .location = urdf_source_location{path_str, 0, "limit"}};
                }
                slot = static_cast<Scalar>(*v);
                return std::nullopt;
            };
            if (auto err = read_limit_attr("lower", joint.position_min)) { return cartan::unexpected(std::move(*err)); }
            if (auto err = read_limit_attr("upper", joint.position_max)) { return cartan::unexpected(std::move(*err)); }
            if (auto err = read_limit_attr("velocity", joint.velocity_max)) { return cartan::unexpected(std::move(*err)); }
            if (auto err = read_limit_attr("effort", joint.effort_max)) { return cartan::unexpected(std::move(*err)); }
        }

        model.joints.push_back(std::move(joint));
    }

    return model;
}

}

#endif
