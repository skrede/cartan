#include <cartan/urdf.h>

#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <filesystem>

using Catch::Approx;

namespace
{

std::filesystem::path fixture_path(const char* name)
{
    return std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / name;
}

}

TEST_CASE("extractor: serial minimal URDF builds chain with expected joint count and axes",
          "[urdf_chain_extractor]")
{
    auto result = cartan::load_urdf<double>(fixture_path("extractor_serial_minimal.urdf"));

    REQUIRE(result.has_value());
    const auto& chain = result->chain;
    const auto& meta = result->metadata;

    REQUIRE(chain.num_joints() == 3);
    REQUIRE(chain.limits().size() == 3);

    // shoulder: revolute around world z at (0,0,0.1).
    const auto& s1 = chain.axes()[0];
    CHECK(s1.is_revolute());
    CHECK(s1.omega()(2) == Approx(1.0).margin(1e-12));
    // v = -omega x point = -(0,0,1) x (0,0,0.1) = (0,0,0).
    CHECK(s1.v().norm() == Approx(0.0).margin(1e-12));

    // extender: prismatic along world z (independent of joint configurations
    // because all joint origins are pure translations along z and the rotation
    // accumulator stays identity at zero configuration).
    const auto& s2 = chain.axes()[1];
    CHECK(s2.is_prismatic());
    CHECK(s2.v()(2) == Approx(1.0).margin(1e-12));

    // wrist: revolute around world z at (0,0, 0.1+0.2+0.05) = (0,0,0.35).
    const auto& s3 = chain.axes()[2];
    CHECK(s3.is_revolute());
    CHECK(s3.omega()(2) == Approx(1.0).margin(1e-12));
    // v = -(0,0,1) x (0,0,0.35) = (0,0,0).
    CHECK(s3.v().norm() == Approx(0.0).margin(1e-12));

    REQUIRE(meta.joint_names.size() == 3);
    CHECK(meta.joint_names[0] == "shoulder");
    CHECK(meta.joint_names[1] == "extender");
    CHECK(meta.joint_names[2] == "wrist");
    CHECK(meta.base_link_name == "base_link");
    CHECK(meta.tool_link_name == "tool");
}

TEST_CASE("extractor: continuous joint yields kinematic_chain with infinity limits at the wrist entry",
          "[urdf_chain_extractor]")
{
    auto result = cartan::load_urdf<double>(fixture_path("extractor_continuous_wrist.urdf"));

    REQUIRE(result.has_value());
    const auto& chain = result->chain;
    REQUIRE(chain.num_joints() == 3);

    const auto& wrist_limits = chain.limits()[2];
    CHECK(wrist_limits.position_min == -std::numeric_limits<double>::infinity());
    CHECK(wrist_limits.position_max == +std::numeric_limits<double>::infinity());

    // Bounded joints remain finite.
    CHECK(std::isfinite(chain.limits()[0].position_min));
    CHECK(std::isfinite(chain.limits()[0].position_max));
    CHECK(std::isfinite(chain.limits()[1].position_min));
    CHECK(std::isfinite(chain.limits()[1].position_max));
}

TEST_CASE("extractor: fixed-joint merge composes offsets into the downstream screw_axis origin",
          "[urdf_chain_extractor]")
{
    auto result = cartan::load_urdf<double>(fixture_path("extractor_fixed_merge.urdf"));

    REQUIRE(result.has_value());
    const auto& chain = result->chain;
    REQUIRE(chain.num_joints() == 2);

    // Hand-computed ground truth:
    // shoulder origin (0,0,0.1), then fixed sensor_mount with translation
    // (0.1, 0, 0.2) and rotation 30deg about z, then elbow at local origin (0,0,0).
    // World joint pose of elbow:
    //   T_acc = T_shoulder * T_sensor_mount * T_elbow_local
    //         = (I, (0,0,0.1)) * (R_z(30), (0.1, 0, 0.2)) * (I, 0)
    //         = (R_z(30), I.act((0.1,0,0.2)) + (0,0,0.1))
    //         = (R_z(30), (0.1, 0, 0.3))
    // Elbow axis local = (0,0,1) -> world = R_z(30) * (0,0,1) = (0,0,1).
    // Elbow point world = (0.1, 0, 0.3).
    // Revolute screw: omega=(0,0,1), v = -(0,0,1) x (0.1, 0, 0.3) = (0, -0.1, 0).

    const auto& s_elbow = chain.axes()[1];
    CHECK(s_elbow.is_revolute());
    CHECK(s_elbow.omega()(0) == Approx(0.0).margin(1e-12));
    CHECK(s_elbow.omega()(1) == Approx(0.0).margin(1e-12));
    CHECK(s_elbow.omega()(2) == Approx(1.0).margin(1e-12));
    CHECK(s_elbow.v()(0) == Approx(0.0).margin(1e-12));
    CHECK(s_elbow.v()(1) == Approx(-0.1).margin(1e-12));
    CHECK(s_elbow.v()(2) == Approx(0.0).margin(1e-12));

    // Home pose: T_acc_after_elbow = (R_z(30), (0.1, 0, 0.3))
    //   * tool_mount fixed origin (I shifted by (0,0,0.05))
    //   = (R_z(30), R_z(30).act((0,0,0.05)) + (0.1, 0, 0.3))
    //   = (R_z(30), (0.1, 0, 0.35)).
    const auto& home = chain.home();
    CHECK(home.translation()(0) == Approx(0.1).margin(1e-12));
    CHECK(home.translation()(1) == Approx(0.0).margin(1e-12));
    CHECK(home.translation()(2) == Approx(0.35).margin(1e-12));

    // Rotation: 30 degrees about z.
    const double cos30 = std::cos(0.5235987755982988);
    const double sin30 = std::sin(0.5235987755982988);
    const auto R = home.rotation().matrix();
    CHECK(R(0, 0) == Approx(cos30).margin(1e-12));
    CHECK(R(1, 0) == Approx(sin30).margin(1e-12));
    CHECK(R(2, 2) == Approx(1.0).margin(1e-12));
}

TEST_CASE("extractor: branched URDF returns branched_kinematic_tree with detail",
          "[urdf_chain_extractor]")
{
    auto result = cartan::load_urdf<double>(fixture_path("extractor_branched.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::branched_kinematic_tree);
    // Detail should name at least one of the branched children.
    const auto& detail = result.error().detail;
    CHECK((detail.find("arm_a") != std::string::npos
        || detail.find("arm_b") != std::string::npos));
}

TEST_CASE("extractor: load_options.base_link missing in URDF returns link_not_found",
          "[urdf_chain_extractor]")
{
    cartan::load_options opts{};
    opts.base_link = "ghost";
    auto result = cartan::load_urdf<double>(fixture_path("extractor_serial_minimal.urdf"), opts);

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::link_not_found);
    CHECK(result.error().detail == "ghost");
}

TEST_CASE("extractor: load_options.tool_link missing in URDF returns link_not_found",
          "[urdf_chain_extractor]")
{
    cartan::load_options opts{};
    opts.tool_link = "phantom";
    auto result = cartan::load_urdf<double>(fixture_path("extractor_serial_minimal.urdf"), opts);

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::link_not_found);
    CHECK(result.error().detail == "phantom");
}

TEST_CASE("extractor: load_sdf returns sdf_not_supported",
          "[urdf_chain_extractor]")
{
    auto result = cartan::load_sdf<double>(fixture_path("extractor_serial_minimal.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::sdf_not_supported);
}

TEST_CASE("load_urdf: malformed URDF surfaces parser error through the composer",
          "[urdf_chain_extractor]")
{
    auto result = cartan::load_urdf<double>(fixture_path("parser_unclosed.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::malformed_xml);
}

TEST_CASE("load_urdf: unsupported joint type composes through",
          "[urdf_chain_extractor]")
{
    auto result = cartan::load_urdf<double>(fixture_path("parser_unknown_joint.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::unsupported_joint_type);
}

// --- Strict-reject semantic-gap cases (build-time, valid serial tree) ---
//
// These fixtures form a single valid chain, so the extractor terminates; the
// pre-fix defect is a silently-wrong chain (frozen limits, NaN axis, or the
// wrong default axis), which the assertions below catch. None of these hang.

TEST_CASE("extractor: revolute joint without <limit> is rejected", "[urdf_chain_extractor][urdf_strict]")
{
    auto result = cartan::load_urdf<double>(fixture_path("adversarial_revolute_no_limit.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::missing_joint_limit);
    CHECK(result.error().detail.find("unbounded_revolute") != std::string::npos);
}

TEST_CASE("extractor: prismatic joint without <limit> is rejected", "[urdf_chain_extractor][urdf_strict]")
{
    auto result = cartan::load_urdf<double>(fixture_path("adversarial_prismatic_no_limit.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::missing_joint_limit);
    CHECK(result.error().detail.find("unbounded_prismatic") != std::string::npos);
}

TEST_CASE("extractor: zero-magnitude axis is rejected", "[urdf_chain_extractor][urdf_strict]")
{
    auto result = cartan::load_urdf<double>(fixture_path("adversarial_zero_axis.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::zero_axis);
    CHECK(result.error().detail.find("zero_axis_joint") != std::string::npos);
}

TEST_CASE("extractor: omitted <axis> defaults to world +X, not +Z", "[urdf_chain_extractor][urdf_strict]")
{
    auto result = cartan::load_urdf<double>(fixture_path("semantic_axis_omitted.urdf"));

    REQUIRE(result.has_value());
    const auto& chain = result->chain;
    REQUIRE(chain.num_joints() == 1);

    const auto& s = chain.axes()[0];
    CHECK(s.is_revolute());
    // URDF spec default axis is (1,0,0). The joint origin is identity, so the
    // world-frame screw axis must be exactly world +X.
    CHECK(s.omega()(0) == Approx(1.0).margin(1e-12));
    CHECK(s.omega()(1) == Approx(0.0).margin(1e-12));
    CHECK(s.omega()(2) == Approx(0.0).margin(1e-12));
}

TEST_CASE("extractor: continuous joint without <limit> stays valid (unlimited)", "[urdf_chain_extractor][urdf_strict]")
{
    auto result = cartan::load_urdf<double>(fixture_path("semantic_continuous_no_limit.urdf"));

    REQUIRE(result.has_value());
    const auto& chain = result->chain;
    REQUIRE(chain.num_joints() == 1);

    const auto& lim = chain.limits()[0];
    CHECK(lim.position_min == -std::numeric_limits<double>::infinity());
    CHECK(lim.position_max == +std::numeric_limits<double>::infinity());
}

// --- Kinematic-cycle DoS (build-time walk) ---
//
// These cases would hang the unbounded pre-fix walk, so they are tagged
// [urdf_dos] and excluded from the pre-fix red run. Post-fix the walk is
// bounded and the parser rejects the cycle before the walk is even reached.

TEST_CASE("extractor: kinematic cycle does not hang and is rejected", "[urdf_chain_extractor][urdf_strict][urdf_dos]")
{
    auto result = cartan::load_urdf<double>(fixture_path("adversarial_cycle.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(!result.error().detail.empty());
}

TEST_CASE("extractor: bounded walk rejects a cyclic parsed_model", "[urdf_chain_extractor][urdf_strict][urdf_dos]")
{
    // Hand-built cyclic model that bypasses the parser's non-tree guard: the
    // walk base -> a -> b -> a would loop forever without the step bound.
    using namespace cartan::urdf;
    parsed_model<double> model{};
    model.robot_name = "cyclic";
    model.links.push_back(parsed_link<double>{"base_link", {}});
    model.links.push_back(parsed_link<double>{"link_a", {}});
    model.links.push_back(parsed_link<double>{"link_b", {}});

    auto make_joint = [](const char* name, const char* parent, const char* child) {
        parsed_joint<double> j{};
        j.name = name;
        j.kind = parsed_joint_kind::revolute;
        j.parent_link = parent;
        j.child_link = child;
        j.position_min = -3.14159;
        j.position_max = 3.14159;
        return j;
    };
    model.joints.push_back(make_joint("root_joint", "base_link", "link_a"));
    model.joints.push_back(make_joint("forward_joint", "link_a", "link_b"));
    model.joints.push_back(make_joint("closing_joint", "link_b", "link_a"));

    auto result = build_chain<double>(model);

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == urdf_failure::cyclic_kinematic_tree);
    CHECK(!result.error().detail.empty());
}

TEST_CASE("extractor: tool_link off the serial walk path is rejected", "[urdf_chain_extractor][urdf_strict]")
{
    // Valid serial chain base -> a -> tip, plus a fixed-leaf world-frame marker
    // hanging off base that the walk elides. Requesting the marker as the tool
    // link cannot be honored: the walk never reaches it.
    using namespace cartan::urdf;
    parsed_model<double> model{};
    model.robot_name = "off_path_tool";
    model.links.push_back(parsed_link<double>{"base_link", {}});
    model.links.push_back(parsed_link<double>{"link_a", {}});
    model.links.push_back(parsed_link<double>{"tip", {}});
    model.links.push_back(parsed_link<double>{"marker", {}});

    auto revolute = [](const char* name, const char* parent, const char* child) {
        parsed_joint<double> j{};
        j.name = name;
        j.kind = parsed_joint_kind::revolute;
        j.parent_link = parent;
        j.child_link = child;
        j.position_min = -1.0;
        j.position_max = 1.0;
        return j;
    };
    parsed_joint<double> marker_joint{};
    marker_joint.name = "marker_mount";
    marker_joint.kind = parsed_joint_kind::fixed;
    marker_joint.parent_link = "base_link";
    marker_joint.child_link = "marker";

    model.joints.push_back(revolute("base_joint", "base_link", "link_a"));
    model.joints.push_back(revolute("arm_joint", "link_a", "tip"));
    model.joints.push_back(marker_joint);

    load_options opts{};
    opts.tool_link = "marker";
    auto result = build_chain<double>(model, opts);

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == urdf_failure::tool_link_unreachable);
    CHECK(result.error().detail.find("marker") != std::string::npos);
}
