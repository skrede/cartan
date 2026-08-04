#include <cartan/urdf.h>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>

/// Asserts what the default reading path grants: no evaluation backend, which
/// is the value that selects the description reader's own built-in evaluator.
/// The first case is the assertion; the other two are the controls that stop it
/// passing for the wrong reason. One fails if the xacro path stops working at
/// all, so the assertion cannot survive the options object losing the member it
/// reads. The other fails if a backend is ever wired in by default, because the
/// built-in evaluator refuses an expression that calls into the host language
/// and an evaluation backend performs it.
///
/// Whether the built library links a Python interpreter is a property of the
/// build rather than of a running process, so nothing here claims it; the
/// tracked build configuration is checked by tools/check_forbidden_calls.py.

namespace
{

std::filesystem::path fixture_path(const char* name)
{
    return std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / name;
}

}

TEST_CASE("evaluator: the default options select no evaluation backend", "[urdf_evaluator]")
{
    const cartan::load_options opts{};
    REQUIRE(opts.description.backend == nullptr);
}

TEST_CASE("evaluator: the built-in evaluator expands a xacro document", "[urdf_evaluator]")
{
    const cartan::load_options opts{};
    auto loaded = cartan::load_urdf<double>(fixture_path("xacro_minimal.urdf.xacro"), opts);

    REQUIRE(loaded.has_value());
    CHECK(loaded->metadata.joint_names.size() == 2);
}

TEST_CASE("evaluator: the default path refuses a host-language expression", "[urdf_evaluator]")
{
    const cartan::load_options opts{};
    auto loaded =
        cartan::load_urdf<double>(fixture_path("xacro_host_expression.urdf.xacro"), opts);

    REQUIRE_FALSE(loaded.has_value());
    REQUIRE(loaded.error().meios_code.has_value());
    CHECK(*loaded.error().meios_code == meios::diagnostic_code::unsupported_expression);
}
