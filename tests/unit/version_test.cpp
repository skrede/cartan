#include "cartan/version.h"

#include <catch2/catch_test_macros.hpp>

#include <string_view>

TEST_CASE("cartan::version reports a non-empty string", "[version]")
{
    constexpr std::string_view v = cartan::version();
    REQUIRE_FALSE(v.empty());
}

TEST_CASE("cartan::version components are non-negative", "[version]")
{
    REQUIRE(cartan::version_major() >= 0);
    REQUIRE(cartan::version_minor() >= 0);
    REQUIRE(cartan::version_patch() >= 0);
}

TEST_CASE("cartan::version string matches major.minor.patch", "[version]")
{
    constexpr std::string_view v = cartan::version();

    const auto first_dot = v.find('.');
    REQUIRE(first_dot != std::string_view::npos);

    const auto second_dot = v.find('.', first_dot + 1);
    REQUIRE(second_dot != std::string_view::npos);

    const auto parse = [](std::string_view s) {
        int n = 0;
        for (char c : s) {
            REQUIRE(c >= '0');
            REQUIRE(c <= '9');
            n = n * 10 + (c - '0');
        }
        return n;
    };

    REQUIRE(parse(v.substr(0, first_dot)) == cartan::version_major());
    REQUIRE(parse(v.substr(first_dot + 1, second_dot - first_dot - 1)) == cartan::version_minor());
    REQUIRE(parse(v.substr(second_dot + 1)) == cartan::version_patch());
}

TEST_CASE("cartan::version macros match accessor return values", "[version]")
{
    REQUIRE(CARTAN_VERSION_MAJOR == cartan::version_major());
    REQUIRE(CARTAN_VERSION_MINOR == cartan::version_minor());
    REQUIRE(CARTAN_VERSION_PATCH == cartan::version_patch());

    constexpr std::string_view macro_string = CARTAN_VERSION_STRING;
    REQUIRE(macro_string == cartan::version());
}
