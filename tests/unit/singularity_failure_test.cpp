#include <cartan/serial/fk/singularity_failure.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <string>

namespace spp = cartan;

namespace
{

// Every enumerator, listed once. A value added to the enum without a line here
// leaves the new case unexercised, which is the same silence the header's
// default-less switches exist to refuse at compile time.
constexpr std::array k_failures{
    spp::singularity_failure::zero_spectrum,
    spp::singularity_failure::invalid_configuration,
    spp::singularity_failure::invalid_length};

}

// The header's whole reason for existing is that one shared absence cannot say
// which question went unanswered. That only holds if the names carry distinct
// text, so distinctness is the property, not non-emptiness.
TEST_CASE("every singularity failure carries its own non-empty diagnostic",
    "[ik][diagnostics][singularity_failure]")
{
    std::array<std::string, k_failures.size()> seen;

    for (std::size_t i = 0; i < k_failures.size(); ++i)
    {
        seen[i] = spp::message(k_failures[i]);
        INFO("enumerator index " << i);
        CHECK_FALSE(seen[i].empty());
    }

    for (std::size_t i = 0; i < seen.size(); ++i)
    {
        for (std::size_t j = i + 1; j < seen.size(); ++j)
        {
            INFO("enumerators " << i << " and " << j << " share the text " << seen[i]);
            CHECK(seen[i] != seen[j]);
        }
    }
}

// A binding layer raises for the caller's mistake and answers an absence for a
// measure that is undefined at a well-formed question, so this classification
// decides which of the two a Python caller sees.
TEST_CASE("the failure classification separates a caller's mistake from an undefined measure",
    "[ik][diagnostics][singularity_failure]")
{
    CHECK_FALSE(spp::is_invalid_argument(spp::singularity_failure::zero_spectrum));

    CHECK(spp::is_invalid_argument(spp::singularity_failure::invalid_configuration));
    CHECK(spp::is_invalid_argument(spp::singularity_failure::invalid_length));
}

// Both functions are constexpr, and a caller that folds them at compile time
// generates no code for the runtime cases above to reach. Pinning one of each
// keeps the constexpr promise from decaying into a claim nothing checks.
TEST_CASE("the diagnostic and its classification are usable in a constant expression",
    "[ik][diagnostics][singularity_failure]")
{
    static_assert(spp::is_invalid_argument(spp::singularity_failure::invalid_length));
    static_assert(!spp::is_invalid_argument(spp::singularity_failure::zero_spectrum));
    static_assert(spp::message(spp::singularity_failure::invalid_length)[0] != '\0');

    SUCCEED("the assertions above are compile-time");
}
