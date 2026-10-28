/// @file ik_status_test.cpp
/// @brief Every solver state and every failure reason answers with its own
///        diagnostic text, and a name both enums carry is answered by the
///        overload the argument's type selects.
///
/// Both message() overloads are constexpr, so a caller that folds them at
/// compile time generates no code for a switch case to reach. Every assertion
/// here reads its enumerator out of an object at run time, which is what makes
/// the switches run at all.

#include <cartan/serial/ik/ik_status.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <string>
#include <cstddef>

namespace spp = cartan;

namespace
{

// Every enumerator, listed once. A value added to either enum without a line
// here leaves its case unexercised, which is the silence these tests exist to
// refuse.
constexpr std::array k_statuses{
    spp::ik_status::running,
    spp::ik_status::converged,
    spp::ik_status::diverged,
    spp::ik_status::stalled,
    spp::ik_status::joint_limit_hit,
    spp::ik_status::iteration_limit,
    spp::ik_status::aborted,
    spp::ik_status::not_initialized,
    spp::ik_status::dimension_mismatch,
    spp::ik_status::non_finite_input,
    spp::ik_status::unsupported_configuration};

constexpr std::array k_failures{
    spp::ik_failure::diverged,
    spp::ik_failure::stalled,
    spp::ik_failure::iteration_limit,
    spp::ik_failure::joint_limit_violation,
    spp::ik_failure::aborted,
    spp::ik_failure::not_initialized,
    spp::ik_failure::dimension_mismatch,
    spp::ik_failure::non_finite_input,
    spp::ik_failure::unsupported_configuration};

// The runner maps a latched setup precondition onto the failure it reports, so
// both enums name the same three.
constexpr std::array k_shared_statuses{
    spp::ik_status::dimension_mismatch,
    spp::ik_status::non_finite_input,
    spp::ik_status::unsupported_configuration};

constexpr std::array k_shared_failures{
    spp::ik_failure::dimension_mismatch,
    spp::ik_failure::non_finite_input,
    spp::ik_failure::unsupported_configuration};

template <typename Enum, std::size_t N>
std::array<std::string, N> diagnostics(const std::array<Enum, N>& values)
{
    std::array<std::string, N> seen;
    for (std::size_t i = 0; i < N; ++i)
    {
        seen[i] = spp::message(values[i]);
    }
    return seen;
}

template <std::size_t N>
void check_each_is_named_once(const std::array<std::string, N>& seen)
{
    for (std::size_t i = 0; i < N; ++i)
    {
        INFO("enumerator index " << i);
        CHECK_FALSE(seen[i].empty());
        for (std::size_t j = i + 1; j < N; ++j)
        {
            INFO("enumerators " << i << " and " << j << " share the text " << seen[i]);
            CHECK(seen[i] != seen[j]);
        }
    }
}

}

// One diagnostic shared by two states cannot say which state the solver
// reached, so distinctness is the property, not non-emptiness.
TEST_CASE("every solver status carries its own non-empty diagnostic",
    "[ik][diagnostics][ik_status]")
{
    check_each_is_named_once(diagnostics(k_statuses));
}

TEST_CASE("every solver failure carries its own non-empty diagnostic",
    "[ik][diagnostics][ik_status]")
{
    check_each_is_named_once(diagnostics(k_failures));
}

TEST_CASE("a precondition both enums name answers through each enum's own switch",
    "[ik][diagnostics][ik_status]")
{
    const std::array from_status = diagnostics(k_shared_statuses);
    const std::array from_failure = diagnostics(k_shared_failures);

    for (std::size_t i = 0; i < from_status.size(); ++i)
    {
        INFO("shared precondition index " << i);
        CHECK_FALSE(from_status[i].empty());
        CHECK_FALSE(from_failure[i].empty());
    }
}

// not_initialized is the one name both enums carry whose two answers differ:
// a solver stepped before setup is not a solve requested before setup. Reading
// the two apart is possible only if the argument's type, and not the shared
// name, selects the overload.
TEST_CASE("the overload the argument's type selects is the one whose text is read",
    "[ik][diagnostics][ik_status]")
{
    const std::array stepped{spp::ik_status::not_initialized};
    const std::array requested{spp::ik_failure::not_initialized};

    CHECK(diagnostics(stepped)[0] != diagnostics(requested)[0]);
}
