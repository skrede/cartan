#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_EXPECTED_HELPERS_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_EXPECTED_HELPERS_H

// The single loud-failure spelling shared by every test-side helper that
// consumes a checked factory. Aborting rather than throwing keeps the helper
// usable from the exceptions-off targets, where a throw would not compile.
//
// This is only sound where every input is a compile-time literal, so that a
// refusal is a bug in the fixture rather than a case under test. A helper whose
// input arrives at runtime -- a solver's joint vector, a drawn configuration, a
// caller's parameter -- must not use it: taking the process down would discard
// whatever the caller had already accumulated. Those helpers report and answer
// with a value, or hand the fallible result back.

#include <cartan/serial/chain/chain_failure.h>

#include <cartan/expected.h>

#include <cstdio>
#include <cstdlib>
#include <utility>

namespace cartan::testing
{

/// message() is left unqualified so argument-dependent lookup picks the
/// overload belonging to whichever failure enum E happens to be.
template <typename T, typename E>
T unwrap(cartan::expected<T, E> held, const char* what)
{
    if (!held.has_value())
    {
        std::fprintf(stderr, "%s: %s\n", what, message(held.error()));
        std::abort();
    }
    return std::move(*held);
}

}

#endif
