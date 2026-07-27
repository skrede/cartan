#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_EXPECTED_HELPERS_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_EXPECTED_HELPERS_H

// The single loud-failure spelling shared by every test-side helper that
// consumes a checked factory. A fixture's geometry is a compile-time literal:
// a refusal is a bug in the fixture rather than a case under test, and falling
// through to a default-constructed value would turn that bug into a silently
// wrong robot. Aborting rather than throwing keeps the helper usable from the
// exceptions-off targets, where a throw would not compile.

#include <cartan/serial/chain/chain_failure.h>

#include <cartan/expected.h>

#include <cstdio>
#include <utility>
#include <cstdlib>

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
