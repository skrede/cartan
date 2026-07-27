#include <cstdio>

// Compiled in the same configuration as the boundary evidence and reading the
// sanitizer and NDEBUG for itself, so a configuration that ought to carry a
// group of evidence but registered less of it than it declares fails here
// instead of reporting an empty pass. The registered counts are derived from
// the registrations themselves; only the expectations are written down.
static int check(const char* group, int registered, int expected)
{
    if (registered == expected)
    {
        return 0;
    }
    std::printf("%s: registered %d, expected %d\n", group, registered, expected);
    return 1;
}

int main()
{
#if defined(__SANITIZE_ADDRESS__)
    const bool address_sanitizer = true;
#elif defined(__has_feature)
#  if __has_feature(address_sanitizer)
    const bool address_sanitizer = true;
#  else
    const bool address_sanitizer = false;
#  endif
#else
    const bool address_sanitizer = false;
#endif

#ifdef NDEBUG
    const bool assertions_off = true;
#else
    const bool assertions_off = false;
#endif

    int failures = check("sanitizer evidence", CARTAN_REGISTERED_EVIDENCE_TESTS,
        address_sanitizer ? CARTAN_EXPECTED_EVIDENCE_TESTS : 0);
    failures += check("differential evidence", CARTAN_REGISTERED_DIFFERENTIAL_TESTS,
        assertions_off ? CARTAN_EXPECTED_DIFFERENTIAL_TESTS : 0);
    return failures;
}
