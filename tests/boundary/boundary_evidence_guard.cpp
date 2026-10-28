#include <cstdio>

// Compiled in the same configuration as the boundary evidence and reading the
// sanitizer for itself, so a configuration that ought to carry a group of
// evidence but registered less of it than it declares fails here instead of
// reporting an empty pass. The registered counts are derived from the
// registrations themselves; only the expectations are written down.
//
// A whole group deregistering is not the same defect as one case being deleted
// from it, and a count alone does not tell them apart. Where a configure-time
// probe gates the group, this binary sees the feature the probe was asked about,
// so the probe must have disagreed; naming it and the way it is known to
// disagree wrongly is the part an operator cannot recover from the ctest output.
// A group nothing gates carries no probe and says so.
static int check(const char* group, const char* probe, const char* cause, int registered,
    int expected)
{
    if (registered == expected)
    {
        return 0;
    }
    std::printf("%s: registered %d, expected %d\n", group, registered, expected);
    if (probe == nullptr)
    {
        std::printf("  %s\n", cause);
    }
    else if (registered == 0 && expected > 0)
    {
        std::printf("  this binary is built with the feature, so the configure-time probe %s\n"
                    "  must have succeeded and did not; re-run cmake and read its result.\n"
                    "  %s\n",
            probe, cause);
    }
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

    int failures = check("sanitizer evidence", "CARTAN_BOUNDARY_ADDRESS_SANITIZER",
        "It runs the probe it compiles, so it also fails where an address-sanitized process "
        "cannot start at all -- a kernel configured for high-entropy ASLR aborts one before "
        "main, and lowering vm.mmap_rnd_bits to 28 is the usual remedy.",
        CARTAN_REGISTERED_EVIDENCE_TESTS,
        address_sanitizer ? CARTAN_EXPECTED_EVIDENCE_TESTS : 0);
    failures += check("differential evidence", nullptr,
        "Nothing gates this group: it compiles its own assertions out and is registered in every "
        "configuration, so the shortfall is a case deleted from the loop that registers it.",
        CARTAN_REGISTERED_DIFFERENTIAL_TESTS, CARTAN_EXPECTED_DIFFERENTIAL_TESTS);
    return failures;
}
