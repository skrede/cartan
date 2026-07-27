#include <cstdio>

// Compiled in the same configuration as the boundary evidence and reading the
// sanitizer for itself, so a configuration that ought to carry the sanitizer
// evidence but registered none of it fails here instead of reporting an empty
// pass.
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

    const int expected = address_sanitizer ? CARTAN_EXPECTED_EVIDENCE_TESTS : 0;
    if (CARTAN_REGISTERED_EVIDENCE_TESTS != expected)
    {
        std::printf("registered %d sanitizer evidence tests, expected %d "
                    "(address sanitizer: %d)\n",
            CARTAN_REGISTERED_EVIDENCE_TESTS, expected,
            static_cast<int>(address_sanitizer));
        return 1;
    }
    return 0;
}
