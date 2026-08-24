// MSVC reports 199711L in __cplusplus unless /Zc:__cplusplus is passed, and
// carries the standard actually in force in _MSVC_LANG instead.
#if defined(_MSVC_LANG)
static_assert(_MSVC_LANG == 202002L,
    "cartan compiles at exactly C++20; the standard in force is not 202002L");
#else
static_assert(__cplusplus == 202002L,
    "cartan compiles at exactly C++20; the standard in force is not 202002L");
#endif

int main()
{
    return 0;
}
