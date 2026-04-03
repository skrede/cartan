#ifndef HPP_GUARD_LIEPP_DETAIL_COMPAT_H
#define HPP_GUARD_LIEPP_DETAIL_COMPAT_H

#include <version>

// constexpr std::acos, std::asin, std::sqrt, etc.
// Available in GCC 26+/libstdc++ and libc++ with __cpp_lib_constexpr_cmath.
// On platforms without it, these functions are still callable at runtime.
#if defined(__cpp_lib_constexpr_cmath) && __cpp_lib_constexpr_cmath >= 202202L
#  define LIEPP_HAS_CONSTEXPR_CMATH 1
#  define LIEPP_CONSTEXPR_CMATH constexpr
#else
#  define LIEPP_HAS_CONSTEXPR_CMATH 0
#  define LIEPP_CONSTEXPR_CMATH
#endif

#endif
