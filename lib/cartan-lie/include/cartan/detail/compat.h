#ifndef HPP_GUARD_CARTAN_DETAIL_COMPAT_H
#define HPP_GUARD_CARTAN_DETAIL_COMPAT_H

#include <version>

// constexpr std::acos, std::asin, std::sqrt, etc.
// Available in GCC 26+/libstdc++ and libc++ with __cpp_lib_constexpr_cmath.
// On platforms without it, these functions are still callable at runtime.
#if defined(__cpp_lib_constexpr_cmath) && __cpp_lib_constexpr_cmath >= 202202L
#  define CARTAN_HAS_CONSTEXPR_CMATH 1
#  define CARTAN_CONSTEXPR_CMATH constexpr
#else
#  define CARTAN_HAS_CONSTEXPR_CMATH 0
#  define CARTAN_CONSTEXPR_CMATH
#endif

#endif
