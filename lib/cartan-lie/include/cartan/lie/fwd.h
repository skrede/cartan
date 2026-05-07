#ifndef HPP_GUARD_CARTAN_LIE_FWD_H
#define HPP_GUARD_CARTAN_LIE_FWD_H

#include <cstddef>
#include <type_traits>

namespace cartan
{

// Policy tags (defined in policy.h)
struct strict_policy;
struct fast_policy;

// Lie group forward declarations
template <typename Scalar, typename Policy = strict_policy>
class so2;

template <typename Scalar, typename Policy = strict_policy>
class se2;

template <typename Scalar, typename Policy = strict_policy>
class so3;

template <typename Scalar, typename Policy = strict_policy>
class se3;

}

#endif
