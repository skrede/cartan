#include "holder.h"

#include <type_traits>

static_assert(std::is_same_v<cartan::lm<chain6>, cartan::builtin_lm<chain6, cartan::no_limits>>,
    "cartan::lm must name the native Levenberg-Marquardt implementation in every configuration");

static_assert(
    std::is_same_v<cartan::lbfgsb<chain6>, cartan::builtin_lbfgsb<chain6, cartan::clamp_limits>>,
    "cartan::lbfgsb must name the native L-BFGS-B implementation in every configuration");
