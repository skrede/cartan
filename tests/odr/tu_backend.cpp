#include "holder.h"

#ifndef CARTAN_ODR_BACKEND_LINKED
#error "this unit must be built with an optional backend component linked; without one, both units of the gate share a configuration and their comparison is vacuous"
#endif

std::size_t holder_size_backend()
{
    return sizeof(holder);
}

/// Assigns this unit's view of the solver into an object the caller laid out by
/// its own view. Well defined exactly when the two views agree, which is why
/// the gate calls it only then; under a disagreement it is the overrun the gate
/// exists to forbid, and it is what makes the mangled name cross the boundary.
void touch(holder& h)
{
    h.solver = cartan::lm<chain6>{};
}
