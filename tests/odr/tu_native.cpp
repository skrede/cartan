#include "holder.h"

#ifdef CARTAN_ODR_BACKEND_LINKED
#error "this unit must be built without an optional backend component; with one, both units of the gate share a configuration and their comparison is vacuous"
#endif

std::size_t holder_size_native()
{
    return sizeof(holder);
}
