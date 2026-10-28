// The control for the weighted-wrapper rejection: the same call over an inner
// policy that does apply a weight. Without this, the rejection beside it could
// come from the fixture rather than from the constraint.
#include "restart_wrapper_weight_slice.h"

#include "cartan/serial/ik/solver/projected_lm.h"

int main()
{
    using chain_type = cartan::compile_gate::chain_type;
    cartan::restart_wrapper<chain_type, cartan::projected_lm<chain_type>> wrapper;
    return cartan::compile_gate::setup_with_weight(wrapper);
}
