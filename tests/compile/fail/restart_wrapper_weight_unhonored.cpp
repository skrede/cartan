// A task weight handed to a wrapper whose inner policy cannot apply one must
// not compile. Levenberg-Marquardt takes no weight, so the wrapper's weighted
// setup does not exist for it; accepting the call would discard the weight
// silently and return a plausible answer computed without it.
#include "restart_wrapper_weight_slice.h"

#include "cartan/serial/ik/solver/lm.h"

int main()
{
    using chain_type = cartan::compile_gate::chain_type;
    cartan::restart_wrapper<chain_type, cartan::lm<chain_type>> wrapper;
    return cartan::compile_gate::setup_with_weight(wrapper);
}
