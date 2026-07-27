#include <cartan/serial/ik/ik.h>

#include <type_traits>

using chain6 = cartan::kinematic_chain<double, 6>;

static_assert(std::is_same_v<cartan::lm<chain6>, cartan::builtin_lm<chain6, cartan::no_limits>>,
    "cartan::lm must name the native Levenberg-Marquardt solver in every configuration");

#ifdef CONSUMER_EXPECTS_ARGMIN
static_assert(CARTAN_HAS_ARGMIN == 1, "linking the component must supply the feature macro");
using argmin_backend_solver = cartan::argmin_lm<chain6>;
#endif

#ifdef CONSUMER_EXPECTS_NLOPT
static_assert(CARTAN_HAS_NLOPT == 1, "linking the component must supply the feature macro");
using nlopt_backend_solver = cartan::nlopt_slsqp<chain6>;
#endif

int main()
{
    return 0;
}
