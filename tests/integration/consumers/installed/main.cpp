#include <cartan/serial/ik/ik.h>

#include <type_traits>

using chain6 = cartan::kinematic_chain<double, 6>;

static_assert(std::is_same_v<cartan::lm<chain6>, cartan::builtin_lm<chain6, cartan::no_limits>>,
    "cartan::lm must name the native Levenberg-Marquardt solver in every configuration");
static_assert(
    std::is_same_v<cartan::lbfgsb<chain6>, cartan::builtin_lbfgsb<chain6, cartan::clamp_limits>>,
    "cartan::lbfgsb must name the native L-BFGS-B solver in every configuration");

#ifdef CONSUMER_EXPECTS_ARGMIN
static_assert(CARTAN_HAS_ARGMIN == 1, "linking the component must supply the feature macro");
#if !__has_include(<argmin/solver/step_budget_solver.h>)
#error "the backend include root did not reach a consumer that linked the component"
#endif
using backend_solver = cartan::argmin_lm<chain6>;
#else
#ifdef CARTAN_HAS_ARGMIN
#error "the backend feature macro reached a consumer that did not link the component"
#endif
#if __has_include(<argmin/solver/step_budget_solver.h>)
#error "the backend include root reached a consumer that did not link the component"
#endif
#endif

int main()
{
    return 0;
}
