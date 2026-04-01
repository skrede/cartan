#include "liepp/lie.h"
#include "liepp/serial_chain.h"

int main()
{
    // Verify umbrella headers include types and detail
    liepp::vector3<double> v = liepp::vector3<double>::Zero();
    (void)v;

    constexpr auto eps = liepp::detail::epsilon_v<double>;
    (void)eps;

    return 0;
}
