#include "cartan/lie.h"
#include "cartan/serial_chain.h"

int main()
{
    // Verify umbrella headers include types and detail
    cartan::vector3<double> v = cartan::vector3<double>::Zero();
    (void)v;

    constexpr auto eps = cartan::detail::epsilon_v<double>;
    (void)eps;

    return 0;
}
