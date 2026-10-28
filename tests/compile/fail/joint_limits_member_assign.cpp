// Naming the data member directly must not compile either.
#include <cartan/serial/chain/joint_limits.h>

int main()
{
    auto lim = cartan::joint_limits<double>::make(-1.0, 1.0).value();
    lim.m_position_max = 2.0;
    return 0;
}
