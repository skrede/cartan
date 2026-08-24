// Assigning through the old field spelling must not compile. This is the case a
// grep over construction sites cannot see: it is a mutation, not a construction,
// and it is what makes the factory the single path rather than merely the first.
#include <cartan/serial/chain/joint_limits.h>

int main()
{
    auto lim = cartan::joint_limits<double>::make(-1.0, 1.0).value();
    lim.position_min = 2.0;
    return 0;
}
