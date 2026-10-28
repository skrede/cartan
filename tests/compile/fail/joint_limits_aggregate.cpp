// The aggregate spelling must not compile: joint_limits has no public
// constructor, so an unvalidated set of bounds has no way in.
#include <cartan/serial/chain/joint_limits.h>

int main()
{
    cartan::joint_limits<double> lim{1.0, -1.0};
    return lim.position_min() < 0.0 ? 0 : 1;
}
