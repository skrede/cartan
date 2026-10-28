// The control for the compile-fail harness: a well-formed program using the
// same headers and the same include plumbing as the rejections beside it. If
// this stops compiling, those rejections stop meaning anything.
#include <cartan/serial/chain/joint_limits.h>

int main()
{
    auto lim = cartan::joint_limits<double>::make(-1.0, 1.0).value();
    return lim.position_min() < 0.0 ? 0 : 1;
}
