#include <cartan/frames/transform.h>
#include <cartan/frames/framed_twist.h>

struct A {};
struct B {};
struct C {};

int main()
{
    auto T = cartan::transform<A, B, double>::identity();
    cartan::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << 0.0, 0.0, 0.0;
    auto ft = cartan::framed_twist<C, double>{tw};
    auto result = cartan::adjoint_map(T, ft);
    return 0;
}
