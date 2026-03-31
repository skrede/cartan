#include <liepp/frames/transform.h>
#include <liepp/frames/framed_twist.h>

struct A {};
struct B {};
struct C {};

int main()
{
    auto T = liepp::transform<A, B, double>::identity();
    liepp::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << 0.0, 0.0, 0.0;
    auto ft = liepp::framed_twist<C, double>{tw};
    auto result = liepp::adjoint_map(T, ft);
    return 0;
}
